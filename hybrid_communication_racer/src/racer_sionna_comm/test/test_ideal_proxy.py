#!/usr/bin/env python3
"""Exercise the serialized ROS boundary without touching message contents."""

import os
import json
import random
import re
import signal
import subprocess
import time

import pytest
import rclpy
from nav_msgs.msg import Odometry
from racer_fidelity_msgs.msg import ChunkData, DroneState
from racer_recovery_core.msg import RecoveryCommand, RecoveryStatus
from racer_sionna_interfaces.msg import LinkQuality, LinkQualityArray
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


def configure_ros_domain(test_offset: int, random_low: int, random_high: int) -> None:
    """Use a caller-reserved domain range when integration tests run in parallel."""
    base = os.environ.get("RACER_TEST_ROS_DOMAIN_ID_BASE")
    domain_id = int(base) + test_offset if base is not None else random.randint(
        random_low, random_high
    )
    if not 0 <= domain_id <= 232:
        raise ValueError(f"invalid ROS domain ID for test: {domain_id}")
    os.environ["ROS_DOMAIN_ID"] = str(domain_id)


@pytest.mark.timeout(30)
def test_ideal_proxy_preserves_order_and_excludes_self():
    configure_ros_domain(0, 201, 229)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=2",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_ideal_proxy_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = []
    self_received = []
    node.create_subscription(
        DroneState,
        "/racer_sionna/rx/drone_1/drone_state",
        lambda message: received.append((message.stamp, list(message.grid_ids))),
        qos,
    )
    node.create_subscription(
        DroneState,
        "/racer_sionna/rx/drone_0/drone_state",
        lambda message: self_received.append(message.stamp),
        qos,
    )
    publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1

        for sequence in (1, 2, 3):
            message = DroneState()
            message.drone_id = 1
            message.stamp = float(sequence)
            message.grid_ids = [sequence, sequence + 10]
            publisher.publish(message)

        deadline = time.monotonic() + 8.0
        while len(received) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received == [
            (1.0, [1, 11]),
            (2.0, [2, 12]),
            (3.0, [3, 13]),
        ]
        assert self_received == []
        # Allow the proxy's one-second statistics timer to emit a snapshot
        # before SIGINT so the fast-path accounting can be asserted below.
        time.sleep(1.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["ideal_logical_messages"] == 3
    assert statistics["perfect_forwarding_calls"] == 3
    assert statistics["perfect_forwarding_receivers"] == 3
    assert statistics["attempted_packets"] == 3
    assert statistics["delivered_packets"] == 3
    assert statistics["queued_packets"] == 0
    assert statistics["dropped_per"] == 0
    assert statistics["ideal_statistical_bytes"] == statistics["attempted_bytes"]
    assert statistics["ideal_statistical_transport_blocks"] >= 3


@pytest.mark.timeout(30)
def test_active_link_hints_are_immediate_then_half_ttl_keepalives():
    configure_ros_domain(8, 120, 149)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=2",
            "-p", "max_retries:=0", "-p", "base_latency_ms:=0.0",
            "-p", "jitter_ms:=0.0", "-p", "active_link_hold_s:=1.0",
            "-p", "active_link_publish_period_s:=0.02",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_active_link_hint_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    hints = []
    node.create_subscription(
        LinkQualityArray,
        "/racer_sionna/active_links",
        lambda message: hints.append(
            [(link.sender_id, link.receiver_id) for link in message.links]
        ),
        qos,
    )
    publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1
        message = DroneState()
        message.drone_id = 1
        message.stamp = 1.0
        publisher.publish(message)
        deadline = time.monotonic() + 1.35
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        nonempty = [hint for hint in hints if hint]
        assert 2 <= len(nonempty) <= 4
        assert all(hint == [(0, 1)] for hint in nonempty)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.communicate(timeout=5)


@pytest.mark.timeout(30)
def test_shared_ofdma_multicast_is_one_physical_tx_with_independent_per():
    configure_ros_domain(9, 160, 189)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=10",
            "-p", "network_topology:=distributed",
            "-p", "fixed_mcs_index:=20", "-p", "tx_power_dbm:=20.0",
            "-p", "max_retries:=0", "-p", "base_latency_ms:=0.0",
            "-p", "jitter_ms:=0.0",
            "-p", "uav_ofdma_diagnostic_logging:=true",
            "-p", "uav_ofdma_log_packet_stride:=1",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_shared_ofdma_multicast_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = {receiver: [] for receiver in range(1, 10)}
    for receiver in received:
        node.create_subscription(
            DroneState,
            f"/racer_sionna/rx/drone_{receiver}/drone_state",
            lambda message, receiver=receiver: received[receiver].append(
                message.stamp
            ),
            qos,
        )
    publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    link_publisher = node.create_publisher(
        LinkQualityArray, "/racer_sionna/link_quality", qos
    )

    def publish_links():
        now = node.get_clock().now()
        message = LinkQualityArray()
        message.stamp = now.to_msg()
        valid_until = (now + Duration(seconds=2.0)).to_msg()
        for receiver in range(1, 10):
            link = LinkQuality()
            link.stamp = message.stamp
            link.valid_until = valid_until
            link.sender_id = 0
            link.receiver_id = receiver
            link.snr_db = 100.0 if receiver % 2 == 1 else -100.0
            link.model = "deterministic_test_path"
            message.links.append(link)
        link_publisher.publish(message)

    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while (
            publisher.get_subscription_count() < 1
            or link_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            publish_links()
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1
        assert link_publisher.get_subscription_count() == 1
        for _ in range(5):
            publish_links()
            rclpy.spin_once(node, timeout_sec=0.05)

        state = DroneState()
        state.drone_id = 1
        state.stamp = 314.0
        publisher.publish(state)
        deadline = time.monotonic() + 5.0
        while (
            any(not received[receiver] for receiver in (1, 3, 5, 7, 9))
            and time.monotonic() < deadline
        ):
            publish_links()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert received == {
            1: [314.0], 2: [], 3: [314.0], 4: [], 5: [314.0],
            6: [], 7: [314.0], 8: [], 9: [314.0],
        }
        time.sleep(1.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)

    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["shared_uav_ofdma_enabled"] is True
    assert statistics["uav_transport"] == "UDP"
    assert statistics["uav_udp_retransmissions"] == 0
    assert statistics["uav_udp_datagrams_enqueued"] == 1
    assert statistics["uav_udp_multicast_datagrams"] == 1
    assert statistics["uav_udp_unicast_datagrams"] == 0
    assert statistics["uav_physical_transmissions_started"] == 1
    assert statistics["uav_physical_transmissions_completed"] == 1
    assert statistics["uav_tx_power_applications"] == 1
    assert statistics["uav_ofdma_prb_allocation_events"] == 1
    assert statistics["uav_ofdma_allocated_prb_slots"] == 66
    assert statistics["uav_ofdma_max_allocated_prbs_per_slot"] == 66
    assert statistics["uav_udp_receiver_attempts"] == 9
    assert statistics["uav_udp_receiver_successes"] == 5
    assert statistics["uav_udp_receiver_per_failures"] == 4
    assert statistics["uav_udp_receiver_no_link_failures"] == 0
    assert statistics["uav_udp_receiver_half_duplex_failures"] == 0
    slot_logs = re.findall(r"RACER_UAV_OFDMA_SLOT[^\n]+", output)
    receiver_logs = re.findall(r"RACER_UAV_OFDMA_RX[^\n]+", output)
    assert len(slot_logs) == 1
    assert "allocated_prbs=66" in slot_logs[0]
    assert "receivers=9" in slot_logs[0]
    assert "tx_power_dbm=20.000" in slot_logs[0]
    assert len(receiver_logs) == 9
    assert any("receiver=2" in line and "result=per_failure" in line
               for line in receiver_logs)


@pytest.mark.timeout(30)
def test_shared_ofdma_enforces_half_duplex_for_overlapping_senders():
    configure_ros_domain(10, 190, 220)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=3",
            "-p", "network_topology:=distributed",
            "-p", "fixed_mcs_index:=20", "-p", "tx_power_dbm:=20.0",
            "-p", "max_retries:=0", "-p", "base_latency_ms:=0.0",
            "-p", "jitter_ms:=0.0",
            "-p", "uav_ofdma_diagnostic_logging:=false",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_shared_ofdma_half_duplex_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = {receiver: [] for receiver in (0, 1, 2)}
    for receiver in received:
        node.create_subscription(
            DroneState,
            f"/racer_sionna/rx/drone_{receiver}/drone_state",
            lambda message, receiver=receiver: received[receiver].append(
                message.stamp
            ),
            qos,
        )
    publishers = [
        node.create_publisher(
            DroneState, f"/racer_sionna/tx/drone_{sender}/drone_state", qos
        )
        for sender in (0, 1)
    ]
    link_publisher = node.create_publisher(
        LinkQualityArray, "/racer_sionna/link_quality", qos
    )

    def publish_links():
        now = node.get_clock().now()
        message = LinkQualityArray()
        message.stamp = now.to_msg()
        valid_until = (now + Duration(seconds=2.0)).to_msg()
        for sender, receiver in ((0, 1), (0, 2), (1, 0), (1, 2)):
            link = LinkQuality()
            link.stamp = message.stamp
            link.valid_until = valid_until
            link.sender_id = sender
            link.receiver_id = receiver
            link.snr_db = 100.0
            link.model = "deterministic_test_path"
            message.links.append(link)
        link_publisher.publish(message)

    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while (
            any(publisher.get_subscription_count() < 1
                for publisher in publishers)
            or link_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            publish_links()
            rclpy.spin_once(node, timeout_sec=0.05)
        assert all(publisher.get_subscription_count() == 1
                   for publisher in publishers)
        for _ in range(5):
            publish_links()
            rclpy.spin_once(node, timeout_sec=0.05)

        for sender, stamp in ((0, 401.0), (1, 402.0)):
            state = DroneState()
            state.drone_id = sender + 1
            state.stamp = stamp
            # Keep both physical transmissions active across many slots so
            # each sending UAV necessarily overlaps the other's multicast.
            state.grid_ids = list(range(20000))
            publishers[sender].publish(state)
        deadline = time.monotonic() + 5.0
        while len(received[2]) < 2 and time.monotonic() < deadline:
            publish_links()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert sorted(received[2]) == [401.0, 402.0]
        assert received[0] == []
        assert received[1] == []
        time.sleep(1.2)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)

    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["uav_physical_transmissions_started"] == 2
    assert statistics["uav_physical_transmissions_completed"] == 2
    assert statistics["uav_tx_power_applications"] == 2
    assert statistics["uav_ofdma_peak_active_senders"] == 2
    assert statistics["uav_ofdma_max_allocated_prbs_per_slot"] == 66
    assert statistics["uav_udp_receiver_attempts"] == 4
    assert statistics["uav_udp_receiver_successes"] == 2
    assert statistics["uav_udp_receiver_half_duplex_failures"] == 2


@pytest.mark.timeout(30)
def test_nearest_neighbor_topology_delivers_only_to_closest_uav():
    configure_ros_domain(5, 90, 119)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=4",
            "-p", "network_topology:=nearest_neighbors",
            "-p", "nearest_neighbor_count:=1",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_nearest_neighbor_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = {receiver: [] for receiver in (1, 2, 3)}
    for receiver in received:
        node.create_subscription(
            DroneState,
            f"/racer_sionna/rx/drone_{receiver}/drone_state",
            lambda message, receiver=receiver: received[receiver].append(
                message.stamp
            ),
            qos,
        )
    state_publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    odometry_publishers = [
        node.create_publisher(Odometry, f"/drone_{drone}/odom", 10)
        for drone in range(4)
    ]
    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while state_publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert state_publisher.get_subscription_count() == 1
        for _ in range(10):
            for drone, x_position in enumerate((0.0, 1.0, 3.0, 10.0)):
                odometry = Odometry()
                odometry.pose.pose.position.x = x_position
                odometry_publishers[drone].publish(odometry)
            rclpy.spin_once(node, timeout_sec=0.05)
        message = DroneState()
        message.drone_id = 1
        message.stamp = 23.0
        state_publisher.publish(message)
        deadline = time.monotonic() + 5.0
        while not received[1] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(1.0)
        assert received == {1: [23.0], 2: [], 3: []}
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["nearest_neighbor_count"] == 1
    assert statistics["nearest_filtered_receivers"] >= 2
    assert statistics["dropped_per"] == 0


@pytest.mark.timeout(30)
def test_lossless_nearest_override_keeps_other_receivers_on_sionna_path():
    configure_ros_domain(7, 30, 59)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=4",
            "-p", "network_topology:=distributed",
            "-p", "lossless_nearest_neighbor_count:=1",
            "-p", "max_retries:=0",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_lossless_nearest_override_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = {receiver: [] for receiver in (1, 2, 3)}
    for receiver in received:
        node.create_subscription(
            DroneState,
            f"/racer_sionna/rx/drone_{receiver}/drone_state",
            lambda message, receiver=receiver: received[receiver].append(
                message.stamp
            ),
            qos,
        )
    state_publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    link_publisher = node.create_publisher(
        LinkQualityArray, "/racer_sionna/link_quality", qos
    )
    odometry_publishers = [
        node.create_publisher(Odometry, f"/drone_{drone}/odom", 10)
        for drone in range(4)
    ]

    def publish_sionna_link():
        now = node.get_clock().now()
        links = LinkQualityArray()
        links.stamp = now.to_msg()
        link = LinkQuality()
        link.stamp = links.stamp
        link.valid_until = (now + Duration(seconds=1.0)).to_msg()
        link.sender_id = 0
        link.receiver_id = 3
        link.snr_db = 100.0
        link.model = "sionna_exact"
        links.links.append(link)
        # Receiver 2 intentionally has no Sionna link. Receiver 1 is closest
        # and must be delivered losslessly without any link-quality sample.
        link_publisher.publish(links)

    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while (
            state_publisher.get_subscription_count() < 1
            or link_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            publish_sionna_link()
            rclpy.spin_once(node, timeout_sec=0.05)
        assert state_publisher.get_subscription_count() == 1
        assert link_publisher.get_subscription_count() == 1
        for _ in range(10):
            for drone, x_position in enumerate((0.0, 1.0, 3.0, 10.0)):
                odometry = Odometry()
                odometry.pose.pose.position.x = x_position
                odometry_publishers[drone].publish(odometry)
            publish_sionna_link()
            rclpy.spin_once(node, timeout_sec=0.05)
        message = DroneState()
        message.drone_id = 1
        message.stamp = 67.0
        state_publisher.publish(message)
        deadline = time.monotonic() + 5.0
        while (
            (not received[1] or not received[3])
            and time.monotonic() < deadline
        ):
            publish_sionna_link()
            rclpy.spin_once(node, timeout_sec=0.02)
        time.sleep(1.0)
        assert received == {1: [67.0], 2: [], 3: [67.0]}
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["lossless_nearest_neighbor_count"] == 1
    assert statistics["lossless_nearest_forwarded_packets"] == 1
    assert statistics["sionna_direct_attempted_packets"] == 2
    assert statistics["direct_attempted_packets"] == 3
    assert statistics["direct_delivered_packets"] == 2
    assert statistics["dropped_no_link"] == 1
    assert statistics["sionna_exact_samples"] > 0


@pytest.mark.timeout(35)
def test_selective_nearest_control_and_directed_chunk_budget():
    configure_ros_domain(8, 120, 149)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=4",
            "-p", "network_topology:=distributed",
            "-p", "lossless_nearest_neighbor_count:=2",
            "-p", "lossless_control_only:=true",
            "-p", "directed_message_unicast:=true",
            "-p", "chunk_data_pre_enqueue_dedup:=true",
            "-p", "chunk_data_max_pending_per_link:=2",
            "-p", "max_retries:=0",
            "-p", "base_latency_ms:=500.0", "-p", "jitter_ms:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_selective_nearest_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    states = {receiver: [] for receiver in (1, 2, 3)}
    chunks = {receiver: [] for receiver in (1, 2, 3)}
    for receiver in states:
        node.create_subscription(
            DroneState,
            f"/racer_sionna/rx/drone_{receiver}/drone_state",
            lambda message, receiver=receiver: states[receiver].append(message.stamp),
            qos,
        )
        node.create_subscription(
            ChunkData,
            f"/racer_sionna/rx/drone_{receiver}/chunk_data",
            lambda message, receiver=receiver: chunks[receiver].append(message.idx),
            qos,
        )
    state_publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    chunk_publisher = node.create_publisher(
        ChunkData, "/racer_sionna/tx/drone_0/chunk_data", qos
    )
    link_publisher = node.create_publisher(
        LinkQualityArray, "/racer_sionna/link_quality", qos
    )
    odometry_publishers = [
        node.create_publisher(Odometry, f"/drone_{drone}/odom", 10)
        for drone in range(4)
    ]

    def publish_link():
        now = node.get_clock().now()
        links = LinkQualityArray()
        links.stamp = now.to_msg()
        link = LinkQuality()
        link.stamp = links.stamp
        link.valid_until = (now + Duration(seconds=1.0)).to_msg()
        link.sender_id = 0
        link.receiver_id = 3
        link.snr_db = 100.0
        link.model = "sionna_exact"
        links.links.append(link)
        link_publisher.publish(links)

    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while (
            state_publisher.get_subscription_count() < 1
            or chunk_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert state_publisher.get_subscription_count() == 1
        assert chunk_publisher.get_subscription_count() == 1
        for _ in range(10):
            for drone, x_position in enumerate((0.0, 1.0, 2.0, 10.0)):
                odometry = Odometry()
                odometry.pose.pose.position.x = x_position
                odometry_publishers[drone].publish(odometry)
            rclpy.spin_once(node, timeout_sec=0.05)

        state = DroneState()
        state.drone_id = 1
        state.stamp = 101.0
        state_publisher.publish(state)
        deadline = time.monotonic() + 3.0
        while (
            (not states[1] or not states[2])
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.02)
        assert states == {1: [101.0], 2: [101.0], 3: []}

        publish_link()
        rclpy.spin_once(node, timeout_sec=0.1)
        for index in (1, 2, 3):
            chunk = ChunkData()
            chunk.from_drone_id = 1
            chunk.to_drone_id = 4
            chunk.chunk_drone_id = 1
            chunk.idx = index
            chunk.voxel_adrs = [index]
            chunk.voxel_occ = [1]
            chunk_publisher.publish(chunk)
        deadline = time.monotonic() + 4.0
        while len(chunks[3]) < 2 and time.monotonic() < deadline:
            publish_link()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert chunks == {1: [], 2: [], 3: [1, 2]}

        duplicate = ChunkData()
        duplicate.from_drone_id = 1
        duplicate.to_drone_id = 4
        duplicate.chunk_drone_id = 1
        duplicate.idx = 1
        duplicate.voxel_adrs = [1]
        duplicate.voxel_occ = [1]
        chunk_publisher.publish(duplicate)
        deadline = time.monotonic() + 1.2
        while time.monotonic() < deadline:
            publish_link()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert chunks == {1: [], 2: [], 3: [1, 2]}
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["lossless_control_only"] is True
    assert statistics["directed_message_unicast"] is True
    assert statistics["chunk_data_pre_enqueue_dedup"] is True
    assert statistics["lossless_nearest_forwarded_packets"] == 2
    assert statistics["directed_unicast_messages"] >= 4
    assert statistics["chunk_enqueue_budget_drops"] >= 1
    assert statistics["chunk_enqueue_duplicates_suppressed"] >= 1


@pytest.mark.timeout(30)
def test_distance_radius_topology_enforces_hard_3d_range():
    configure_ros_domain(6, 60, 89)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=4",
            "-p", "network_topology:=distance_radius",
            "-p", "communication_range_m:=4.0",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_distance_radius_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = {receiver: [] for receiver in (1, 2, 3)}
    for receiver in received:
        node.create_subscription(
            DroneState,
            f"/racer_sionna/rx/drone_{receiver}/drone_state",
            lambda message, receiver=receiver: received[receiver].append(
                message.stamp
            ),
            qos,
        )
    state_publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    odometry_publishers = [
        node.create_publisher(Odometry, f"/drone_{drone}/odom", 10)
        for drone in range(4)
    ]
    positions = (
        (0.0, 0.0, 0.0),
        (0.0, 0.0, 4.0),
        (2.4, 2.4, 2.4),
        (4.01, 0.0, 0.0),
    )
    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while state_publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert state_publisher.get_subscription_count() == 1
        for _ in range(10):
            for publisher, (x_position, y_position, z_position) in zip(
                odometry_publishers, positions
            ):
                odometry = Odometry()
                odometry.pose.pose.position.x = x_position
                odometry.pose.pose.position.y = y_position
                odometry.pose.pose.position.z = z_position
                publisher.publish(odometry)
            rclpy.spin_once(node, timeout_sec=0.05)
        message = DroneState()
        message.drone_id = 1
        message.stamp = 41.0
        state_publisher.publish(message)
        deadline = time.monotonic() + 5.0
        while not received[1] and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        time.sleep(1.0)
        assert received == {1: [41.0], 2: [], 3: []}
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["network_topology"] == "distance_radius"
    assert statistics["communication_range_m"] == 4.0
    assert statistics["range_filtered_receivers"] >= 2
    assert statistics["dropped_queue"] == 0


@pytest.mark.timeout(30)
def test_ap_assisted_proxy_gathers_and_deduplicates():
    configure_ros_domain(1, 180, 199)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=3",
            "-p", "network_topology:=ap_assisted",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_ap_proxy_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    received = {1: [], 2: []}
    node.create_subscription(
        DroneState,
        "/racer_sionna/rx/drone_1/drone_state",
        lambda message: received[1].append(message.stamp),
        qos,
    )
    node.create_subscription(
        DroneState,
        "/racer_sionna/rx/drone_2/drone_state",
        lambda message: received[2].append(message.stamp),
        qos,
    )
    publisher = node.create_publisher(
        DroneState, "/racer_sionna/tx/drone_0/drone_state", qos
    )
    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1
        message = DroneState()
        message.drone_id = 1
        message.stamp = 17.0
        publisher.publish(message)
        deadline = time.monotonic() + 8.0
        while any(len(values) < 1 for values in received.values()) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        # Leave enough time for any slower duplicate AP copy to arrive.
        duplicate_deadline = time.monotonic() + 1.2
        while time.monotonic() < duplicate_deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received == {1: [17.0], 2: [17.0]}
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["network_topology"] == "ap_assisted"
    assert statistics["ap_global_updates_received"] >= 1
    assert statistics["logical_delivered_packets"] == 2


@pytest.mark.timeout(30)
def test_bs_round_robin_uploads_only_incremental_map_chunks():
    configure_ros_domain(2, 160, 179)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=3",
            "-p", "network_topology:=bs_round_robin",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
            "-p", "bs_min_turn_ms:=5.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_bs_round_robin_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    publisher = node.create_publisher(
        ChunkData, "/racer_sionna/tx/drone_0/chunk_data", qos
    )
    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while publisher.get_subscription_count() < 1 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() == 1
        message = ChunkData()
        message.from_drone_id = 1
        message.to_drone_id = 2
        message.chunk_drone_id = 1
        message.idx = 1
        message.voxel_adrs = [17, 23]
        message.voxel_occ = [0, 1]
        publisher.publish(message)
        time.sleep(1.5)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["network_topology"] == "bs_round_robin"
    assert statistics["bs_round_robin_enabled"] is True
    assert statistics["bs_round_robin_turns"] >= 3
    assert statistics["bs_upload_grants_delivered"] >= 3
    assert statistics["bs_incremental_chunks_received_uplink"] == 1
    assert statistics["bs_known_map_chunks"] == 1
    assert statistics["ap_uplink_attempted_packets"] == 0


@pytest.mark.timeout(30)
def test_bs_round_robin_repairs_a_missing_direct_map_chunk():
    configure_ros_domain(3, 140, 159)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=2",
            "-p", "network_topology:=bs_round_robin",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
            "-p", "bs_min_turn_ms:=5.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_bs_repair_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    chunk_publisher = node.create_publisher(
        ChunkData, "/racer_sionna/tx/drone_0/chunk_data", qos
    )
    link_publisher = node.create_publisher(
        LinkQualityArray, "/racer_sionna/link_quality", qos
    )
    received = []
    node.create_subscription(
        ChunkData,
        "/racer_sionna/rx/drone_1/chunk_data",
        lambda message: received.append((message.chunk_drone_id, message.idx)),
        qos,
    )

    def publish_bs_links():
        now = node.get_clock().now()
        message = LinkQualityArray()
        message.stamp = now.to_msg()
        valid_until = (now + Duration(seconds=1.0)).to_msg()
        # Node 2 is the BS. Intentionally omit both direct UAV links so the
        # only possible delivery from UAV 0 to UAV 1 is the scheduled BS path.
        for sender, receiver in ((0, 2), (2, 0), (1, 2), (2, 1)):
            link = LinkQuality()
            link.stamp = message.stamp
            link.valid_until = valid_until
            link.sender_id = sender
            link.receiver_id = receiver
            link.snr_db = 40.0
            link.model = "deterministic_test_path"
            message.links.append(link)
        link_publisher.publish(message)

    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while (
            chunk_publisher.get_subscription_count() < 1
            or link_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            publish_bs_links()
            rclpy.spin_once(node, timeout_sec=0.05)
        assert chunk_publisher.get_subscription_count() == 1
        assert link_publisher.get_subscription_count() == 1
        for _ in range(5):
            publish_bs_links()
            rclpy.spin_once(node, timeout_sec=0.05)
        chunk = ChunkData()
        chunk.from_drone_id = 1
        chunk.to_drone_id = 2
        chunk.chunk_drone_id = 1
        chunk.idx = 7
        chunk.voxel_adrs = [3, 9]
        chunk.voxel_occ = [1, 0]
        chunk_publisher.publish(chunk)
        deadline = time.monotonic() + 5.0
        while not received and time.monotonic() < deadline:
            publish_bs_links()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert received == [(1, 7)]
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["direct_delivered_packets"] == 0
    assert statistics["bs_incremental_chunks_received_uplink"] == 1
    assert statistics["bs_missing_chunks_delivered_downlink"] == 1
    assert statistics["uav_ofdma_total_prbs"] == 33
    phy = statistics["phy"]
    assert phy["bandwidth_hz"] == 100_000_000.0
    assert phy["spectrum_partition_enabled"] is True
    assert phy["uav_broadcast_bandwidth_hz"] == 50_000_000.0
    assert phy["uav_broadcast_resource_blocks"] == 33
    assert phy["fixed_mcs_index"] == 14
    assert phy["uav_mcs_counts"]["16QAM"] > 0
    assert phy["bs_bandwidth_hz"] == 50_000_000.0
    assert phy["bs_resource_blocks"] == 33
    assert phy["bs_mcs_mode"] == "adaptive"
    assert phy["bs_fixed_mcs_index"] == -1
    assert phy["bs_mcs_counts"]["256QAM"] > 0


@pytest.mark.timeout(30)
def test_rl_bs_file_bridge_controls_upload_and_owner_specific_relay(tmp_path):
    configure_ros_domain(11, 90, 119)
    action_path = tmp_path / "action.txt"
    state_path = tmp_path / "communication_state.json"
    ground_truth_path = tmp_path / "ground_truth_occupied_voxels.txt"
    ground_truth_path.write_text("7\n19\n")
    observed_path = tmp_path / "observed_occupied_voxels.txt"
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=sionna", "-p", "drone_count:=2",
            "-p", "network_topology:=bs_round_robin",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
            "-p", "max_retries:=0", "-p", "bs_max_retries:=0",
            "-p", "rl_bs_scheduler_enabled:=true",
            "-p", "rl_bs_decision_period_ms:=10.0",
            "-p", f"rl_bs_action_path:={action_path}",
            "-p", f"rl_bs_state_path:={state_path}",
            "-p", f"ground_truth_occupied_voxels_path:={ground_truth_path}",
            "-p", f"observed_occupied_voxels_path:={observed_path}",
            "-p", "require_ground_truth_map:=true",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_rl_bs_file_bridge_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    chunk_publisher = node.create_publisher(
        ChunkData, "/racer_sionna/tx/drone_0/chunk_data", qos
    )
    link_publisher = node.create_publisher(
        LinkQualityArray, "/racer_sionna/link_quality", qos
    )
    received = []
    node.create_subscription(
        ChunkData,
        "/racer_sionna/rx/drone_1/chunk_data",
        lambda message: received.append((message.chunk_drone_id, message.idx)),
        qos,
    )

    def publish_bs_links():
        now = node.get_clock().now()
        message = LinkQualityArray()
        message.stamp = now.to_msg()
        valid_until = (now + Duration(seconds=1.0)).to_msg()
        # Direct 0->1 is intentionally absent. Node 2 is the BS.
        for sender, receiver in ((0, 2), (2, 0), (1, 2), (2, 1)):
            link = LinkQuality()
            link.stamp = message.stamp
            link.valid_until = valid_until
            link.sender_id = sender
            link.receiver_id = receiver
            link.snr_db = 40.0
            link.model = "deterministic_test_path"
            message.links.append(link)
        link_publisher.publish(message)

    def wait_for(predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        payload = None
        while time.monotonic() < deadline:
            publish_bs_links()
            rclpy.spin_once(node, timeout_sec=0.02)
            try:
                payload = json.loads(state_path.read_text())
            except (OSError, json.JSONDecodeError):
                continue
            if predicate(payload):
                return payload
        raise AssertionError(f"timed out waiting for RL state; last={payload}")

    output = ""
    try:
        deadline = time.monotonic() + 10.0
        while (
            chunk_publisher.get_subscription_count() < 1
            or link_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            publish_bs_links()
            rclpy.spin_once(node, timeout_sec=0.05)
        assert chunk_publisher.get_subscription_count() == 1
        chunk = ChunkData()
        chunk.from_drone_id = 1
        chunk.to_drone_id = 2
        chunk.chunk_drone_id = 1
        chunk.idx = 41
        chunk.voxel_adrs = [7, 13]
        chunk.voxel_occ = [1, 0]
        chunk_publisher.publish(chunk)
        # Wait until the proxy has observed the source chunk before publishing
        # the one-shot action epoch. Sleeping here is racy under the full test
        # suite because DDS delivery and the 10 ms scheduler run independently.
        wait_for(
            lambda value: value.get("map_summary", {}).get(
                "local_known_chunks"
            ) == [1, 0]
        )

        # N=2 canonical bits: B01, B10, u0, u1. Upload owner 0 once.
        action_path.write_text("1 2 0 0 1 0\n")
        upload_state = wait_for(
            lambda value: value.get("action_epoch") == 1
            and value.get("map_summary", {}).get("bs_known_chunks") == 1
        )
        assert upload_state["bs_uplink_prb_slots"] > 0
        assert upload_state["bs_downlink_prb_slots"] == 0
        assert upload_state["redundant_exploration_ratio"] == 0.0
        assert upload_state["bs_global_map_iou"] == 0.5

        # Relay only owner-0 information to UAV 1; do not upload again.
        action_path.write_text("2 2 1 0 0 0\n")
        relay_state = wait_for(
            lambda value: value.get("action_epoch") == 2
            and value.get("bs_downlink_prb_slots", 0) > 0
        )
        deadline = time.monotonic() + 3.0
        while not received and time.monotonic() < deadline:
            publish_bs_links()
            rclpy.spin_once(node, timeout_sec=0.02)
        assert received == [(1, 41)]
        assert relay_state["bs_uplink_prb_slots"] == upload_state["bs_uplink_prb_slots"]
        time.sleep(1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            output, _ = process.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            output, _ = process.communicate(timeout=5)
    matches = re.findall(r"RACER_SIONNA_STATS (\{[^\n]+\})", output)
    assert matches
    statistics = json.loads(matches[-1])
    assert statistics["rl_bs_scheduler_enabled"] is True
    assert statistics["direct_delivered_packets"] == 0
    assert statistics["bs_incremental_chunks_received_uplink"] == 1
    assert statistics["bs_missing_chunks_delivered_downlink"] == 1
    assert statistics["bs_uplink_prb_slots"] > 0
    assert statistics["bs_downlink_prb_slots"] > 0
    assert statistics["redundant_exploration_ratio"] == 0.0
    assert statistics["bs_global_map_iou"] == 0.5
    assert statistics["ground_truth_occupied_voxels"] == 2
    assert statistics["task_quality_history"]
    assert observed_path.read_text().splitlines() == ["7"]


@pytest.mark.timeout(30)
def test_bs_round_robin_routes_recovery_control_through_ap_links():
    configure_ros_domain(4, 120, 139)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=2",
            "-p", "network_topology:=bs_round_robin",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
            "-p", "bs_min_turn_ms:=5.0",
        ],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=os.environ.copy(),
        start_new_session=True,
    )
    rclpy.init()
    node = Node("racer_sionna_recovery_control_test")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE)
    status_received = []
    command_received = []
    node.create_subscription(
        RecoveryStatus,
        "/racer_ap_recovery/status_uplink",
        lambda message: status_received.append(
            (message.drone_id, message.episode_id)
        ),
        qos,
    )
    node.create_subscription(
        RecoveryCommand,
        "/racer_sionna/rx/drone_0/recovery_command",
        lambda message: command_received.append(
            (message.drone_id, message.partner_id)
        ),
        qos,
    )
    status_publisher = node.create_publisher(
        RecoveryStatus,
        "/racer_sionna/tx/drone_0/recovery_status",
        qos,
    )
    command_publisher = node.create_publisher(
        RecoveryCommand,
        "/racer_ap_recovery/command_downlink",
        qos,
    )
    try:
        deadline = time.monotonic() + 10.0
        while (
            status_publisher.get_subscription_count() < 1
            or command_publisher.get_subscription_count() < 1
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert status_publisher.get_subscription_count() == 1
        assert command_publisher.get_subscription_count() == 1

        status = RecoveryStatus()
        status.drone_id = 1
        status.episode_id = 9
        status.phase = RecoveryStatus.PHASE_WAITING_FOR_AP
        status.request_repartition = True
        status_publisher.publish(status)

        command = RecoveryCommand()
        command.drone_id = 1
        command.episode_id = 9
        command.assignment_epoch = 3
        command.action = RecoveryCommand.ACTION_REALLOCATE
        command.partner_id = 2
        command_publisher.publish(command)

        deadline = time.monotonic() + 8.0
        while (
            not status_received or not command_received
        ) and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        assert status_received == [(1, 9)]
        assert command_received == [(1, 2)]
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
