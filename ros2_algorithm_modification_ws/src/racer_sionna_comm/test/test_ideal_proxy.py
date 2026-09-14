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
def test_ideal_proxy_coalesces_latest_state_and_excludes_self():
    configure_ros_domain(0, 201, 229)
    process = subprocess.Popen(
        [
            "ros2", "run", "racer_sionna_comm",
            "racer_sionna_communication_proxy", "--ros-args",
            "-p", "mode:=ideal", "-p", "drone_count:=2",
            "-p", "base_latency_ms:=0.0", "-p", "jitter_ms:=0.0",
            "-p", "ideal_coalesce_window_ms:=200.0",
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
        while not received and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received == [(3.0, [3, 13])]
        assert self_received == []
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)


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
