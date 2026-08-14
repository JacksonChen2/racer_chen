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
from racer_fidelity_msgs.msg import ChunkData, DroneState
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
            time.sleep(0.03)

        deadline = time.monotonic() + 8.0
        while len(received) < 3 and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert received == [
            (1.0, [1, 11]),
            (2.0, [2, 12]),
            (3.0, [3, 13]),
        ]
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
