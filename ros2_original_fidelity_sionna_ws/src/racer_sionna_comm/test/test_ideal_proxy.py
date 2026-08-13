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
from racer_fidelity_msgs.msg import DroneState
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


@pytest.mark.timeout(30)
def test_ideal_proxy_preserves_order_and_excludes_self():
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(201, 229))
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
    os.environ["ROS_DOMAIN_ID"] = str(random.randint(180, 199))
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
