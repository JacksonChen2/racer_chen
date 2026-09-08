"""Map bypass must work with unavailable radio links; state must use radio."""
import json
import os
import re
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from racer_fidelity_msgs.msg import ChunkData, ChunkStamps, DroneState


def test_map_perfect_keeps_state_on_sionna():
    os.environ["ROS_DOMAIN_ID"] = "198"
    process = subprocess.Popen([
        "ros2", "run", "racer_sionna_comm", "racer_sionna_communication_proxy",
        "--ros-args", "-p", "mode:=sionna", "-p", "drone_count:=2",
        "-p", "map_perfect_delivery:=true", "-p", "max_retries:=0",
    ], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True,
       start_new_session=True)
    rclpy.init()
    node = Node("map_perfect_boundary_test")
    received = {key: [] for key in ("chunk_data", "chunk_stamps", "drone_state")}
    publishers = {}
    for key, kind in (("chunk_data", ChunkData), ("chunk_stamps", ChunkStamps),
                      ("drone_state", DroneState)):
        node.create_subscription(kind, f"/racer_sionna/rx/drone_1/{key}",
                                 lambda msg, key=key: received[key].append(msg), 100)
        publishers[key] = node.create_publisher(kind, f"/racer_sionna/tx/drone_0/{key}", 100)
    try:
        deadline = time.monotonic() + 12
        while not all(p.get_subscription_count() for p in publishers.values()):
            assert time.monotonic() < deadline
            rclpy.spin_once(node, timeout_sec=0.05)
        chunk = ChunkData()
        chunk.from_drone_id = 1
        chunk.to_drone_id = 2
        chunk.chunk_drone_id = 1
        chunk.idx = 1
        chunk.voxel_adrs = [123]
        chunk.voxel_occ = [1]
        publishers["chunk_data"].publish(chunk)
        publishers["chunk_stamps"].publish(ChunkStamps())
        state = DroneState()
        state.drone_id = 1
        publishers["drone_state"].publish(state)
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
        assert len(received["chunk_data"]) == 1
        assert list(received["chunk_data"][0].voxel_adrs) == [123]
        assert len(received["chunk_stamps"]) == 1
        assert not received["drone_state"]
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        output, _ = process.communicate(timeout=10)
    snapshots = re.findall(r"RACER_SIONNA_STATS\s+(\{.*\})", output)
    assert snapshots, output
    stats = json.loads(snapshots[-1])
    assert stats["map_perfect_delivery_enabled"]
    assert stats["map_perfect_messages"] == 2
    assert stats["ideal_logical_messages"] == 2
    assert stats["uav_udp_datagrams_enqueued"] == 1
