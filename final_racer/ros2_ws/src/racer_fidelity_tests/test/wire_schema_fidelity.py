#!/usr/bin/env python3
"""Compare every active ROS2 wire schema with its ROS1 source definition."""

from pathlib import Path
import re
import sys


root = Path(sys.argv[1])
ported = Path(sys.argv[2])
mapping = {
    "msg/Bspline.msg": "swarm_exploration/bspline/msg/Bspline.msg",
    "msg/ChunkData.msg": "swarm_exploration/plan_env/msg/ChunkData.msg",
    "msg/ChunkStamps.msg": "swarm_exploration/plan_env/msg/ChunkStamps.msg",
    "msg/DeletedGoals.msg": "swarm_exploration/exploration_manager/msg/DeletedGoals.msg",
    "msg/DroneState.msg": "swarm_exploration/exploration_manager/msg/DroneState.msg",
    "msg/GridIds.msg": "swarm_exploration/exploration_manager/msg/GridIds.msg",
    "msg/GridTour.msg": "swarm_exploration/exploration_manager/msg/GridTour.msg",
    "msg/HGrid.msg": "swarm_exploration/exploration_manager/msg/HGrid.msg",
    "msg/IdxList.msg": "swarm_exploration/plan_env/msg/IdxList.msg",
    "msg/PairOpt.msg": "swarm_exploration/exploration_manager/msg/PairOpt.msg",
    "msg/PairOptResponse.msg": "swarm_exploration/exploration_manager/msg/PairOptResponse.msg",
    "msg/PositionCommand.msg": "uav_simulator/Utils/quadrotor_msgs/msg/PositionCommand.msg",
    "msg/SentGoals.msg": "swarm_exploration/exploration_manager/msg/SentGoals.msg",
    "srv/SolveMTSP.srv": "swarm_exploration/utils/lkh_mtsp_solver/srv/SolveMTSP.srv",
    "srv/SolveTSP.srv": "swarm_exploration/utils/lkh_tsp_solver/srv/SolveTSP.srv",
}


def normalized(path: Path, baseline: bool):
    result = []
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if not line:
            continue
        line = re.sub(r"\s*=\s*", "=", line)
        line = re.sub(r"\s+", " ", line)
        if baseline:
            line = re.sub(r"^time ", "builtin_interfaces/Time ", line)
            line = re.sub(r"^Header ", "std_msgs/Header ", line)
            line = line.replace(" voxel_occ_", " voxel_occ")
        result.append(line)
    return result


for port_relative, source_relative in mapping.items():
    expected = normalized(root / source_relative, True)
    actual = normalized(ported / port_relative, False)
    extension = []
    if port_relative == "msg/DroneState.msg":
        # Pairwise-robust RACER had already widened grid IDs and recorded its
        # version/commitment fields. Local-component startup appends gossip
        # after that established schema; other modes leave the suffix empty.
        expected = [
            "int32 drone_id",
            "int32[] grid_ids",
            "uint64 assignment_epoch",
            "float64 commitment_until",
            "bool requesting_work",
            "float64 recent_attempt_time",
            "float64 stamp",
            "float32[] pos",
            "float32[] vel",
            "float32 yaw",
            "int32[] heard_neighbor_ids",
            "int32 local_component_coordinator_id",
            "int32[] local_component_member_ids",
            "float32[] local_component_member_positions",
            "uint32 local_component_assignment_epoch",
            "int32[] local_component_ack_ids",
        ]
    elif port_relative == "msg/GridIds.msg":
        expected = ["int32[] ids"]
    elif port_relative == "msg/PairOpt.msg":
        expected = [
            "int32 from_drone_id",
            "int32 to_drone_id",
            "uint8 PHASE_PROPOSE=0",
            "uint8 PHASE_COMMIT=1",
            "uint8 PHASE_ABORT=2",
            "uint8 phase",
            "uint64 transaction_id",
            "uint64 from_assignment_epoch",
            "uint64 to_assignment_epoch",
            "uint64 new_assignment_epoch",
            "float64 commitment_until",
            "float64 stamp",
            "int32[] ego_ids",
            "int32[] other_ids",
        ]
    elif port_relative == "msg/PairOptResponse.msg":
        expected = [
            "int32 from_drone_id",
            "int32 to_drone_id",
            "int32 STATUS_REJECTED=0",
            "int32 STATUS_PREPARED=1",
            "int32 STATUS_BUSY=2",
            "int32 STATUS_COMMITTED=3",
            "int32 status",
            "float64 stamp",
            "uint64 transaction_id",
            "uint64 responder_assignment_epoch",
        ]
    if actual != expected + extension:
        raise SystemExit(
            f"wire schema differs for {port_relative}:\n"
            f"ROS1-normalized={expected}\nallowed-extension={extension}\nROS2={actual}")
print(
    f"PASS: {len(mapping)} ROS2 schemas preserve their recorded definitions; "
    "DroneState includes pairwise-robust fields and the local-component suffix"
)
