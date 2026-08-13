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
    if expected != actual:
        raise SystemExit(
            f"wire schema differs for {port_relative}:\n"
            f"ROS1-normalized={expected}\nROS2={actual}")
print(f"PASS: {len(mapping)} ROS2 message/service schemas match their ROS1 definitions")
