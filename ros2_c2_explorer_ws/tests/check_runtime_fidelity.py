#!/usr/bin/env python3
"""Audit ROS2 interfaces and algorithm parameters against the ROS1 launch."""

from __future__ import annotations

import math
from pathlib import Path
import re
import sys
import xml.etree.ElementTree as ET


WORKSPACE = Path(__file__).resolve().parents[1]
BASELINE = WORKSPACE.parent / "C2-Explorer" / "src" / "swarm_exploration"
PORT_INTERFACES = WORKSPACE / "src" / "c2_explorer_msgs"
BASELINE_LAUNCH = (
    BASELINE / "exploration_manager" / "launch" / "single_drone_planner.xml"
)
PORT_CONFIG = (
    WORKSPACE / "src" / "c2_explorer_isaac" / "config" / "c2_warehouse.yaml"
)

# These values are intentionally tied to the Isaac scene/sensor boundary. They
# must not be mistaken for changes to C2 task allocation or planning weights.
ISAAC_BOUNDARY_PARAMETERS = {
    "sdf_map.ground_height",
    "sdf_map.max_ray_length",
    "sdf_map.virtual_ceil_height",
    "map_ros.depth_filter_maxdist",
    "map_ros.visualization_truncate_height",
}

# ROS1 launch entries absent from the shared YAML for a reviewed reason.
ALLOWED_MISSING_PARAMETERS = {
    "color_a",  # Per-UAV color is supplied by the ROS2 launch action.
    "exploration.problem_id",  # Set to 1/2 on the two ROS2 LKH server nodes.
    "frontier.cluster_size_z",  # No source code reads this legacy entry.
    "map_ros.virtual_ground_enable",  # Visualization-only publisher in ROS1.
    "map_ros.virtual_ground_z",
    "map_ros.virtual_ground_stride",
}

# roslaunch substitutions used by the original 2 m/s simulation scenarios.
EVALUATED_PARAMETERS = {
    "exploration.vm": 2.0,
    "exploration.am": 2.0,
    "exploration.vz": 2.0,
    "exploration.yd": 60.0 * 3.1415926 / 180.0,
    "exploration.ydd": 90.0 * 3.1415926 / 180.0,
    "frontier.candidate_dphi": 15.0 * 3.1415926 / 180.0,
    "perception_utils.lidar_pitch": 0.0,
    "perception_utils.lidar_top_angle": 50.0 * 3.1415926 / 180.0,
    "perception_utils.lidar_bottom_angle": 5.0 * 3.1415926 / 180.0,
    "perception_utils.lidar_left_angle": 60.0 * 3.1415926 / 180.0,
    "perception_utils.lidar_right_angle": 60.0 * 3.1415926 / 180.0,
    "manager.max_vel": 2.0,
    "manager.max_acc": 2.0,
    "manager.max_yawdot": 120.0 * 3.1415926 / 180.0,
    "search.max_vel": 2.0,
    "search.max_acc": 2.0,
    "optimization.max_vel": 2.0,
    "optimization.max_acc": 2.0,
    "bspline.limit_vel": 2.0,
    "bspline.limit_acc": 2.0,
}


def scalar(value: str):
    value = value.strip().strip('"').strip("'")
    if value.lower() == "true":
        return True
    if value.lower() == "false":
        return False
    try:
        return int(value)
    except ValueError:
        try:
            return float(value)
        except ValueError:
            return value


def yaml_parameters(path: Path) -> dict[str, object]:
    result: dict[str, object] = {}
    pattern = re.compile(r"^\s{4}([A-Za-z0-9_.]+):\s+(.+?)\s*$")
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].rstrip()
        match = pattern.match(line)
        if match:
            result[match.group(1)] = scalar(match.group(2))
    return result


def baseline_parameters(path: Path) -> dict[str, object]:
    result: dict[str, object] = {}
    root = ET.parse(path).getroot()
    for entry in root.iter("param"):
        name = entry.attrib.get("name", "").replace("/", ".")
        value = entry.attrib.get("value", "")
        if name and "$(" not in value:
            result[name] = scalar(value)
    result.update(EVALUATED_PARAMETERS)
    return result


def equal(first: object, second: object) -> bool:
    if isinstance(first, (int, float)) and not isinstance(first, bool):
        if not isinstance(second, (int, float)) or isinstance(second, bool):
            return False
        return math.isclose(float(first), float(second), rel_tol=1.0e-7, abs_tol=1.0e-9)
    return first == second


def normalized_interface(path: Path, port: bool) -> list[str]:
    lines: list[str] = []
    for raw_line in path.read_text().splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line:
            continue
        if not port and line.startswith("time "):
            line = "builtin_interfaces/Time " + line[len("time ") :]
        if not port and path.name == "ChunkData.msg":
            line = re.sub(r"\bvoxel_occ_\b", "voxel_occ", line)
        lines.append(line)
    return lines


def interface_files(root: Path) -> dict[str, Path]:
    simulator_boundary_interfaces = {
        "PositionCommand.msg",
        "LinkQuality.msg",
        "LinkQualityArray.msg",
        "CommStatistics.msg",
    }
    return {
        path.name: path
        for path in root.rglob("*")
        if path.suffix in {".msg", ".srv"}
        and path.name not in simulator_boundary_interfaces
    }


def main() -> int:
    errors: list[str] = []
    baseline_interfaces = interface_files(BASELINE)
    port_interfaces = interface_files(PORT_INTERFACES)
    if baseline_interfaces.keys() != port_interfaces.keys():
        errors.append(
            "interface set differs: "
            f"ROS1={sorted(baseline_interfaces)} ROS2={sorted(port_interfaces)}"
        )
    for name in sorted(baseline_interfaces.keys() & port_interfaces.keys()):
        expected = normalized_interface(baseline_interfaces[name], False)
        actual = normalized_interface(port_interfaces[name], True)
        if expected != actual:
            errors.append(f"interface field mismatch: {name}\n  {expected}\n  {actual}")

    baseline = baseline_parameters(BASELINE_LAUNCH)
    port = yaml_parameters(PORT_CONFIG)
    checked = 0
    for name, expected in sorted(baseline.items()):
        if name in ISAAC_BOUNDARY_PARAMETERS or name in ALLOWED_MISSING_PARAMETERS:
            continue
        if name not in port:
            errors.append(f"missing ROS1 algorithm parameter: {name}")
            continue
        checked += 1
        if not equal(expected, port[name]):
            errors.append(
                f"parameter mismatch: {name}: ROS1={expected!r}, ROS2={port[name]!r}"
            )

    if errors:
        print("C2 runtime fidelity check FAILED", file=sys.stderr)
        print("\n".join(errors), file=sys.stderr)
        return 1
    print(
        "C2 runtime fidelity check passed: "
        f"{len(port_interfaces)} interfaces field-equivalent, "
        f"{checked} ROS1 algorithm parameters equal, "
        f"{len(ISAAC_BOUNDARY_PARAMETERS)} Isaac boundary parameters audited"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
