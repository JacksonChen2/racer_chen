#!/usr/bin/env python3
"""Verify that execution safety is effective and has no planner feedback path."""

import importlib.util
from pathlib import Path
import re
import sys

import numpy as np


isaac_dir = Path(sys.argv[1]).resolve()
spec = importlib.util.spec_from_file_location(
    "racer_safety", isaac_dir / "safety_cpp_bridge.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)

preferred = np.asarray((0.8, 0.0, 0.0), dtype=float)
position = np.asarray((0.0, 0.0, 0.0), dtype=float)
velocity = np.asarray((0.2, 0.0, 0.0), dtype=float)
peer_position = np.asarray((0.7, 0.0, 0.0), dtype=float)
snapshots = tuple(value.copy() for value in
                  (preferred, position, velocity, peer_position))
filtered = np.asarray(module.cbf_swarm_filter(
    preferred, position, [(2, peer_position, (0.0, 0.0, 0.0))],
    safe_distance=1.0, speed_limit=1.5, current_velocity=velocity))
if np.allclose(filtered, preferred):
    raise SystemExit("CBF did not constrain a command directed into a close peer")
for value, snapshot in zip((preferred, position, velocity, peer_position), snapshots):
    if not np.array_equal(value, snapshot):
        raise SystemExit("safety filter mutated a planning/input array")

# Regression for the 300 s Warehouse run: in a gap narrower than two
# conservative clearances, the farther opposing surface used to be processed
# last and projected the command back toward the nearest surface.  The nearest
# surface at +Y must now retain final authority and command a clear retreat.
narrow_gap = np.asarray(module.pointcloud_obstacle_filter(
    (0.0, 0.8, 0.0),
    (0.0, 0.0, 0.0),
    ((0.0, 0.285, 0.0), (0.0, -0.60, 0.0)),
    clearance=0.52,
    speed_limit=0.85,
    current_velocity=(0.0, 0.0, 0.0),
))
if narrow_gap[1] >= -0.15:
    raise SystemExit(
        "farther surface overrode the nearest narrow-gap safety constraint"
    )

# A PhysX sphere sweep reports *free travel of the already inflated vehicle
# envelope*.  Regression for v11: converting only the hit position to a point
# constraint allowed a thin rack frame to be crossed.  Direct free-travel
# enforcement must reverse an inward command before the 6 cm contact reserve.
sweep_brake = np.asarray(module.sweep_obstacle_filter(
    (0.0, 1.0, 0.0),
    [((0.0, 1.0, 0.0), 0.03)],
    speed_limit=2.0,
    current_velocity=(0.0, 0.14, 0.0),
    clearance=0.06,
))
if sweep_brake[1] >= 0.0:
    raise SystemExit("swept-envelope reserve did not command a retreat")

bridge = (isaac_dir / "original_racer_isaac.py").read_text()
publish = re.search(
    r"self\.cloud_publishers\[drone_id\]\.publish\(\s*"
    r"create_xyzi_cloud\(stamp, \"world\", ([^)]+)\)\s*\)",
    bridge, re.MULTILINE)
if not publish or publish.group(1).replace(" ", "") != "mapping_points,mapping_hit":
    raise SystemExit("planner cloud is not exclusively the mapping-camera output")
if "self.safety_points[drone_id] = remembered" not in bridge:
    raise SystemExit("independent safety point storage is missing")
if "self.scene_query_points" not in bridge:
    raise SystemExit("thin-obstacle PhysX sweep safety storage is missing")
for token in (
    "self.scene_query_constraints",
    "min(direction_hits)",
    "sweep_obstacle_filter(",
    "SCENE_QUERY_CLEARANCE = 0.06",
):
    if token not in bridge:
        raise SystemExit(f"direct swept-envelope safety is missing: {token}")
if "VEHICLE_RADIUS = (\n    SELF_FILTER_RADIUS" not in bridge:
    raise SystemExit("PhysX safety sweep does not cover the full rotor envelope")
if "OBSTACLE_CONTROL_CLEARANCE = max(0.30, VEHICLE_RADIUS)" not in bridge:
    raise SystemExit("static safety clearance double-counts the dynamic stopping margin")
if 'default=76800' not in bridge:
    raise SystemExit("normal sensor path does not preserve the full ROS1 depth grid")
if 'SOURCE_MAX_SPEED = 2.0 if ARGS.vehicle_model == "racer_so3"' not in bridge:
    raise SystemExit("Isaac execution lacks bounded authority to recover tracking lag")
if "ReliabilityPolicy.BEST_EFFORT" not in bridge or "depth=5" not in bridge:
    raise SystemExit("dense sensor transport can backpressure the physics loop")
if re.search(r"create_xyzi_cloud\([^)]*scene_query_points", bridge, re.DOTALL):
    raise SystemExit("PhysX sweep safety points leaked into the planner cloud")

# Isaac's non-ideal body may be deflected from the ideal command by the
# execution safety layer.  Its measured odometry still proves that the volume
# it swept was free.  The ROS2 sensor boundary must preserve that free corridor
# for the unchanged non-optimistic return A*, without injecting safety-lidar or
# scene-query obstacle points into SDFMap.
map_port = (
    isaac_dir.parent.parent
    / "racer_original_core"
    / "port"
    / "map_ros_port.cpp"
).read_text()
map_config = (isaac_dir.parent / "config" / "original_warehouse_simple.yaml").read_text()
for token in (
    "recordTraversedPosition(camera_pos_)",
    "void MapROS::prepareReturnCorridor()",
    "if (return_corridor_enabled_) return;",
    "return_corridor_enabled_ = true",
    "map_->md_->occupancy_buffer_[address] = free_log_odds",
    "finishCorridorUpdate(update_min, update_max, true)",
):
    if token not in map_port:
        raise SystemExit(f"measured swept-free-space transport is missing: {token}")
if "map_ros.traversed_clearance_radius: 0.30" not in map_config:
    raise SystemExit("Warehouse swept-free-space radius is missing")
cloud_callback = map_port.split("void MapROS::cloudCallback", 1)[1].split(
    "void MapROS::recordTraversedPosition", 1)[0]
if cloud_callback.index("map_->inputPointCloud") > cloud_callback.index(
        "recordTraversedPosition(camera_pos_)"):
    raise SystemExit("measured trajectory must be recorded after original cloud fusion")
if "updateMapChunk" in map_port:
    raise SystemExit("return-only local free corridor leaked into swarm map chunks")
corridor_radius = float(re.search(
    r"map_ros\.traversed_clearance_radius:\s*([0-9.]+)", map_config
).group(1))
if not 0.0 < corridor_radius < 0.332:
    raise SystemExit(
        "measured free corridor must be smaller than the collision-free body envelope"
    )
# A sustained discrepancy between the original PositionCommand and physical
# odometry is an execution-boundary failure, not a new planning input. The
# adapter must brake and signal it on a per-agent topic; the minimal FSM hook
# may only preserve the selected goal and invoke the original collision-replan
# path from measured odometry.
adapter_source = (isaac_dir.parent / "src" / "position_command_adapter.cpp").read_text()
core_source = (
    isaac_dir.parent.parent
    / "racer_original_core"
    / "upstream"
    / "exploration_manager"
    / "src"
    / "fast_exploration_fsm.cpp"
).read_text()
return_hook = "if (fd_->go_back_) expl_manager_->sdf_map_->prepareReturnCorridor();"
if return_hook not in core_source:
    raise SystemExit("return corridor is not gated by the original go_back branch")
for token in (
    'prefix + "/tracking_lost"',
    'declare_parameter<double>("tracking_error_threshold", 1.2)',
    'declare_parameter<double>("tracking_low_speed_threshold", 0.25)',
    'declare_parameter<double>("tracking_severe_error_threshold", 3.0)',
    "tracking_lost_pub_->publish(std_msgs::msg::Empty())",
    "current_time < recovery_brake_until_",
    "recovery_armed_ = false",
    "recovery_armed_ = true",
    "tracking_failure_candidate",
):
    if token not in adapter_source:
        raise SystemExit(f"tracking-loss execution boundary is missing: {token}")
for token in (
    'nh.subscribe("/racer/tracking_lost"',
    "fd_->static_state_ = true;",
    "fd_->avoid_collision_ = true;",
    'transitState(PLAN_TRAJ, "trackingLostCallback")',
):
    if token not in core_source:
        raise SystemExit(f"FSM execution recovery is missing: {token}")
tracking_callback = core_source.split(
    "void FastExplorationFSM::trackingLostCallback", 1)[1].split(
        "void FastExplorationFSM::odometryCallback", 1)[0]
for forbidden in ("updateFrontierStruct", "planExploreMotion", "next_pos_ =", "next_yaw_ ="):
    if forbidden in tracking_callback:
        raise SystemExit(
            f"tracking recovery illegally changes a planner decision: {forbidden}"
        )

# The ROS1 benchmark planned a point position.  In Isaac, map bounds must be
# expressed as feasible vehicle-centre coordinates so the unchanged planner
# cannot select a viewpoint that the independent rotor-envelope barrier must
# permanently reject.  The bounds are deliberately rounded inward to the
# 0.1 m SDF voxel grid.
scenario_path = isaac_dir / "scenario_cpp_bridge.py"
scenario_spec = importlib.util.spec_from_file_location(
    "racer_scenario", scenario_path)
scenario_module = importlib.util.module_from_spec(scenario_spec)
sys.modules[scenario_spec.name] = scenario_module
scenario_spec.loader.exec_module(scenario_module)
scenario = scenario_module.warehouse_simple_scene()
config = (isaac_dir.parent / "config" / "original_warehouse_simple.yaml").read_text()

def parameter(name):
    match = re.search(rf"^\s*{re.escape(name)}:\s*([-+0-9.eE]+)\s*$", config,
                      re.MULTILINE)
    if not match:
        raise SystemExit(f"Warehouse planner parameter {name} is missing")
    return float(match.group(1))

box_min = np.asarray(tuple(
    parameter(f"sdf_map.box_min_{axis}") for axis in "xyz"))
box_max = np.asarray(tuple(
    parameter(f"sdf_map.box_max_{axis}") for axis in "xyz"))
safety_min = np.asarray(scenario.safety_min, dtype=float)
safety_max = np.asarray(scenario.safety_max, dtype=float)
rotor_envelope = 0.332
if np.any(box_min < safety_min + rotor_envelope - 1.0e-9):
    raise SystemExit("Warehouse planner minimum admits an unsafe vehicle centre")
if np.any(box_max > safety_max - rotor_envelope + 1.0e-9):
    raise SystemExit("Warehouse planner maximum admits an unsafe vehicle centre")
for start in scenario.starts:
    if np.any(np.asarray(start) <= box_min) or np.any(np.asarray(start) >= box_max):
        raise SystemExit("Warehouse start lies outside the calibrated centre box")
print(
    "PASS: execution safety is isolated and planner bounds admit safe centres"
)
