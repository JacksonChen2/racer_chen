#!/usr/bin/env python3
"""Reject any unrecorded difference from original_racer_ros1_ws."""

import hashlib
import pathlib
import re
import sys


baseline_root = pathlib.Path(sys.argv[1])
port_root = pathlib.Path(sys.argv[2])


def files(root: pathlib.Path):
    return {p.relative_to(root).as_posix(): p for p in root.rglob("*") if p.is_file()}


baseline_files = files(baseline_root)
port_files = files(port_root)
if baseline_files.keys() != port_files.keys():
    missing = sorted(baseline_files.keys() - port_files.keys())
    extra = sorted(port_files.keys() - baseline_files.keys())
    raise SystemExit(f"source tree mismatch; missing={missing}, extra={extra}")

allowed = {
    "exploration_manager/include/exploration_manager/fast_exploration_fsm.h",
    "exploration_manager/src/fast_exploration_fsm.cpp",
    "plan_manage/src/planner_manager.cpp",
    "plan_env/include/plan_env/sdf_map.h",
    "plan_env/src/multi_map_manager.cpp",
}
different = set()
for relative, baseline_path in baseline_files.items():
    baseline_bytes = baseline_path.read_bytes()
    port_bytes = port_files[relative].read_bytes()
    if hashlib.sha256(baseline_bytes).digest() != hashlib.sha256(port_bytes).digest():
        different.add(relative)
        if relative not in allowed:
            raise SystemExit(f"unrecorded upstream algorithm modification: {relative}")

if different != allowed:
    raise SystemExit(f"expected transport-boundary changes {allowed}, observed {different}")

# Normalize only the recorded ROS2 API/control-boundary changes, then demand
# exact source equality. This prevents an algorithm change from hiding in an
# otherwise allowed boundary file.
fsm_header = port_files[
    "exploration_manager/include/exploration_manager/fast_exploration_fsm.h"
].read_text()
fsm_header = fsm_header.replace(
    "  void safetyCallback(const ros::TimerEvent& e);\n"
    "  void trackingLostCallback(const std_msgs::EmptyConstPtr& msg);",
    "  void safetyCallback(const ros::TimerEvent& e);")
fsm_header = fsm_header.replace(
    "  ros::Subscriber trigger_sub_, odom_sub_, tracking_lost_sub_;",
    "  ros::Subscriber trigger_sub_, odom_sub_;")
if fsm_header.rstrip("\n").encode() != baseline_files[
        "exploration_manager/include/exploration_manager/fast_exploration_fsm.h"
].read_bytes().rstrip(b"\n"):
    raise SystemExit("FSM header differs beyond the recorded execution-boundary hook")

fsm = port_files["exploration_manager/src/fast_exploration_fsm.cpp"].read_text()
tracking_subscription = '''  // ROS2/Isaac execution boundary only: the physical vehicle can be stopped by
  // the independent safety layer while the original ideal tracker advances.
  // Recover the original collision-replan semantics from measured odometry;
  // do not select a new frontier, task, viewpoint, or goal here.
  tracking_lost_sub_ =
      nh.subscribe("/racer/tracking_lost", 1, &FastExplorationFSM::trackingLostCallback, this);
'''
tracking_callback = '''void FastExplorationFSM::trackingLostCallback(const std_msgs::EmptyConstPtr& msg) {
  (void)msg;
  if (state_ != EXPL_STATE::PLAN_TRAJ && state_ != EXPL_STATE::PUB_TRAJ &&
      state_ != EXPL_STATE::EXEC_TRAJ)
    return;

  // Keep the already selected next_pos_/next_yaw_. This only invalidates the
  // unexecuted trajectory and restores a physically valid initial condition.
  replan_pub_.publish(std_msgs::Empty());
  fd_->static_state_ = true;
  fd_->avoid_collision_ = true;
  transitState(PLAN_TRAJ, "trackingLostCallback");
  ROS_WARN("Execution tracking lost: replan current goal from measured odometry");
}

'''
return_corridor_hook = '''    // ROS2/Isaac sensor boundary only: just before the unchanged return
    // planner runs, expose the collision-monitored corridor actually
    // traversed by the physical body.  Keeping this disabled during normal
    // exploration prevents measured tube surfaces from becoming frontiers.
    if (fd_->go_back_) expl_manager_->sdf_map_->prepareReturnCorridor();
'''
if (fsm.count(tracking_subscription) != 1 or
        fsm.count(tracking_callback) != 1 or
        fsm.count(return_corridor_hook) != 1):
    raise SystemExit("recorded execution-boundary recovery hook is missing or duplicated")
fsm = fsm.replace(tracking_subscription, "").replace(tracking_callback, "")
fsm = fsm.replace(return_corridor_hook, "")
fsm = fsm.replace(
    "ros::Time(msg->start_time).toSec() <=\n        sdat.swarm_trajs_",
    "msg->start_time.toSec() <= sdat.swarm_trajs_")
fsm = fsm.replace("ros::Time(msg->start_time).toSec()", "msg->start_time.toSec()")
if fsm.encode() != baseline_files[
        "exploration_manager/src/fast_exploration_fsm.cpp"].read_bytes():
    raise SystemExit("FSM differs beyond the recorded ROS2 time conversion")

maps = port_files["plan_env/src/multi_map_manager.cpp"].read_text()
maps = re.sub(r"msg\.voxel_occ(?!_)", "msg.voxel_occ_", maps)
if maps.rstrip("\n").encode() != baseline_files[
        "plan_env/src/multi_map_manager.cpp"].read_bytes().rstrip(b"\n"):
    raise SystemExit("map manager differs beyond the recorded ROS2 field spelling")

sdf_header = port_files["plan_env/include/plan_env/sdf_map.h"].read_text()
sdf_header = sdf_header.replace("  void prepareReturnCorridor();\n", "")
sdf_header = sdf_header.replace(
    "bool use_swarm_tf_{ false };", "bool use_swarm_tf_;")
if sdf_header.rstrip("\n").encode() != baseline_files[
        "plan_env/include/plan_env/sdf_map.h"].read_bytes().rstrip(b"\n"):
    raise SystemExit("SDFMap differs beyond deterministic initialization of its identity transform flag")

planner = port_files["plan_manage/src/planner_manager.cpp"].read_text()
degenerate_guard = """  // Generate traj through waypoints-based method. Astar::backtrack returns two
  // points when the goal is in the start voxel, and shortenPath intentionally
  // collapses them when they are within 1 mm. The ROS1 code then passed a
  // zero-segment problem to PolynomialTraj and indexed a 0x0 matrix. Preserve
  // the selected position and the original polynomial/B-spline pipeline by
  // representing this degenerate hover-and-yaw command as two finite-duration
  // polynomial segments at the same position.
  const bool degenerate_hover = tour.size() == 1;
  const int pt_num = degenerate_hover ? 3 : tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = degenerate_hover ? tour.front() : tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  if (degenerate_hover) {
    // Reuse the endpoint acceleration time already used by the original
    // global trajectory generator, while retaining the original yaw bound.
    const double duration = max(time_lb, pp_.max_vel_ / (2 * pp_.max_acc_));
    times.setConstant(duration / double(pt_num - 1));
  } else {
    for (int i = 0; i < pt_num - 1; ++i)
      times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);
  }
"""
baseline_block = """  // Generate traj through waypoints-based method
  const int pt_num = tour.size();
  Eigen::MatrixXd pos(pt_num, 3);
  for (int i = 0; i < pt_num; ++i) pos.row(i) = tour[i];

  Eigen::Vector3d zero(0, 0, 0);
  Eigen::VectorXd times(pt_num - 1);
  for (int i = 0; i < pt_num - 1; ++i)
    times(i) = (pos.row(i + 1) - pos.row(i)).norm() / (pp_.max_vel_ * 0.5);
"""
if planner.count(degenerate_guard) != 1:
    raise SystemExit("recorded degenerate hover guard is missing or duplicated")
planner = planner.replace(degenerate_guard, baseline_block)
if planner.encode() != baseline_files[
        "plan_manage/src/planner_manager.cpp"].read_bytes():
    raise SystemExit("planner manager differs beyond the recorded zero-segment repair")

print(
    f"PASS: {len(baseline_files)} upstream files verified; "
    "2 ROS2 API edits, 2 defined-behavior repairs, 1 Isaac execution "
    "boundary hook, and 1 return-sensor boundary hook are exactly recorded")
