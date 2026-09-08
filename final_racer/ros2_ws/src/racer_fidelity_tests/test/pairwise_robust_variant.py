#!/usr/bin/env python3
"""Static integration checks for the intentional Pairwise Robust RACER variant."""

from pathlib import Path
import sys


src = Path(sys.argv[1]).resolve()
msgs = src / "racer_fidelity_msgs" / "msg"
core = src / "racer_original_core" / "upstream"


def require(text, tokens, label):
    missing = [token for token in tokens if token not in text]
    if missing:
        raise SystemExit(f"{label}: missing {missing}")


# Grid ids must survive the full HGrid range on every maintained schema copy.
for path in (
    msgs / "DroneState.msg",
    msgs / "PairOpt.msg",
    msgs / "GridIds.msg",
    core / "exploration_manager/msg/DroneState.msg",
    core / "exploration_manager/msg/PairOpt.msg",
    core / "exploration_manager/msg/GridIds.msg",
):
    schema = path.read_text()
    if "int8[]" in schema or "int32[]" not in schema:
        raise SystemExit(f"32-bit grid-id schema is not active in {path}")

pair_schema = (msgs / "PairOpt.msg").read_text()
require(pair_schema, (
    "PHASE_PROPOSE", "PHASE_COMMIT", "PHASE_ABORT", "transaction_id",
    "from_assignment_epoch", "to_assignment_epoch", "new_assignment_epoch",
), "pair transaction schema")

fsm = (core / "exploration_manager/src/fast_exploration_fsm.cpp").read_text()
opt = fsm.split("void FastExplorationFSM::optTimerCallback", 1)[1].split(
    "void FastExplorationFSM::initialAssignmentTimerCallback", 1)[0]
if opt.index("state1.recent_attempt_time_ = tn") > opt.index("allocateGrids("):
    raise SystemExit("failed pair-opt is not throttled before the expensive allocation")
require(fsm, (
    "initialAssignmentTimerCallback", "Initial spatial partition",
    "applyGlobalAssignment(initial_assignment_msg_)",
    "assignment_commitment_duration_", "requesting_work_",
    "releaseAssignmentForWork", "work_steal_failure_threshold_",
    "PHASE_PROPOSE", "STATUS_PREPARED", "PHASE_COMMIT", "STATUS_COMMITTED",
    "committed via state confirmation", "proposed grid union is stale or incomplete",
    "[TASK MODE]", "[PAIR SEARCH]", "[GLOBAL RECOVERY REQUEST]",
    "[GLOBAL RECOVERY RESULT]", "[GLOBAL FINISH CHECK]",
    "base_assignment_epochs", "conflicting global recovery",
), "allocation coordination")

global_schema = (msgs / "GlobalGridAssignment.msg").read_text()
require(global_schema, (
    "PURPOSE_RECOVERY", "global_finish", "recovery_uav_ids",
    "affected_uav_ids", "base_assignment_epochs",
), "global recovery schema")

astar = (core / "path_searching/src/astar2.cpp").read_text()
component = astar.split("bool Astar::searchKnownFreeComponent", 1)[1].split(
    "double Astar::getEarlyTerminateCost", 1)[0]
require(component, (
    "isInBox", "getInflateOccupancy", "SDFMap::UNKNOWN", "goal_bins",
    "connectivity_max_nodes_", "local_expansion_goal",
), "known-free connectivity")

manager = (core / "exploration_manager/src/fast_exploration_manager.cpp").read_text()
planning = manager.split("int FastExplorationManager::planExploreMotion", 1)[1].split(
    "int FastExplorationManager::planTrajToView", 1)[0]
if planning.index("getReachableViewpointsInfo") > planning.index("findGridAndFrontierPath"):
    raise SystemExit("reachability filtering is not before task selection")
require(planning, (
    "reachable_frontier_ids_", "local_expansion_goal", "return FAIL",
), "planning-stage reachability")

config = (src / "racer_isaac_adapter/config/original_warehouse_simple.yaml").read_text()
require(config, (
    "fsm.pair_transaction_timeout", "fsm.assignment_commitment_duration",
    "fsm.initial_partition_enabled", "fsm.work_steal_failure_threshold",
    "fsm.global_recovery_pair_fail_threshold",
    "fsm.global_recovery_cooldown_cycles",
    "fsm.global_recovery_coalesce_cycles",
    "fsm.global_finish_confirmation_cycles",
    "reachability.connectivity_max_nodes",
), "variant configuration")
if "fsm.initial_partition_enabled: false" not in config:
    raise SystemExit("initial GlobalGridAssignment is still a default pairwise prerequisite")
if "bsEventGlobalMode() && last_applied_global_assignment_epoch_ == 0" in fsm:
    raise SystemExit("BS-assisted IDLE still waits for an initial global epoch")

print("PASS: pairwise-first allocation and guarded global recovery are wired into the source tree")
