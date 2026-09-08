#!/usr/bin/env python3
"""Static integration checks for local-component startup assignment."""

from pathlib import Path
import sys


root = Path(sys.argv[1]).resolve()
fsm = (
    root
    / "racer_original_core/upstream/exploration_manager/src/fast_exploration_fsm.cpp"
).read_text()
header = (
    root
    / "racer_original_core/upstream/exploration_manager/include/exploration_manager/fast_exploration_fsm.h"
).read_text()
state_schema = (root / "racer_fidelity_msgs/msg/DroneState.msg").read_text()
assignment_schema = (
    root / "racer_fidelity_msgs/msg/GlobalGridAssignment.msg"
).read_text()
config = (
    root / "racer_isaac_adapter/config/original_warehouse_simple.yaml"
).read_text()


def require(text, tokens, label):
    missing = [token for token in tokens if token not in text]
    if missing:
        raise SystemExit(f"{label}: missing {missing}")


require(
    state_schema,
    (
        "heard_neighbor_ids",
        "local_component_coordinator_id",
        "local_component_member_ids",
        "local_component_member_positions",
        "local_component_assignment_epoch",
        "local_component_ack_ids",
    ),
    "local-component state gossip schema",
)
require(assignment_schema, ("member_ids", "offsets", "grid_ids"), "assignment scope")
require(
    header,
    (
        "localComponentAssignmentTimerCallback",
        "updateLocalComponentMembership",
        "applyLocalComponentAssignment",
        "local_component_bootstrap_complete_",
    ),
    "FSM declarations",
)
require(
    fsm,
    (
        'exploration_assignment_mode_ == "local_component"',
        "RACER_LOCAL_COMPONENT_DISCOVERY",
        "RACER_LOCAL_COMPONENT_ASSIGN",
        "RACER_LOCAL_COMPONENT_APPLIED",
        "RACER_LOCAL_COMPONENT_COMMITTED",
        "local_component_peer_heard_ids_",
        "getId() != local_component_coordinator_id_",
        "local_component_assignment_msg_",
        "servicePairTransactions(tn)",
    ),
    "FSM local-component implementation",
)
require(
    config,
    (
        "fsm.local_component_neighbor_freshness",
        "fsm.local_component_stability_duration",
        "fsm.local_component_assignment_interval",
    ),
    "local-component configuration",
)

opt = fsm.split("void FastExplorationFSM::optTimerCallback", 1)[1].split(
    "void FastExplorationFSM::initialAssignmentTimerCallback", 1
)[0]
if opt.index("servicePairTransactions(tn)") > opt.index(
    "localComponentAssignmentTimerCallback(tn)"
):
    raise SystemExit("existing transactions are not serviced before startup gating")
if opt.index("local_component_bootstrap_complete_") > opt.index(
    "outgoing_transaction_.active"
):
    raise SystemExit("pairwise allocation can race component ACK completion")

print("PASS: local-component discovery, scoped assignment, ACK gate, and pairwise handoff are wired")
