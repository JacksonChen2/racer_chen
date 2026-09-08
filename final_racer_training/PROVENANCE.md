# Provenance and verification

## Extracted source

This directory was extracted on 2026-09-03 from the most recent Qwen8B-FP8
training campaign from the evening of 2026-09-02:

`../hybrid_communication_racer/experiments/five_sites_same_aisle_10uav_300s_20260831/qwen8b_fp8_resource_ltask025_perfect_direct_30ep_20260902`

That campaign completed five 300 s episodes and was stopped during episode 6.
Its resolved `training_state.json`, rather than an older nominal design, is the
source of the preserved model architecture and hyperparameters.

Eighteen supporting Python modules in `agentic_crpo/agentic_crpo` are
byte-for-byte identical to the corresponding latest modules. `train_crpo.py`
only removes its old Qwen14 fallback/default-resume exception and gives the
extracted Qwen8B-FP8 variant its own default name. The ordered aggregate
SHA-256 of the source modules before that specialization is:

`9e4404fd0bdd3c7ef90a144d2d9cf916e358a593647a675c710327e8da298c80`

The resulting specialized module aggregate SHA-256 is:

`faa15567b4b19c8c29245b66d9fd046e6a4f843befc6de1eb6f00c4459d69515`

At extraction time, the Isaac adapter source was byte-for-byte identical to the
latest adapter and had ordered aggregate SHA-256:

`954384db33520ffeff08a29d582f9716aab1d51dcd88f56c01492c8c4ee860fd`

The YAML values are unchanged except for the algorithm label, private `/tmp`
bridge namespace, final-racer perfect-reference path, and local output path.
No Qwen14B configuration, obsolete algorithm variant, old experiment output,
or checkpoint was copied.

The four-process refactor subsequently added only the compact physical-state
publisher to the local Isaac overlay. It also copied the final-racer
communication package into this training overlay and added shared-state,
action-consumption, and ACK endpoints there. The source `../final_racer` tree
has not been modified by this refactor.

## Reused final-racer inputs

- Core planner and ROS interfaces: `../final_racer/ros2_ws`
- Communication implementation baseline: `../final_racer/sionna_distributed_overlay_ws`
- 75.2913% perfect reference: `../final_racer/reference_result/formal_300s/warehouse_full_distributed_result.json`
- Ground-truth occupied voxels: `../final_racer/data/gt_occupied_voxels.txt`
- Local model: `/home/jiazheng/ai_models/Qwen3-8B-FP8`

The launch scripts check ROS package prefixes at startup, preventing an
accidental fallback to the hybrid workspace.

## Verification

`scripts/build.sh` successfully built the dedicated Isaac adapter and
communication overlays.
`scripts/check.sh` verifies paths, shell syntax, absence of legacy runtime
dependencies, ROS package isolation, vLLM/FP8 settings, the perfect-reference
data, and all Python tests. The current suite passes 47/47 tests, including a
cross-interpreter mmap atomicity test and action/ACK ordering test. `colcon
test-result --all` reports 34/34 passing test records across five communication
test targets and the Isaac control-batch target.

Before the four-process refactor, an end-to-end 5 s startup test of the legacy
synchronous runtime was completed on 2026-09-03. It loaded
Qwen3-8B-FP8 through vLLM 0.27.1 with native FP8 and CUTLASS, constructed the
550-dimensional dual-encoder policy, collected exactly 50 transitions at
100 ms intervals from 250 communication slots, and performed one valid
50-transition partial-rollout update. The update record reported unchanged
simulation time and slot indices across PPO training. The trainer exited 0 and
saved `crpo_final.zip` plus `training_state.json`.

The short simulator wrapper returned its expected legacy acceptance status 1:
five seconds is too short to exercise every RACER FSM path, and its historical
`bs_round_robin` guard expects non-RL round-robin grants. The campaign's strict
training artifacts and bridge checks passed; normal 300 s launches additionally
retain the independent 0.73 coverage/isolation guard.

The new four-process runtime has been build-, ABI-, unit-, and ROS-regression
tested, but a full Qwen+Isaac four-process episode has deliberately not been
started as part of this code-only refactor verification. The short integration
command is documented in `README.md` for an explicit GPU-consuming run.
