# Qwen-8B + coverage-CRPO + BS fan-out-4

This is a separate algorithm variant beside the original Qwen-14B design.
It reuses the same two-timescale state, 100-bit PPO action, RACER Direct-U2U
path, 50 MHz fixed-MCS UAV broadcast subband, and 50 MHz adaptive-MCS BS
subband.  Its configuration, bridge files, run directory, checkpoint identity,
reward, and constraint are independent.

## Algorithm definition

The frozen slow agent is the local checkpoint:

```text
/home/jiazheng/ai_models/Qwen3-8B-FP8
```

Every `H=50` fine slots it outputs four ranked receiver UAVs for every source
UAV and one integer BS upload priority (0--4) per UAV.  The parser expands these
compact outputs into the internal `W_task` and `omega_sem` tensors.  Its prompt
also states that each information owner has only four BS relay destinations.
Qwen remains asynchronous, frozen, offline-only, and is never optimized by
CRPO.

The fine policy observation and action dimensions for ten UAVs are 550 and
100.  Let `C_joint(t)` be the latest measured joint mapping coverage.  The new
reward is:

```text
r_t = C_joint(t)
```

Coverage is held at the last real mapper value between mapper updates.  Over a
fixed 300-second horizon, the undiscounted return is proportional to coverage
AUC, rewarding both fast exploration and high final coverage.  The code also
supports `reward.type: coverage_delta` for a purely telescoping final-coverage
objective, but this variant deliberately selects literal `coverage`.

For proposed relay matrix `B_raw`, define source-UAV fan-out:

```text
f_i(t) = sum_{j != i} B_raw[i,j]
v_t = max_i [f_i(t)-4]+ / (N-1-4)       (N > 5)
```

`v_t` is in `[0,1]`; for ten UAVs, selecting all nine receivers gives cost 1.
CRPO uses `cost_t=v_t`, a rolling mean-step-cost estimate, `Gamma=0`, and
`eta=0`.  Any attempted overflow therefore switches the actor toward cost
rectification.

The physical scheduler does not wait for the policy to learn the rule.  Before
writing an action, every row above four is projected to its four highest
priority selected receivers.  Priority is configured as:

```text
0.70 W_task + 0.15 normalized relay queue
            + 0.10 normalized AoI + 0.05 normalized BS downlink SNR
```

Uploads in the last action row are not part of fan-out and remain unchanged.
The file bridge performs a second validation and refuses any matrix above the
limit.  Thus the proposed action supplies the CRPO violation signal while the
executed action always satisfies `max_i f_i <= 4`.

## Isolation from the Qwen-14B variant

The new configuration is `config_qwen8b_coverage_fanout4.yaml`.  It uses:

```text
bridge: /tmp/racer_agentic_crpo_qwen8b_fanout4/
runs:   agentic_crpo/runs/qwen8b_coverage_fanout4/
variant id: qwen8b_coverage_fanout4
```

The original `config.yaml`, Qwen-14B checkpoint, Qwen-14B run directories, and
stopped 30-episode campaign are not modified.  Training rejects a resume
checkpoint whose saved variant or constraint estimator does not match, so an
old 14B/task-loss checkpoint cannot silently seed this policy.

This reward and constraint do not require a perfect-communication trajectory.
In particular, the regenerated 36.73% reference is not used.  Raw coverage
comes directly from the mission snapshot.  Perfect-reference task-loss fields
may still be recorded as diagnostics when a valid reference is explicitly
provided, but they do not enter this variant's reward or CRPO cost.

## Checks and launch

Code-path smoke test, which disables Qwen and uses the deterministic mock:

```bash
source /home/jiazheng/ai_envs/racer-crpo/bin/activate
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/hybrid_communication_racer
PYTHONPATH=./agentic_crpo python -m agentic_crpo.train_crpo \
  --config agentic_crpo/config_qwen8b_coverage_fanout4.yaml \
  --mock-smoke \
  --output-dir /tmp/qwen8b_coverage_fanout4_smoke
```

For a real run, the simulator must use the matching independent bridge paths:

```bash
export RACER_COMMUNICATION_MODE=sionna
export RACER_NETWORK_TOPOLOGY=bs_round_robin
export RACER_REQUIRE_SIONNA=true
export RACER_RL_BS_SCHEDULER_ENABLED=true
export RACER_RL_BS_ACTION_PATH=/tmp/racer_agentic_crpo_qwen8b_fanout4/action.txt
export RACER_RL_BS_STATE_PATH=/tmp/racer_agentic_crpo_qwen8b_fanout4/communication_state.json
export RACER_RL_BS_MISSION_STATE_PATH=/tmp/racer_agentic_crpo_qwen8b_fanout4/mission_state.json
export RACER_RL_BS_DECISION_PERIOD_MS=20.0
./run_warehouse_simple_sionna.sh
```

Then train against that live bridge:

```bash
source /home/jiazheng/ai_envs/racer-crpo/bin/activate
python -m agentic_crpo.train_crpo \
  --config agentic_crpo/config_qwen8b_coverage_fanout4.yaml
```

TensorBoard reports both proposed and executed maximum fan-out, violating
sender count, projection drops, normalized step cost, coverage reward, and the
usual communication/task diagnostics.
