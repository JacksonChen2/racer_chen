# Two-timescale Qwen-14B + CRPO

The retained Qwen-14B resource-minimization/task-loss design described below
remains the default.  A separate Qwen-8B variant using coverage reward and a
hard per-source BS relay fan-out of four is documented in
[`QWEN8B_COVERAGE_FANOUT4.md`](QWEN8B_COVERAGE_FANOUT4.md) and configured by
`config_qwen8b_coverage_fanout4.yaml`; it does not overwrite this design.

This directory adds an optional learning scheduler to the existing RACER,
Isaac Sim, and Sionna stack. The original Direct U2U scheduler and transport
remain active and independent. CRPO can select only UAV-to-BS map uploads and
owner-specific BS-to-UAV relays; it never allocates PRBs.

## Architecture

```text
RACER + Isaac mission state       Sionna/C++ communication state
            |                                  |
            +---------- compact snapshots -----+
                               |
          every H slots         |             every slot
    compact large state -> frozen Qwen         |
                         -> (W_task, omega)     |
                               | atomic reuse   |
                               +------> small normalized state
                                                |
                                      Bernoulli CRPO actor
                                                |
                         N^2 bits -> [B relay; u upload]
                                                |
                              optional C++ BS scheduler bridge
```

The local checkpoint is configured, rather than embedded in Python, at
`/home/jiazheng/ai_models/Qwen3-14B-FP8`. It is Qwen3-14B (40 layers, hidden
size 5120), occupies about 16 GiB in FP8 files, and is loaded only with
`local_files_only=True`. The loader forces Transformers offline mode, calls
`eval()`, disables all parameter gradients, and generates under
`torch.no_grad()`. BF16/FP16 are supported; optional bitsandbytes flags are off
by default.
Because this checkpoint is FP8 and native Transformers FP8 kernels are shipped
separately, `dequantize_fp8: true` reconstructs its weights locally in the
configured BF16/FP16 dtype. This avoids any kernel/model download; the tradeoff
is roughly 28 GiB of GPU memory instead of 16 GiB.

The runnable Qwen-8B configurations use `backend: vllm` and keep the local
checkpoint in FP8 instead of dequantizing it. On this RTX PRO 6000 Blackwell
(SM120), the verified vLLM 0.27.1 path sets `linear_backend: cutlass`, disables
DeepGEMM and the FlashInfer sampler, and uses native sampling. The model is
created once and kept resident; do not create one vLLM engine per guidance
request. `max_num_seqs: 1` matches the single background guidance worker and
avoids capturing CUDA graphs for unused concurrency levels. `max_model_len:
8192` leaves room for the full compact state plus the configured 1024 output
tokens.

The slow state contains only compressed map statistics, RACER FSM states,
positions, all directed U2U and bidirectional UAV/BS window-mean SNR values, the
full channel-outage matrix, summaries of planned trajectories and exploration
regions, pair/BS AoI, version gaps, and exploration progress. Qwen returns
exactly two plain-text sections:
`UAV links` lists each source UAV's four highest-priority receivers in ranked
order, and `BS priority` assigns every UAV an integer upload priority from 0 to
4. The ranked links are expanded into the internal `task_dependency` matrix and
the BS priorities into `semantic_importance`. For top-4, link ranks become
weights `1.0, 0.75, 0.5, 0.25`; BS levels are scaled from 0--4 and normalized
when configured. Counts, IDs, uniqueness and bounds are validated.
An asynchronous request keeps the previous valid guidance until the new result
has been atomically accepted; inference or parse failure does not stop the fine
scheduler.

To convert a saved Qwen response into the exact `(N+1) x N` guidance matrix and
row-major vector used by the small model:

```bash
python scripts/map_qwen_output_to_weight_matrix.py --n-uavs 10 \
  qwen_output.txt --output guidance_weights.json
```

## Actor tensors and action mapping

For `N` UAVs the flat observation has `5N^2 + 5N` float32 values in
`[0,1]`. At the configured `N=10`, its dimension is **550**.

| Slice | Width | Contents |
|---|---:|---|
| `positions` | `3N` | normalized xyz |
| `channels` | `N^2+N` | off-diagonal U2U, UAV-BS, BS-UAV SNR |
| `aoi` | `N^2` | off-diagonal pair AoI and UAV-BS AoI |
| `missing` | `N^2` | pair and BS missing data, log normalized |
| `queues` | `N^2` | upload and two-hop relay queues, log normalized |
| `guidance` | `N^2+N` | `W_task` including its zero diagonal, then `omega` |
No perfect trajectory or ground-truth map enters this vector. Those data are
read only by the offline loss evaluator.

The actor distribution is SB3's native Bernoulli distribution over
`MultiBinary(N*N)`. `ActionMapping` expands bits in this canonical order:

1. `B[i,j]` for owner `i=0..N-1`, receiver `j=0..N-1`, skipping `i==j`;
2. `u[i]` for `i=0..N-1`.

The result is an `(N+1) x N` matrix: the first `N` rows are owner-specific BS
relays and the last row is upload selection. The diagonal is created as zero,
not represented by meaningless policy bits.

Before that Bernoulli distribution is constructed, the policy deterministically
recovers per-UAV BS-link availability from the normalized UAV-BS and BS-UAV
channel slices. Exact zero is reserved for the bridge's `-120 dB` no-link
sentinel; an available SNR clipped below the configured normalization range is
kept at `1e-6`, so a poor but available link remains selectable. Relay bit
`i -> j` is feasible only when both endpoints have a BS link, and upload bit
`i` follows UAV `i`'s BS availability. The availability vector is concatenated
to the 128-wide fused actor feature, and infeasible logits are masked before
sampling/log-probability/entropy evaluation, giving them probability exactly
zero during rollout, PPO/CRPO updates, and inference.

## Objective, task loss, and CRPO

`RacerCRPOEnv.step()` uses only measured BS PRB-slot deltas:

```text
C_BS = C_UAV_to_BS + C_BS_to_UAV
reward = -C_BS / bs_resource_normalizer
cost = L_task(t+1) - L_task(t)
L_task = alpha_traj D_traj + alpha_cov D_cov
       + alpha_red D_red + alpha_map D_map
```

The task constraint uses a fixed exploration horizon (`300 s` in
`environment.backend.fixed_exploration_time_s`), not completion time. At every
small-model action, `ReferenceTaskMetricEvaluator` updates four quantities:

```text
D_env  = ||workspace_max - workspace_min||_2
D_traj = sum_i sum_k ||p_i,k - p_i,k,pc||_2 / (N K_t D_env)
D_cov  = integral_0^t [C_pc(tau)-C(tau)]+ d tau
         / (integral_0^t C_pc(tau) d tau + epsilon)
R_red  = 1 - |union_i V_i,obs| / sum_i |V_i,obs|
D_red  = [R_red-R_red,pc]+ / (1-1/N-R_red,pc+epsilon)
IoU    = |V_BS,occ intersection V_GT,occ|
         / |V_BS,occ union V_GT,occ|
D_map  = [IoU_pc-IoU]+ / (IoU_pc+epsilon)
```

Perfect positions are linearly interpolated by simulation time. Coverage and
the two perfect diagnostic curves use hold-last interpolation; no artificial
coverage samples are generated between mapper updates. Every degradation is
finite and clipped to `[0,1]`. Defaults are
`(alpha_traj, alpha_cov, alpha_red, alpha_map) = (0.30, 0.30, 0.10, 0.30)`, so
`L_task` is also in `[0,1]`. Trajectory and coverage are the primary exploration
KPIs; redundancy receives less weight because it partially overlaps them, and
map IoU receives 0.30 so resource minimization cannot simply suppress all BS
uploads.

The signed cost telescopes exactly:

```text
sum_t c_t = sum_t (L_task(t+1)-L_task(t))
          = L_task(T_fix)-L_task(0).
```

The episode tracker checks this identity at termination and emits a warning if
the configured `constraint.telescoping_tolerance` is exceeded. The constrained
quantity is the final task loss, including the explicitly retained initial
constant: `E[L_task(T_fix)] <= Gamma_task`.

The total 100 MHz carrier is split only in BS-assisted modes: UAV UDP
broadcasts use a dedicated 50 MHz / 33-PRB subband with fixed NR MCS 14, while
all UAV-to-BS and BS-to-UAV traffic shares the other 50 MHz / 33-PRB subband
and selects MCS adaptively from link SNR. The pure distributed baseline keeps
the original 100 MHz / 66-PRB PHY. Sionna SNR is referenced to 100 MHz and the
proxy applies the corresponding noise-bandwidth correction for each 50 MHz
radio. UL and DL queues reserve one common BS serialization timeline, so they
cannot each claim the full BS subband concurrently; the UAV broadcast subband
can operate at the same time.

The C++ proxy counts a transmission attempt from its actual serialized packet
duration and the route's 33-PRB geometry. An owner upload is deduplicated at the BS,
so selecting several `B[i,j]` values does not charge that upload several times.
Direct-link PRB slots are kept in the separate `C_U2U` diagnostic and never
enter `C_BS` or reward.

`Gamma_task` is resolved before training. A non-null
`constraint.gamma_task` is authoritative; the current experiment value is
`0.15`. If it is `null`, paired deterministic results are used:

```text
Gamma_task = L_full + rho (L_dist - L_full),   rho = 0.25.
```

`distributed_only` emits an all-zero BS action while original Direct U2U keeps
running. `full_bs` emits every off-diagonal relay and every upload bit; the
proxy still enforces channel, queue, deduplication, and half-duplex feasibility.

`CRPOPPO` subclasses SB3 PPO without changing site-packages. Its local rollout
buffer stores cost values/returns/advantages and computes independent reward
and cost GAE. `CRPOActorCriticPolicy` adds `V_c` beside PPO's actor and `V_r`;
both value losses are optimized in every mode. Before each PPO update, the
mission-level estimator selects:

- reward mode when `J_C_hat <= Gamma_task + eta`, using `A_reward`;
- cost rectification otherwise, using `-A_cost`.

`EpisodeCostTracker` survives rollout boundaries, retains the latest complete
missions, and checks `sum(cost) = L_final - L_initial`. Before the first
complete mission, the source is explicitly logged as a cost-critic estimate.
Models contain the actor, both critics, optimizer, CRPO counters and tracker;
the run directory additionally stores resolved normalization/Qwen config and
episode statistics.

## Files and integration boundaries

- `agentic_crpo/`: Qwen, state builders, Gym environment, backend bridge,
  CRPO policy/buffer/algorithm, training and evaluation CLIs.
- `src/racer_sionna_comm/src/communication_proxy_node.cpp`: optional action
  reader, owner-aware upload/relay scheduling, AoI/queue state, and separate
  actual BS/Direct resource counters.
- `src/racer_isaac_adapter/isaac_sim/original_racer_isaac.py`: optional atomic
  mission-state summary writer.
- `run_warehouse_simple_sionna.sh`: forwards the bridge paths only when
  `RACER_RL_BS_SCHEDULER_ENABLED=true`.

All new runtime switches default to disabled, so existing ideal/Sionna runs
follow their previous scheduler and output path.

## Environment and tests

The isolated environment prepared on this machine reuses the local Qwen
environment and adds SB3/Gymnasium without modifying SB3 itself:

```bash
source /home/jiazheng/ai_envs/racer-crpo/bin/activate
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/hybrid_communication_racer
python -m pip install -e './agentic_crpo[fp8,vllm]'
PYTHONPATH=./agentic_crpo pytest -q agentic_crpo/tests
python -m agentic_crpo.train_crpo --config agentic_crpo/config.yaml --mock-smoke
```

The large checkpoint test is opt-in because it occupies GPU memory:

```bash
RUN_QWEN_LOAD_TEST=1 PYTHONPATH=./agentic_crpo \
  pytest -q -s agentic_crpo/tests/test_qwen_global_agent.py -k local_load
```

Build and verify the ROS integration with:

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select racer_sionna_comm racer_isaac_adapter --symlink-install
source install/setup.bash
colcon test --packages-select racer_sionna_comm
colcon test-result --verbose
```

## Formal training and evaluation

First prepare the GT/reference as described below. Use the same
scenario/start positions/seed family and start the simulator in terminal 1:

```bash
RACER_COMMUNICATION_MODE=sionna \
RACER_NETWORK_TOPOLOGY=bs_round_robin \
RACER_RL_BS_SCHEDULER_ENABLED=true \
RACER_FIDELITY_DRONE_COUNT=10 RACER_FIDELITY_DURATION=300 \
RACER_STOP_ON_COMPLETION=0 \
RACER_RANDOM_SEED=42 \
RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH=/absolute/path/to/gt_occupied_voxels.txt \
RACER_REQUIRE_GROUND_TRUTH_MAP=true \
RACER_RESULT_DIR="$PWD/agentic_crpo/runs/formal_seed42_sim" \
./run_warehouse_simple_sionna.sh
```

After the bridge state appears, start training in terminal 2. Size wall time from measured trainer
FPS and Isaac real-time factor: approximately
`training_wall_s = total_timesteps / trainer_fps` and
`sim_duration_s >= training_wall_s * real_time_factor`, with margin. A single
long mission can then be trained with:

```bash
source /home/jiazheng/ai_envs/racer-crpo/bin/activate
python -m agentic_crpo.train_crpo \
  --config agentic_crpo/config.yaml \
  --perfect-reference /absolute/path/to/perfect_result.json \
  --total-timesteps 40000 \
  --output-dir "$PWD/agentic_crpo/runs/formal_seed42"
```

For multiple independent missions, launch a fresh simulator per seed and use
`--resume .../crpo_final.zip`; this preserves CRPO counters and the
mission-level tracker. Evaluate deterministically against an active matching
simulator with:

```bash
python -m agentic_crpo.eval_crpo \
  --config agentic_crpo/config.yaml \
  --perfect-reference /absolute/path/to/perfect_result.json \
  --checkpoint agentic_crpo/runs/formal_seed42/crpo_final.zip \
  --episodes 1
```

The real backend intentionally refuses to train when the Isaac mission-state
file or a reference containing trajectory, coverage, measured redundancy, and
GT-based BS-map IoU histories is missing. This prevents silent use of zero or
surrogate task loss.

## GT map and perfect-reference preparation

`ChunkData.voxel_adrs` uses the RACER mapper's global linear voxel address. The
C++ proxy maintains each origin UAV's observed-address set for `R_red`, applies
only successfully delivered UAV-to-BS `voxel_occ` updates to the BS occupied
set, and reads the GT occupied set from a whitespace-separated `uint32` text
file. In ideal mode it additionally maintains the perfect global aggregator.
The GT, perfect curves, and resulting degradations stay in loss-only telemetry;
neither state builder reads them (covered by an isolation test).

An existing old reference without these histories is rejected. Prepare a
matching 300-second reference in two passes. Pass 1 exports the final occupied
set; pass 2 uses that fixed file as GT and produces the actual perfect curves.
Keep scene, starts, UAV count, seed, duration, and all RACER settings identical:

```bash
export REF_ROOT="$PWD/agentic_crpo/references/perfect_seed42"
mkdir -p "$REF_ROOT/gt_pass" "$REF_ROOT/reference_pass"

RACER_COMMUNICATION_MODE=ideal RACER_REQUIRE_SIONNA=false \
RACER_NETWORK_TOPOLOGY=distributed RACER_FIDELITY_DRONE_COUNT=10 \
RACER_FIDELITY_DURATION=300 RACER_RECORD_TRAJECTORY_HISTORY=1 \
RACER_STOP_ON_COMPLETION=0 \
RACER_RANDOM_SEED=42 RACER_RESULT_DIR="$REF_ROOT/gt_pass" \
RACER_OBSERVED_OCCUPIED_VOXELS_PATH="$REF_ROOT/gt_occupied_voxels.txt" \
./run_warehouse_simple_sionna.sh

RACER_COMMUNICATION_MODE=ideal RACER_REQUIRE_SIONNA=false \
RACER_NETWORK_TOPOLOGY=distributed RACER_FIDELITY_DRONE_COUNT=10 \
RACER_FIDELITY_DURATION=300 RACER_RECORD_TRAJECTORY_HISTORY=1 \
RACER_STOP_ON_COMPLETION=0 \
RACER_RANDOM_SEED=42 RACER_RESULT_DIR="$REF_ROOT/reference_pass" \
RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH="$REF_ROOT/gt_occupied_voxels.txt" \
RACER_REQUIRE_GROUND_TRUTH_MAP=true \
./run_warehouse_simple_sionna.sh
```

For real BS-assisted/baseline runs, set the same
`RACER_GROUND_TRUTH_OCCUPIED_VOXELS_PATH` and
`RACER_REQUIRE_GROUND_TRUTH_MAP=true`.

## Baseline calibration and training commands

Start a fresh matching simulator for each policy/seed, then run one evaluator
against it. For example:

```bash
python -m agentic_crpo.eval_baseline --config agentic_crpo/config.yaml \
  --policy distributed_only --seed 42 --episodes 1 \
  --perfect-reference /absolute/path/to/perfect_result.json \
  --output agentic_crpo/baselines/seed42_distributed_only.json

python -m agentic_crpo.eval_baseline --config agentic_crpo/config.yaml \
  --policy full_bs --seed 42 --episodes 1 \
  --perfect-reference /absolute/path/to/perfect_result.json \
  --output agentic_crpo/baselines/seed42_full_bs.json
```

To enable automatic calibration, set `constraint.gamma_task: null` and list all
paired JSON files under `distributed_only_results` and `full_bs_results`, then:

```bash
python -m agentic_crpo.calibrate_constraint \
  --config agentic_crpo/config.yaml \
  --output agentic_crpo/baselines/calibration.json
```

A short code-path check (mock) and a real/formal run are:

```bash
python -m agentic_crpo.train_crpo --config agentic_crpo/config.yaml \
  --mock-smoke --gamma-task 0.15

python -m agentic_crpo.train_crpo --config agentic_crpo/config.yaml \
  --perfect-reference /absolute/path/to/perfect_result.json \
  --gamma-task 0.15 --total-timesteps 40000 \
  --output-dir agentic_crpo/runs/formal_seed42
```

TensorBoard records all `constraint/*`, `task/*`, and `communication/*` fields;
episode summaries additionally include final loss, summed signed cost, and
total BS resource. `C_U2U` is diagnostic only.
