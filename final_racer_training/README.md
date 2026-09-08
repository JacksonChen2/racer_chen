# final_racer_training

This is the isolated training layer for the frozen `../final_racer` baseline.
The default runtime uses four independent top-level process groups:

1. Isaac Sim (physical simulation and sensors)
2. RACER/ROS 2 (unchanged planner plus the communication proxy)
3. Qwen3-8B-FP8/vLLM
4. dual-encoder CRPO/PPO

Versioned JSON mailboxes plus fixed binary Fast-State blocks/rings are
exchanged through mmap files under a unique `/dev/shm/final_racer_*` directory.
Model objects, CUDA tensors,
optimizer state, rollout buffers, planner objects, and Isaac objects remain in
their owning process. Linux futex wakeups provide event-driven notification;
there is no model scheduling poll loop or fixed model invocation period.

See [FOUR_PROCESS_ARCHITECTURE.md](FOUR_PROCESS_ARCHITECTURE.md) for the IPC
schema, lifecycle, asynchronous action/transition semantics, and remaining isolation
boundaries.

## Build and verify

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/final_racer_training
./scripts/build.sh
./scripts/check.sh
```

`training_overlay_ws` locally overrides only the Isaac adapter and
communication proxy. The RACER core still resolves from `../final_racer`.

## Train

The normal entry point now uses the four-process supervisor:

```bash
./scripts/run_qwen8b_fp8_training.sh
```

For one short integration episode:

```bash
RACER_QWEN8_EPISODES=1 RACER_QWEN8_DURATION=5 \
  ./scripts/run_qwen8b_fp8_training.sh
```

The former synchronous/file-bridge implementation remains available in
`config/qwen8b_fp8_latest.yaml` and `scripts/run_training_campaign.sh`; it was
not overwritten.

## Preserved algorithm settings

- local Qwen3-8B-FP8 through vLLM/CUTLASS FP8;
- 440 physical values + 110 guidance values, 550 total observations and 100
  binary actions for 10 UAVs;
- unchanged dual-encoder policy, reward, CRPO/PPO update equations, and
  communication PHY/MAC model; `D_map=max(0,C_joint-C_BS)` uses live known-map
  coverages with the same planning-box denominator;
- negative BS-resource reward;
- `Gamma_task=0.25`, task weights `(0.30, 0.30, 0.10, 0.30)`;
- PPO defaults: `n_steps=384`, batch 64, 10 epochs, learning rate `2e-4`,
  SiLU, `target_kl=0.03`, and `[128]` policy/value heads;
- bottom-layer communication slots at 20 ms / 50 Hz and Fast RL State plus
  transition sampling at 100 ms / 10 Hz;
- event-driven missing-byte/queue caches and asynchronous task-metric cost
  backfill keyed by canonical `step_id`/`sim_time`;
- coverage callback at 10 Hz;
- simulator time continues during Qwen inference and PPO updates;
- PPO/CRPO updates run on an optimizer-owned learner policy while a frozen,
  parameter-disjoint behavior policy keeps generating actions from new states;
- update-window behavior transitions are logged and excluded from the next
  on-policy rollout, which starts only after the atomic policy-version swap.

The model cycles themselves are non-periodic. A slow cycle skips superseded
input versions and immediately consumes the newest complete snapshot.
