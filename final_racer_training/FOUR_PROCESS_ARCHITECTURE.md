# Four-process shared-memory runtime

## Process creation and lifecycle

`agentic_crpo.process_supervisor` creates a unique tmpfs directory, initializes
the fixed shared-memory ABI, and starts four children with separate process
groups and private address spaces:

| Child | Entry point | Private state |
|---|---|---|
| Isaac | `runtime/run_final_racer_training.sh` with `RACER_PROCESS_ROLE=isaac` | SimulationApp, scene, renderer, sensor and rigid-body objects |
| RACER | the same runner with `RACER_PROCESS_ROLE=racer` | ROS nodes, A*/ESDF/B-spline/planner objects, communication queues |
| LLM | `agentic_crpo.llm_worker` | Qwen model, tokenizer, vLLM engine, KV cache and CUDA context |
| RL | `agentic_crpo.train_crpo` | policy/value networks, optimizer, gradients, rollout buffer and CUDA/CPU tensors |

The supervisor contains no planner or learning decisions. It monitors child
pidfds, publishes the shutdown flag, wakes futex waiters, terminates process
groups, assembles the result, closes mmaps, and removes the unique tmpfs
directory. Signals are received through a selector wake pipe, not a sleep loop.

ROS 2 RACER is intentionally one supervised subsystem/process group rather
than literally one Unix PID: `ros2 launch` retains the original separate RACER
nodes. Converting every upstream node into one component process would alter
the planner runtime and is outside this isolation change.

## Shared blocks and ownership

Ordinary state/mailbox blocks have one writer after initialization, a 128-byte
ABI header, two fixed-capacity payload slots, CRC32, active slot, monotonic
version, source `sim_step`, `sim_time`, writer PID, and a futex notification
word. `transition_ring` and `task_metric_ring` each have 4096 ordered
fixed-size records, enough for every 100 ms boundary in a complete 300 s
episode.

| Block | Sole writer | Readers | Main payload |
|---|---|---|---|
| `physical` | Isaac | result/debug tooling; LLM reads terminal flags only | simulator positions/FSM, union coverage, compact per-UAV map counts, terminal flags |
| `physical_fast` | Isaac | RACER proxy | fixed binary task clock, coverage, and terminal flags |
| `communication` | RACER task worker | LLM | low-frequency BS map, BS-received FSM/trajectory, receiver-aware missing bytes, AoI and channel state |
| `state_event` | Isaac and RACER (notification word only) | LLM | wakeup when either independently owned state block changes; it carries no state payload |
| `guidance` | LLM | RL | validated `W_task`, `omega_sem`, guidance/source versions and inference timing |
| `guidance_fast` | LLM | RACER proxy | fixed binary `W_task`/`omega_sem` embedded in subsequent Fast RL State |
| `action` | RL | RACER proxy | action matrix plus action/source versions |
| `action_ack` | RACER proxy | debuggers only | non-blocking diagnostic of boundary activation; RL does not read it |
| `transition_ring` | RACER Fast-State publisher | RL actor | fixed binary positions/channel/AoI/missing/queue/guidance and executed-action metadata |
| `task_metric_ring` | RACER task worker | RL trainer | asynchronous map/redundancy/coverage inputs keyed by `step_id`/`sim_time` |
| `shutdown` | supervisor | all workers | shutdown reason/flag |
| `status_*` | named process | supervisor/debuggers | ready/stopping/PID status |

Fast State never traverses a map, trajectory, communication history, or packet
queue. Missing bytes/chunk gaps and BS-route queue bytes are updated at chunk
and queue events; AoI is `sim_time-last_success_time`. Qwen's finite channel
window remains a bounded private `deque(maxlen=channel_history)`.

The writer serializes completely into the inactive slot while holding an
exclusive `flock`, then commits active index/version/futex together. Readers
take a shared lock only long enough to copy a complete active slot. A composite
physical+communication snapshot briefly locks both blocks in deterministic
order, then all preprocessing/inference happens on private copies. The LLM
snapshot boundary discards physical positions, FSM, local-map summaries and
union coverage; only terminal flags cross from `physical`. Map progress,
telemetry and trajectory fields come from the BS-observable `communication`
block. Detailed channel/AoI arrays are reduced to per-UAV summaries before the
prompt is formatted.

## LLM cycle

There is one ordinary loop and no inference thread or request queue. At the
start of a cycle it atomically copies the newest physical+communication tuple,
records those source versions, builds the existing large-model state, performs
one Qwen inference, validates it, and commits one guidance version. Afterward it
checks the current versions: if they advanced during inference it immediately
uses the newest tuple; otherwise it blocks in `FUTEX_WAIT` until RACER publishes
or the supervisor wakes it for shutdown. Intermediate versions are not queued.

`RACER_LLM_CYCLE_BEGIN/END/ERROR` include `llm_cycle_id`, physical,
communication/racer/map sources, simulation step/time, wall times, guidance ID,
latency, and skip counts.

## Asynchronous RL action and transition flow

RL uses the latest completed guidance when forming each observation. Policy
inference produces one action, and the backend atomically commits an action
mailbox value containing:

```text
action_id, source_physical_version, source_communication_version,
source_guidance_id, source_sim_step, policy_version,
generated_wall_time_ns, source_sim_time_s, source_step_id, action bits
```

Publication is fire-and-forget: RL never waits for `action_ack`. The proxy
copies new mailbox values into `latest_action`, promotes only the newest value
to `active_action` at a 100 ms boundary, and holds it for the following five
20 ms radio slots. Every slot invokes upload/relay scheduling from the active
action even if no new mailbox version arrived. Existing known-chunk and
pending-chunk checks prevent a held action from duplicating packet data.

At every 100 ms boundary the scheduler copies bounded caches into a
`FastBoundarySnapshot`; an independent callback publishes its fixed binary ABI
to the ordered transition ring before the next action is activated. Its metadata
contains `action_version`, `policy_version`, `action_age`, `sim_timestamp`,
the actual relay/upload bits and five-slot hold count. Thus two intervals that
both execute `a0` produce two separate records. PPO/CRPO consumes the ring in
order during rollout collection. During an optimizer update, a control-only
pump continues consuming new boundaries and publishing fresh actions without
adding those transitions to a training buffer.

`RACER_RL_CYCLE_BEGIN`, `RACER_RL_ACTION_CYCLE`, and
`RACER_RL_TRANSITION` include the RL cycle/action IDs, source versions,
guidance ID, policy timing, action publish time, executed action/policy version,
action age, result versions, ring backlog, and skip counts.

During a rollout, the RL process keeps its existing main thread as the
transition collector and adds one policy-inference thread. A
preallocated two-slot SPSC exchange copies each new observation into a fixed
slot. The inference worker performs one shared-encoder `forward_crpo()`, packs
its proposed action, reward/cost values, old log probability and masked logits
into one CPU transfer, publishes the action without reading an ACK or waiting
for another state, and returns a DecisionRecord keyed by action version, source
step and source simulation time.
The backend's former combined `step()` is correspondingly split into a
non-blocking `publish_action()` and an ordered `collect_transition()`; `step()`
remains only as a compatibility wrapper.

As soon as the collector receives `s_(t+1)`, it queues inference for that state
before finalizing transition `t`. Fresh transition bookkeeping therefore runs
on the main RL thread while the next policy inference runs in the inference
worker. If executed action bits, action version, source step and source
simulation timestamp all match, the collector writes the DecisionRecord's CPU
values directly without `evaluate_actions_crpo()` or another cost-value
prediction. For a stale/held action, the collector preserves the same
current-state masked-distribution semantics by scoring the actual action from
the cached CPU logits. It performs no policy forward, CUDA transfer or policy
lock, so stale bookkeeping cannot delay the next action publication.
The RL entry point fixes both PyTorch intra-op and inter-op CPU thread counts
to one before constructing the environment or policy.

When the rollout closes, the optimizer-owned `learner_policy` is rebased to
the frozen `behavior_policy` weights and updated in a learner thread. A small
update control thread consumes the newest boundary while the existing
latest-only inference worker runs the frozen behavior model. The optimizer has
no reference to behavior parameters. After all learner epochs, a new frozen
model is prepared and its reference is swapped only after the last behavior
forward completes. A separate high-priority CUDA stream is used for behavior
inference when learner and behavior share a GPU, and learner minibatches yield
between optimizer steps.

`RACER_RL_INFERENCE` records inference/queue/action-publication durations and
confirms that neither ACK nor next state was awaited. `RACER_RL_TRANSITION`
records finalize start/end, inference handoff wait, measured overlap with the
next inference, fresh/stale processing duration, total RL-cycle duration and
whether either side observed blocking. TensorBoard receives rollout means,
counts and maxima for these timing series.

## Three state chains and asynchronous CRPO cost

The 10 Hz Fast RL State contains only positions, directed channel state, AoI,
pair/BS missing bytes, BS upload/relay queue bytes, and the latest embedded
Qwen guidance. Payload length depends only on configured UAV count.

The low-frequency LLM State is produced at the configured high-level period
(5 s for `high_level_interval=250` with 20 ms slots). It contains FSM,
trajectory/map summaries, and information-version gaps, and never enters actor
inference.

Every Fast boundary also appends a FIFO marker to the task-observer worker.
That worker owns private voxel mirrors, computes BS map coverage and redundant
exploration, and publishes `task_metric_ring`. The map-loss component uses
`D_map(t) = max(0, C_joint(t) - C_BS(t))`, where both coverages count known
FREE+OCCUPIED voxels over the same planning-box denominator. Transition `k` is
first stored with `cost_pending=true`; after rollout collection stops, the
unchanged task-loss evaluator resolves the metric for `s_{k+1}` and backfills
`c_k` by transition `step_id`. Cost GAE, minibatch sampling, and CRPO update
all fail closed while any cost remains pending.

The proxy uses a four-thread ROS 2 `MultiThreadedExecutor` with separate
mutually-exclusive groups for scheduler/action apply, packet RX/TX, Fast State
publication, and telemetry. Heavy task work is outside the executor and never
takes the communication-state mutex. Logs expose `fast_rl_state_build_ms`,
`fast_rl_state_publish_ms`, `action_apply_ack_ms`, `wait_next_state_ms`,
`task_metric_compute_ms`, `missing_bytes_update_ms`, and
`queue_cache_update_ms`; the first five also report running mean and maximum
latency so long-map regressions can be detected from one episode log.

## Latest-state-wins and PPO

LLM and RL maintain independent last-consumed physical/communication versions.
Neither writes a global consumed flag. Futex notifications are hints; versions
are authoritative, so coalesced wakeups cannot create a stale request queue.

The bottom-layer communication remains 50 Hz (`T_comm=20 ms`) and Isaac's
physical block is published at 50 Hz. The complete RACER model feature snapshot
and transition sampler run on exact 100 ms boundaries (`T_RL=5*T_comm=100 ms`).
The action is zero-order held between boundaries. Isaac/RACER continue during
Qwen inference and PPO updates. During an update, the frozen policy keeps
mapping each latest state to a newly published action; superseded unstarted
states may still be skipped. Those control-only transitions are logged but
never enter the next rollout. After the atomic `k -> k+1` swap, the rollout
buffer restarts from the latest observation and accepts only executed actions
whose `policy_version` is `k+1`; any retained old-version boundary is skipped.
The update refuses a buffer containing any other policy version. PPO logs
record start/end versions, actions generated during the update, maximum action
age, inference latency, storage isolation, parameter immutability and swap
success.

## Unchanged and residual isolation boundaries

The entire `../final_racer` directory, upstream RACER planner/core, scenes,
vehicle assets, Sionna runtime, Qwen checkpoint, reward definition, CRPO/PPO
math, state dimensions, and policy architecture remain unchanged. Changes are
confined to this training directory's Isaac adapter overlay, its local copy of
the communication proxy, Python IPC/workers, supervisor, and launch scripts.

No model object, optimizer, rollout buffer, planner object, simulator object,
or CUDA tensor crosses a process boundary. Each CUDA-using child creates its
own context; there is no CUDA IPC and no forced global GPU serialization.
Single-GPU contention may increase wall time and therefore increase version
skips, which is observable in logs.

Shared-memory mode has a fail-fast invariant in both the launcher and Isaac
argument validation: neither the legacy single-GPU pause gate nor the legacy
synchronous-RL boundary gate may be enabled. Isaac logs
`ISAAC_TRAINING_SCHEDULING_MODE` at startup with both gates disabled. Qwen
inference, policy inference, and PPO updates can therefore slow
Isaac through ordinary GPU/CPU contention, but cannot intentionally stop
`world.step()` or wait for a model release token.

Remaining threads are internal to their original owner: Isaac's existing
sensor worker pool, ROS/middleware executor/runtime threads, RACER's existing
optional asynchronous task observer, and vLLM internals. They do not reference
another subsystem's Python objects or mutable model state. The legacy
file/synchronous runner still exists as an explicit fallback, but the default
four-process config does not enable its pause gates or file bridge.
