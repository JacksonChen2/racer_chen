# Event-driven one-shot BS scheduling experiment

This is an opt-in A/B path. The existing `shared_memory` backend and the
fixed 100 ms decision / five 20 ms slots / five-slot action hold remain the
default. The event experiment is selected only by the new backend kind and
runtime flag.

## Cycle semantics

```text
emit state S_k
  -> no BS scheduling while actor/learner computes
  -> accept action A_k only if its state/version/slot provenance is S_k
  -> call the BS action scheduler once at the next open 20 ms PHY slot
  -> transmit/retry/settle only packets attributed to A_k
  -> emit S_(k+1) after all A_k requests and packets have settled
  -> repeat
```

An action is never held or replayed in later scheduler ticks. Direct U2U
simulation continues normally; BS periodic uploads and reserved BS control
traffic are rejected at startup in this mode so no unrelated BS work can run
during actor inference. A local chunk-payload request is part of the action's
result barrier and retains its existing bounded timeout.

The state-to-state transition duration is measured from simulation time:

```text
delta_t = result_sim_time - source_state_sim_time
```

It therefore includes actor/update latency, waiting for the next open PHY
slot, and the selected transmissions' completion time. Reward, unchanged
task metrics, and resource counters cover that exact interval. The dedicated
event rollout buffer scales reward/cost gamma and GAE lambda by
`delta_t / 0.1 s`; the numerical PPO/CRPO hyperparameters remain identical to
the fixed-clock comparison configuration.

## Run

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen
RACER_QWEN8_EPISODES=1 \
  final_racer_training/scripts/run_event_driven_one_shot_campaign.sh
```

The event entry uses:

- backend: `shared_memory_event_driven_one_shot`
- supervisor: `agentic_crpo.event_driven_llm_prior_process_supervisor`
- trainer: `agentic_crpo.train_event_driven_llm_prior_crpo`
- runtime flag: `RACER_RL_BS_EVENT_DRIVEN_ONE_SHOT=true`

The comparison config preserves the current reward, constraint, action space,
policy/critic architecture, and PPO/CRPO values. The only config differences
from the fixed experiment are the variant name, backend kind, output paths,
and disabled single-GPU pause file names.

Useful event logs are `RACER_EVENT_ACTION_ONESHOT`,
`RACER_EVENT_TRANSITION_COMPLETE`, `RACER_EVENT_RL_ACTION_CYCLE`, and
`event/delta_t_{min,mean,max}_s`.
