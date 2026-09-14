# Synchronous online Qwen + CRPO/PPO training

The Qwen-8B task-loss campaign uses a simulation-time barrier with

`T_comm = 0.02 s`, `T_RL = 0.10 s`, and `T_RL = 5 T_comm`.

The current Qwen8B task-loss launcher also sets
`RACER_COVERAGE_UPDATE_RATE_HZ=10`. Each UAV therefore computes and publishes
coverage every 100 ms of ROS simulation time, aligned with the RL decision
interval. PPO/Qwen wall-clock latency cannot advance this timer while the
simulation is frozen.

At `t=0`, Isaac publishes `/clock`, odometry, and mission state without a
physics step. The communication proxy publishes `s_0`, then Isaac writes a
`paused` acknowledgement. The trainer writes both an action tagged with the
current RL decision index and a matching release record. Only then can Isaac
advance.

For each decision interval, the communication proxy holds that action for
five communication slots. It still examines current queues, channel state,
and pending chunks on every slot, so holding an action does not duplicate a
packet or force retransmission. At the fifth slot it publishes the next state
and the resource deltas accumulated over the interval. Isaac publishes the
matching mission/task state at the same simulation time and freezes again.

The resulting trainer step is exactly

`(s_t, a_t, r_t, c_t, s_(t+1), done)`

over 100 ms of simulation time. The file bridge rejects a state that skips a
decision, advances by anything other than five slots, advances by anything
other than 100 ms, or reports mismatched interval/cumulative resources.

When the rollout contains 384 transitions, `env.step()` has already returned
the acknowledged boundary state while Isaac remains paused. PPO/CRPO trains
at that boundary. The next policy action releases the simulator after the
update. A terminal rollout shorter than 384 is sampled over its valid prefix
and trained once before shutdown. A resumed checkpoint retains model,
optimizer, counters, and prior statistics, but discards the prior episode's
terminal observation and stop flags so the new Isaac process is reset at its
own exact `s_0`.

## Logs

- `RACER_RL_SYNC_BOUNDARY_PAUSED` / `RESUMED`: Isaac boundary, state sequence,
  communication slot, decision index, and unchanged simulation time.
- `RACER_RL_SYNC_ACTION`: action epoch and the boundary it releases.
- `RACER_RL_TRANSITION`: simulation time, slot, decision, held-slot count,
  transition duration, done, and rollout size.
- `RACER_PPO_UPDATE_BEGIN` / `END`: update boundary before and after training;
  the end record contains `simulation_time_frozen`.

Every update record is also stored in `training_state.json` under
`synchronous_ppo_updates`.

## Validation performed

The short real-stack validation is in
`validation/synchronous_online_qwen8b_smoke_20260901` (relative to the project
root). It ran Isaac, Sionna, Qwen-8B, and CRPO/PPO for 2 simulated seconds.
For test speed only, it used `n_steps=8`, batch size 4, and one PPO epoch; the
formal configuration remains `n_steps=384`, batch size 64, and ten epochs.

Observed results:

- 20 transitions from 2.0 / 0.1, with slots 5 through 100 and decisions 1
  through 20;
- every transition reported five held slots and a 100 ms interval;
- rollout updates had sizes 8, 8, and terminal remainder 4;
- all three updates retained identical simulation time, communication slot,
  and decision index before/after training and reported
  `simulation_time_frozen=true`;
- the final terminal boundary was simulation time 2.0, slot 100, decision 20;
- the communication proxy reported zero missing-action slots.

The ROS communication integration test additionally freezes `/clock` for 350
ms of wall time and verifies the state file is byte-for-byte unchanged, then
advances to the next exact boundary. Run the checks with:

```bash
cd agentic_crpo
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /home/jiazheng/ai_envs/racer-crpo/bin/python -m pytest -q

cd ..
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon test --packages-select racer_sionna_comm
colcon test-result --verbose
```

A 300 s episode therefore produces 3000 transitions. With `n_steps=384`, it
performs seven full 384-transition updates plus one valid 312-transition
terminal update, assuming a single environment.
