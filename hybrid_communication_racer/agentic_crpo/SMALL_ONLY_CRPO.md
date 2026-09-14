# Small-model-only CRPO/PPO variant

This is a separate baseline and does not replace either Qwen-14B or Qwen-8B.
It disables Qwen loading and inference, and removes the complete large-model
output from the small policy input.

For ten UAVs, the observation changes from 550 to 440 float32 values:

| Slice | Width | Contents |
|---|---:|---|
| `positions` | `3N` | normalized xyz |
| `channels` | `N^2+N` | U2U, UAV-BS, and BS-UAV SNR |
| `aoi` | `N^2` | pair and BS AoI |
| `missing` | `N^2` | missing bytes |
| `queues` | `N^2` | upload and relay queues |
The removed `guidance` slice is `N^2+N=110` values at `N=10`. Action mapping,
negative BS-resource reward, task-loss constraint (`Gamma_task=0.15`), radio
configuration, synchronous 20 ms/100 ms timing, PPO network, and training
hyperparameters remain identical to the latest Qwen-8B task-loss experiment.

Configuration:
`config_small_only_taskloss_sync20ep_lr1e4_b128_e5_kl005.yaml`

Launcher:
`scripts/run_small_only_taskloss_sync20ep_lr1e4_b128_e5_kl005.sh`
