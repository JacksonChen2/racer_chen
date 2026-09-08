# Pairwise Robust RACER

本工作区从 `ros2_original_fidelity_sionna_ws` 的当前源码复制而来，未复制实验结果、分析结果、`build/`、`install/` 和 `log/`。原工作区保持不变。

## 设计边界

本版本保留 RACER 的 HGrid、两两 ACVRP 分配、frontier scoring、局部 viewpoint refinement 和 trajectory planning。任务协调默认直接进入两两分配；全局消息只在空任务 UAV 连续多轮无法通过 pair 获得任务后用于恢复，或在全局连续确认无任务时发布最终完成结论。

## 修改内容

1. 栅格 ID 的通信字段由 `int8[]` 改为 `int32[]`，避免 ID 大于 127 时溢出。
2. pair-opt 在调用 ACVRP 前记录 attempt 时间；失败或收益不足也进入冷却，不再由 20 Hz timer 忙循环重复计算。
3. 每架 UAV 的分配携带 `assignment_epoch`。两两分配使用 `PROPOSE/PREPARED/COMMIT/COMMITTED` 两阶段事务，双方先校验旧 epoch，再提交同一个新 epoch；COMMIT 可幂等重发，也可由 DroneState 的 epoch 独立确认。
4. `fsm.initial_partition_enabled` 默认关闭，启动不再等待首个 Global epoch 或全队 ACK；显式打开时只保留为旧实验兼容路径。
5. 成功分配后设置 commitment，普通 pair-opt 在承诺期内不打乱任务。空任务 UAV 先进入 `PAIR_SEARCH`；达到失败阈值且至少检查过一个新鲜 peer、当前没有事务时，才发布 global recovery 请求。
6. UAV 1 在一个窗口内合并 recovery 请求，优先分配未认领任务，其次从负载大于 1 的 donor 转移少量最近任务。正常 UAV 的其余 ownership 保持不变。
7. Recovery 消息携带用途、requester/affected UAV 和生成时 assignment epoch；延迟消息不能覆盖快照后完成的 pair commit，且只取消与 affected UAV 冲突的事务。
8. 只有全队状态新鲜、所有 UAV 均无 ownership、全局 frontier/grid pool 连续多轮为空时才进入 `FINISH`。
9. 最终 task/cost comparison 前，对所有 candidate viewpoints 做一次当前已知自由空间连通域搜索。UNKNOWN、inflated obstacle 和 planning boundary 外部均不可通过；每个 frontier 只保留可达候选。本轮不可达不会改图或进入永久 blacklist。若暂时没有可达 frontier，可先执行短距离的 known-free local expansion，再重新搜索。

## 主要参数

参数位于 `src/racer_isaac_adapter/config/original_warehouse_simple.yaml`：

- `fsm.attempt_interval`
- `fsm.pair_opt_interval`
- `fsm.pair_transaction_timeout`
- `fsm.pair_retry_interval`
- `fsm.assignment_commitment_duration`
- `fsm.pair_min_improvement`
- `fsm.pair_switch_penalty`
- `fsm.work_steal_failure_threshold`
- `fsm.work_steal_idle_delay`
- `fsm.initial_partition_enabled`
- `fsm.initial_partition_interval`
- `fsm.initial_partition_balance_weight`
- `fsm.global_recovery_pair_fail_threshold`
- `fsm.global_recovery_cooldown_cycles`
- `fsm.global_recovery_coalesce_cycles`
- `fsm.global_recovery_max_tasks_per_idle`
- `fsm.global_finish_confirmation_cycles`
- `reachability.connectivity_max_nodes`
- `reachability.local_expansion_min_dist`
- `reachability.local_expansion_max_dist`

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```
