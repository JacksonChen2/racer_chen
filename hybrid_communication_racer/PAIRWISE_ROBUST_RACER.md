# Pairwise Robust RACER

本工作区从 `ros2_original_fidelity_sionna_ws` 的当前源码复制而来，未复制实验结果、分析结果、`build/`、`install/` 和 `log/`。原工作区保持不变。

## 设计边界

本版本保留 RACER 的 HGrid、两两 ACVRP 分配、frontier scoring、局部 viewpoint refinement 和 trajectory planning。全局消息只用于探索开始时的一次性空间初分区；此后任务调整仍由原 RACER 的两两分配完成，不是周期性全局调度。

## 修改内容

1. 栅格 ID 的通信字段由 `int8[]` 改为 `int32[]`，避免 ID 大于 127 时溢出。
2. pair-opt 在调用 ACVRP 前记录 attempt 时间；失败或收益不足也进入冷却，不再由 20 Hz timer 忙循环重复计算。
3. 每架 UAV 的分配携带 `assignment_epoch`。两两分配使用 `PROPOSE/PREPARED/COMMIT/COMMITTED` 两阶段事务，双方先校验旧 epoch，再提交同一个新 epoch；COMMIT 可幂等重发，也可由 DroneState 的 epoch 独立确认。
4. UAV 1 在开始阶段按起飞位置和 HGrid 中心做一次均衡、空间连续的多机初分区。确认各 UAV 已应用后，恢复纯两两优化。
5. 成功分配后设置 commitment，普通 pair-opt 在承诺期内不打乱任务。持续规划失败或无任务的 UAV 会发布 `requesting_work`，并通过原两两分配从有工作的 UAV 获取栅格。
6. 最终 task/cost comparison 前，对所有 candidate viewpoints 做一次当前已知自由空间连通域搜索。UNKNOWN、inflated obstacle 和 planning boundary 外部均不可通过；每个 frontier 只保留可达候选。本轮不可达不会改图或进入永久 blacklist。若暂时没有可达 frontier，可先执行短距离的 known-free local expansion，再重新搜索。

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
- `reachability.connectivity_max_nodes`
- `reachability.local_expansion_min_dist`
- `reachability.local_expansion_max_dist`

## 构建

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

