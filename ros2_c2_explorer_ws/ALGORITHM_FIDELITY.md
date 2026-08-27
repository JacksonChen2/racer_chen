# 算法一致性说明

## 复现原则

ROS 2 版本不是重新用 Python 实现论文流程，而是直接编译原始 C² C++ 算法。`src/c2_explorer_core/upstream` 与 ROS 1 基准拥有相同的 405 个文件；其中 400 个文件逐字节一致，5 个文件只有经过审计的 ROS 2 字段/时间适配、健壮性保护和未定义行为确定化。`tests/check_source_fidelity.py` 固定了这五组基准与移植后哈希。

Isaac Sim Python 代码只位于传感器和动力学边界。占据栅格融合、ESDF、任务单元、分配、会合优化、路径搜索、轨迹优化均在原始 C² C++ 进程内执行。

## 论文逻辑到源码

| 论文中的 C² 模块 | 保留的核心实现 |
| --- | --- |
| Connectivity-aware task representation | `active_perception/src/uniform_grid.cpp` 中的 CCL 任务中心生成与 `updateConnectivityGraph`；`connectivity_graph.cpp` 中的图搜索；`hgrid.cpp` 中的图路径代价和相邻惩罚 |
| Contiguity-driven task allocation | `exploration_manager/src/c2_exploration_manager.cpp` 中的 `findGlobalTourOfGrid`、中心/网格代价矩阵和 ACVRP 分配 |
| Decentralized local task refinement | `c2_exploration_fsm.cpp` 中的会合检测、Proposal、响应、Commit、Commit-ACK、Finalize/重试状态 |
| Connectivity-constrained routing | `hgrid.cpp`、`uniform_grid.cpp` 和原始 LKH-3 ATSP/ACVRP 求解器 |
| CP-guided local exploration planning | `findGridAndFrontierPath`、`findTourOfFrontier`、`planTrajToView`、`frontier_finder.cpp` 和 `heading_planner.cpp` |
| Kinodynamic/B-spline trajectory generation | `path_searching`、`plan_manage`、`bspline`、`bspline_opt`、`poly_traj` |
| Decentralized map/state exchange | `multi_map_manager.cpp`、`communication_graph.cpp`、`DroneState`、`ChunkData`、`Heartbeat` 及共享 ROS 2 话题 |

## 保持不变的行为

- 每架 UAV 独立运行完整 C² FSM，ROS 2 兼容层使用单线程执行器，保持 ROS 1 单回调队列的串行语义。
- CCL/连通图任务单元、图上代价、相邻惩罚、中心一致性和 ACVRP 矩阵均来自原始源码。
- 会合优化的门控、提议、提交、确认、重试与回退逻辑不在桥接层重写。
- TSP/ACVRP 使用原始 C² vendored LKH-3。问题 1/2 由 ROS 2 服务调用；原始依赖独立进程全局状态的问题类型继续通过独立 CLI 生命周期处理。
- 位置与偏航轨迹仍由原始 kinodynamic A*、B 样条和 NLopt 链路生成；Isaac 仅执行最终 `PositionCommand`。
- 地图继续由原始 `SDFMap::inputPointCloud`、占据概率更新、膨胀和 ESDF 代码维护。

## Isaac 边界

- Isaac 发布世界坐标系 `PointCloud2` 与 `Odometry`；ROS 2 `MapROS` 只把它们送入原始 `SDFMap`。
- 使用仓库 RACER 复现中已经验证的 SO3/PhysX 无人机、深度相机、360° 低层安全雷达和碰撞统计，从而让两个算法共享相同的仿真执行条件。
- `/clock` 驱动所有 C² 定时器、轨迹时间和邻居超时，避免慢于实时的 Isaac 运行破坏算法时序。
- 安全制动位于飞行器执行层，不参与 C² 任务分配或规划代价；所有干预都会进入结果 JSON。

## 可自动核对的基准

```bash
python3 tests/check_source_fidelity.py
python3 tests/check_runtime_fidelity.py
```

预期输出：

```text
C2 source fidelity check passed: 405 files, 400 byte-identical, 5 audited boundary/guard edits
C2 runtime fidelity check passed: 13 interfaces field-equivalent, 136 ROS1 algorithm parameters equal, 5 Isaac boundary parameters audited
```

运行时审计还核对了 ROS 1 单回调队列语义：ROS 2 算法节点使用单线程执行器；点云回调在返回前完成占据栅格膨胀，ESDF 仍由 50 ms 定时器更新；LKH 服务失败通过兼容层恢复为 ROS 1 `ServiceClient::call()==false`；ACVRP 类型 3 在独立 LKH 进程运行前强制 `PRECISION=1`。因此前沿/FSM 定时器看到地图的先后顺序和求解失败分支均与原版一致。
