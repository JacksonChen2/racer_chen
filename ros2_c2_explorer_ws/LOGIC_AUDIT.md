# C²-Explorer 逻辑一致性复审

复审日期：2026-08-26  
ROS 1 基准：`C2-Explorer` commit `fd1c76a49a4453f91da4984e123b56932c5382f3`  
论文：`c-explorer.pdf`，SHA-256 `0c50667957b13e8b2bbcc639d67d7b12eb5e7f0d4e3f1d3e255a73e4ba23f0df`

## 结论

修正后，ROS 2/Isaac 版本的 C² 算法决策逻辑与原始 ROS 1 实现一致。任务单元与连通图、连续性约束 ACVRP 分配、会合 Proposal/Commit/ACK 协议、CP 引导的前沿游历、kinodynamic A*、B 样条/NLopt 轨迹以及地图块交换都直接运行原始 C++ 源码，没有在 Python 或桥接层重写。

无法逐字节相同的部分只位于 ROS 2 IDL、时间/服务 API 和 Isaac 传感器/动力学边界；它们已逐项审计，所有会改变正常算法分支的非必要差异均已修正。

## 复审范围与结果

| 核对项 | 结果 |
| --- | --- |
| 原始算法源码 | 405 个文件；400 个逐字节一致，5 个为锁定双侧 SHA-256 的字段/时间/健壮性保护 |
| ROS 消息与服务 | 13 个接口字段类型、顺序、数组语义等价；仅 `time` 的 ROS 2 类型和非法尾下划线字段名适配 |
| ROS 1 启动参数 | 136 个算法参数逐项相等 |
| 论文主链路 | connectivity-aware representation、contiguity-driven allocation、会合优化、CP-guided planning、平滑轨迹均追踪到原始调用链 |
| 回调模型 | 单线程 executor 保持 ROS 1 串行 callback queue；地图膨胀/ESDF 顺序相同 |
| LKH | 原始 vendored LKH-3；TSP/ACVRP 服务名、问题类型、独立进程和失败分支相同 |

## 发现并修正的不一致

1. 占据栅格配置曾使用 RACER 模板值。现已恢复原始 `single_drone_planner.xml` 的 `p_hit=0.90`、`p_miss=0.48`、`p_min=0.10`、`p_max=0.98` 和 `signed_dist=true`。这些值会影响 UNKNOWN/FREE/OCCUPIED 状态、前沿及 ESDF，因此必须属于算法保真范围。
2. ROS 2 服务本身没有 ROS 1 回调布尔返回通道。此前传输成功会让 `ServiceClient::call()` 始终为真；现在 LKH 服务用未被算法读取的 `response.empty` 编码回调成功位，兼容层恢复 ROS 1 的真假返回，求解失败继续进入原始失败分支。
3. 类型 3 ACVRP 独立进程此前没有在服务端再次强制 `PRECISION=1`。现已复制原始参数文件修补顺序，然后调用本地原始 LKH CLI。
4. ROS 2 地图边界曾把障碍膨胀和 ESDF 一起推迟到 50 ms 定时器。现已恢复原版顺序：`inputPointCloud` 后在同一个点云回调内立即 `clearAndInflateLocalMap()`，定时器只运行 `updateESDF3d()`，避免 FSM/前沿定时器短暂看到未膨胀的新地图。
5. 原 ROS 1 `SDFMap::initMap()` 会在未初始化 `MapParam::use_swarm_tf_` 的情况下读取该字段；UBSan 在 Release 等效构建中确认了这一未定义行为。现显式设为 `false`，与原始 HGrid 的默认参数和统一世界坐标系语义一致，避免随机进入坐标变换分支。
6. 原 ROS 1 前沿水平分裂代码用 `auto` 保存 Eigen `.real()` 惰性表达式，导致其引用的临时矩阵离开作用域后仍被读取；ASan 在 `splitHorizontally()` 的 PCA 分支中复现了正式运行的崩溃。现只把表达式求值到固定尺寸值对象，计算和分裂结果不变。
7. 原 ROS 1 在 A* 起终点重合、路径简化为单点时仍调用多项式轨迹生成器，段数为 0 后写入零尺寸矩阵。现对该无运动目标返回原 FSM 已有的 `FAIL`，由下一周期重新选择/规划；所有非退化路径及轨迹优化数学保持不变。

## 保留的 Isaac 边界差异

- 地图原点、高度、box 和虚拟顶面取自仓库 USD 的可飞行器中心空间，而不是原论文仿真地图。
- 深度相机有效范围为 4.6 m；4.5 m raycast 截断让原始 `SDFMap` 把无返回深度射线标为空闲。该差异只定义传感器可见范围，不修改任务分配或路径代价。
- Isaac 发布同一物理时钟下的世界系点云和里程计，`MapROS` 仍调用原始点云融合函数。
- PhysX 飞行器和低层安全制动只执行/保护原始 `PositionCommand`；不会生成任务、改写前沿或改变 ACVRP 矩阵。
- 覆盖率和碰撞统计是只读验收输出。

## 自动核对

```bash
python3 tests/check_source_fidelity.py
python3 tests/check_runtime_fidelity.py
```

当前输出：

```text
C2 source fidelity check passed: 405 files, 400 byte-identical, 5 audited boundary/guard edits
C2 runtime fidelity check passed: 13 interfaces field-equivalent, 136 ROS1 algorithm parameters equal, 5 Isaac boundary parameters audited
```

## 最终 Isaac 回归

4 UAV、15 s、100 Hz PhysX、10 Hz 深度的短时闭环回归通过：四架 FSM 全部执行，604 帧点云，107 次连通图构建，84 次 HGrid 游历，8 次 ACVRP 分配，68 次轨迹规划；路径长度为 7.68/9.63/0.93/12.56 m。物理碰撞、LKH 失败和进程崩溃均为 0，最小机间距 1.85 m。

结果文件：`validation/fidelity_audit_4uav/warehouse_simple_result.json`。该回归用于验证完整算法链路，不以 15 秒内完成整张地图为条件；完整探索仍使用默认 900 秒及 `C2_REQUIRE_COMPLETION=1`。
