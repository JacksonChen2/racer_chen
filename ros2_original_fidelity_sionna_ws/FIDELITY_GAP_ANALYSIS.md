# RACER ROS1 → ROS2/Isaac Sim 原版一致性差异审计

审计日期：2026-08-09  
唯一算法基准：`../original_racer_ros1_ws/src/RACER`  
被审计的既有 ROS2/C++ 实现：`../ros2_3d_C_ws/src/racer_3d_cpp`  
新移植工作区：`ros2_original_fidelity_ws`

## 1. 审计结论

既有 `racer_3d_cpp` 不是原版 RACER 的等价 ROS2 移植，而是独立实现的轻量 3D 探索器。它在 HGrid、前沿与视点、任务分配、协同协议、路径搜索、轨迹优化、yaw 规划以及 FSM 完成条件上均改变了算法决策。该实现不能通过调参变成原版一致实现，也不能作为本次移植的算法源。

本次移植采用以下硬性规则：

1. 算法源只来自 `original_racer_ros1_ws/src/RACER`，不从 Python 版、既有 ROS2 版或通信增强版反向移植算法。
2. 原版核心公式、状态机分支、阈值语义、采样顺序、整数缩放、LKH 文件格式、NLopt 代价与梯度、消息时序判定均保留。
3. 允许改变的内容仅限 ROS2 通信、ROS2 仿真时间、Isaac 传感器输入、`map`/机体系坐标变换以及 Crazyflie 控制输出。
4. 原版的 B-spline `SWARM` 代价和收到冲突轨迹后触发重规划属于原版规划决策，必须保留。
5. 新增 CBF、机体碰撞检测、预测碰撞检测和紧急制动位于轨迹执行器之后；它们不改变 HGrid、前沿/视点、LKH 输入、Kinodynamic A*、NLopt 目标函数、yaw 轨迹或 FSM 任务判定。
6. 未通过同输入差分测试的模块不得进入 Isaac 验收；未通过零碰撞、最小安全距离或正常返航的 Isaac 运行不得宣称完成。

## 2. 逐模块差异与移植约束

### 2.1 地图与 HGrid

原版来源：

- `active_perception/src/hgrid.cpp`
- `active_perception/src/uniform_grid.cpp`
- `active_perception/include/active_perception/{hgrid,uniform_grid}.h`
- `plan_env/src/{sdf_map,edt_environment,raycast,multi_map_manager}.cpp`

原版行为：

- 两级 `UniformGrid`；网格只在 XY 平面细分，每个网格覆盖完整 Z 范围。
- 一级网格分裂为四个二级 XY 子网格，而不是八个 XYZ 子网格。
- 一级初始激活；二级初始关闭。一级网格在 `free > min_free` 时分裂。
- 相关性是 `unknown_num >= min_unknown` 或包含前沿。
- 保留前一次路径的一级/二级网格一致性，通过 `first_ids`、`second_ids`、`consistent_cost` 和 `consistent_cost2` 进入代价矩阵。
- 近邻网格使用原版 A* 求路径距离，远网格使用 `1.5 * 欧氏距离 + consistent_cost2`。
- 初始化时仅 1 号机获得全部相关一级网格，后续依靠成对优化去中心化分配。

既有 ROS2/C++ 差异：

- `HierarchicalGrid3D` 在 XYZ 三轴八分细化，使用已知比例触发分裂。
- 网格 ID 是字符串 `level:x:y:z`；原版是两层连续整数地址。
- 初始所有权按距离和负载贪心分散到所有无人机。
- 路线为最近邻加 2-opt；没有原版一致性网格、代价惩罚和 LKH 矩阵语义。

移植约束：直接移植原版 `HGrid` 和 `UniformGrid` 算法。Isaac 点云只允许通过地图输入适配器写入原版 `SDFMap`；不得把现有 `HierarchicalGrid3D` 放入规划链。

同输入测试：固定占据栅格、更新框、前沿均值、无人机状态和旧网格序列，逐项比较激活 ID、未知体素数、中心、分裂结果、`first_ids`/`second_ids`、完整代价矩阵和网格 tour。

### 2.2 前沿检测、聚类、视点采样与局部视点选择

原版来源：

- `active_perception/src/frontier_finder.cpp`
- `active_perception/src/perception_utils.cpp`
- `active_perception/src/graph_node.cpp`
- `exploration_manager/src/fast_exploration_manager.cpp`

原版行为：

- 在原版 SDFMap 更新区域内搜索前沿，使用原邻域扩展、聚类、水平分裂、降采样、包围盒和 dormant frontier 维护逻辑。
- 使用 `candidate_rmin/rmax/rnum/dphi` 环形采样、最小距离、EDT 净空、相机 FOV 和逐体素光线投射计算可见数。
- 按可见数和衰减选取 top viewpoints；全局成本由 `ViewNode::computeCost` 组合路径、速度与 yaw 动力学成本。
- 局部细化构建分层视点图，使用原版 Dijkstra 选择跨多个前沿的视点序列。

既有 ROS2/C++ 差异：

- 前沿由 `VoxelMap::frontierClusters()` 直接聚类，没有原版更新/删除/dormant/合并状态。
- 候选固定为 2 个半径 × 3 个仰角 × 8 个方位角，并只保留自定义评分前两名。
- 使用 `gain + cluster_size + owner_bonus - path_distance` 自定义分数；没有原版 ViewNode 成本矩阵和分层视点图 Dijkstra。

移植约束：保留原版 `FrontierFinder`、`PerceptionUtils`、`GraphNode/ViewNode` 和 `FastExplorationManager::refineLocalTour`。相机内外参由 Isaac 适配为原版参数，不改变可见性和评分算法。

同输入测试：固定 SDFMap 和相机位姿，比较活动/休眠前沿体素集合、平均点、包围盒、候选位置/yaw/可见数、top viewpoints、成本矩阵和局部细化序列。

### 2.3 LKH 全局 tour 与 ACVRP 成对任务分配

原版来源：

- `exploration_manager/src/fast_exploration_manager.cpp`
- `utils/lkh_tsp_solver`
- `utils/lkh_mtsp_solver`

原版行为：

- 前沿和单机网格 tour 写为显式全矩阵 ATSP，成本乘 100 后截断为整数。
- 成对重分配固定使用 ACVRP；矩阵包含虚拟 depot、两机节点和活动网格节点。
- demand 为 `unknown_num * 0.1`；capacity 为总未知量 `* 0.75 * 0.1`；`RUNS=1`，ACVRP `SEED=0`。
- 使用原仓库内置 LKH/LKH-3 源码并按原 tour 文件解析规则还原整数网格 ID。

既有 ROS2/C++ 差异：

- 小于等于 10 个网格时枚举二分子集并求开放路径；更大集合使用负载贪心回退。
- 没有 LKH-3、ATSP/ACVRP 文件、虚拟 depot、原 demand/capacity 和原 tour 解析。

移植约束：编译原仓库自带的 LKH 与 LKH-3 源码。ROS2 可把服务调用改为进程内/ROS2 服务适配，但求解器输入文件内容、随机种子、运行次数和解析逻辑不得改变。每个 agent 使用独立工作目录，避免 LKH 全局变量和文件互相污染。

同输入测试：保存原版与移植版生成的 `.atsp/.par`，要求规范化后逐字节一致；固定 seed 比较 `.tour`、网格分配和 tour 顺序。

### 2.4 去中心化协同协议

原版来源：

- `exploration_manager/src/fast_exploration_fsm.cpp`
- `exploration_manager/msg/{DroneState,PairOpt,PairOptResponse}.msg`
- `plan_env/src/multi_map_manager.cpp`
- `plan_env/msg/{ChunkData,ChunkStamps,IdxList}.msg`

原版行为：

- 25 Hz 广播本机状态，20 Hz 检查成对优化，轨迹 10 Hz 重发。
- 仅选择 ID 更大的、状态小于 0.2 秒、近期无尝试/交互且网格并集非空的无人机。
- 将所有活动但未分配网格加入本次联合 ACVRP。
- 请求和响应包含严格时间戳去重；接收端可能因频繁尝试返回拒绝；发送端等待匹配时间戳确认后才提交本地结果。
- `repeat_send_num` 次重复发送用于不稳定通信；地图按 chunk/stamp 增量同步。

既有 ROS2/C++ 差异：

- JSON 字符串传输，5 Hz 状态和约 3 秒轮转配对。
- 发送后立即修改所有权；没有请求确认、拒绝、重复发送、乱序时间戳防护、最近交互策略和未分配网格补入。
- 地图是整图压缩广播，不是原版 chunk/stamp 协议。

移植约束：生成等价 ROS2 typed messages，保持字段、频率、时间戳判断、重发和状态提交时机。QoS/命名空间可以为 ROS2 调整；丢包模型放在通信适配层，不改协议状态机。

同输入测试：使用确定性虚拟时钟注入正常、重复、乱序、超时和拒绝消息，比较候选机选择、请求内容、响应状态、提交/回滚时刻、`recent_*` 字段和最终 dominance grid。

### 2.5 Kinodynamic A*

原版来源：

- `path_searching/src/kinodynamic_astar.cpp`
- `path_searching/include/path_searching/kinodynamic_astar.h`
- `plan_manage/src/planner_manager.cpp`

原版行为：

- 搜索状态为三维位置加三维速度；控制输入为三维加速度，边持续时间离散采样。
- 保留速度/加速度约束、horizon、时间维索引、动态环境模式、shot trajectory、多项式根求解、原启发式、tie breaker、节点池和两阶段重试。
- 中距离目标走 Kinodynamic A*；近距离/初始化走原 waypoint 优化；远距离按原 7 m 规则截断几何路径。

既有 ROS2/C++ 差异：

- 只有 26 邻域几何栅格 A*，状态不含速度/时间/加速度，没有 shot trajectory 或动力学启发式。

移植约束：直接保留原 `KinodynamicAstar`、采样与 `FastPlannerManager::kinodynamicReplan`；只把参数读取、日志和时间接口换成 ROS2 适配。

同输入测试：固定地图、起终位置/速度/加速度和参数，比较返回状态、扩展节点序列、控制输入/持续时间、shot 成功标志、采样点集、边界导数和最终 kino 轨迹（浮点容差 `1e-9`，NLopt 前输入必须相同）。

### 2.6 NLopt 非均匀 B-spline

原版来源：

- `bspline/src/non_uniform_bspline.cpp`
- `bspline_opt/src/bspline_optimizer.cpp`
- `plan_manage/src/planner_manager.cpp`

原版行为：

- 原版非均匀 B-spline 参数化、De Boor 求值、导数、可行性检查和时间重分配。
- NLopt 目标保留 SMOOTHNESS、DISTANCE、FEASIBILITY、START、END、GUIDE、WAYPOINTS、VIEWCONS、MINTIME 和 SWARM 全部原公式与梯度。
- 优化器算法编号、最大迭代/时间、边界、`xtol_rel=1e-5`、两阶段重试和起终边界状态保持不变。

既有 ROS2/C++ 差异：

- 自定义固定次数“平滑梯度 + ESDF 推离”，没有 NLopt、原代价权重/梯度、导数边界、可行性和 swarm trajectory 代价。
- 时间只按路径长度构造梯形近似，失败时退回折线路径。

移植约束：工作区隔离构建 NLopt 2.7.1，不覆盖系统库；直接编译原版优化器和非均匀 B-spline。禁止保留现有迭代推点或折线回退作为规划替代。

同输入测试：比较参数化控制点/knots/导数；对每个 cost bit 比较 cost 与解析梯度；固定 NLopt 版本、算法和初值后比较最优控制点、knot span、迭代数与最终 cost。

### 2.7 yaw 规划

原版来源：

- `plan_manage/src/planner_manager.cpp` 的 `planYawExplore`
- `active_perception/src/heading_planner.cpp`

原版行为：

- 位置轨迹时长固定切为 12 个 yaw 段。
- 使用三阶 B-spline 起始 yaw/yawdot/yawddot 边界；沿位置轨迹前视 2 秒生成 yaw waypoint。
- 用 `relax_time1/2` 和 `max_yawdot` 筛选 waypoint，进行角度连续展开，终点 yaw 零导数。
- 通过同一个 NLopt B-spline 优化器求 SMOOTHNESS|START|END|WAYPOINTS。
- 保留 HeadingPlanner 的离散 yaw 图、FOV 信息增益和 Dijkstra 实现，即使默认 RACER 主链主要调用 `planYawExplore`。

既有 ROS2/C++ 差异：

- 只对当前视点方向调用 `atan2`，控制消息直接携带单一 yaw；没有 yaw B-spline、边界导数、前视 waypoint、角度连续性或 yawdot 限制。

移植约束：直接保留两套原版 yaw 代码；Isaac 控制适配器仅在执行时采样原版 yaw/yawdot，不重新计算朝向。

同输入测试：固定位置 B-spline、起终 yaw 和参数，比较 waypoint/索引、展开后的角度、yaw 控制点、knots、yaw/yawdot/yawddot 采样。

### 2.8 FSM 完成、空闲与返航

原版来源：

- `exploration_manager/src/fast_exploration_fsm.cpp`
- `exploration_manager/include/exploration_manager/{fast_exploration_fsm,expl_data}.h`

原版行为：

- 状态为 INIT → WAIT_TRIGGER → PLAN_TRAJ → PUB_TRAJ → EXEC_TRAJ，并包含 IDLE 与 FINISH。
- 里程计就绪后等待 2 秒，再等待显式触发。
- 执行中按“当前前沿已覆盖、轨迹快结束、周期阈值”触发重规划。
- 无前沿/无 dominance grid 进入 IDLE，不按覆盖率直接完成。
- IDLE 100 秒后将 `next_pos` 设为任务触发时记录的起点、`next_yaw=0`，走原轨迹规划返航。
- 返航轨迹接近目标 1 m 后进入 FINISH。

既有 ROS2/C++ 差异：

- 没有原状态机；传感器就绪后自动规划。
- 达到 `completion_coverage`（默认 0.90）或收到字符串即停止；没有 100 秒 IDLE 复查/返航和原版 FINISH 判定。

移植约束：保留原状态、分支、阈值和返航逻辑。ROS2 timer 必须绑定 Isaac `/clock`；显式触发可由启动适配器发布，但不能绕过 WAIT_TRIGGER。

同输入测试：确定性推进虚拟时钟，覆盖 odom 未就绪、触发、规划失败、NO_GRID、前沿消失、IDLE 100 秒、返航重规划和到家等事件，要求逐步状态与动作一致。

### 2.9 独立 Isaac 安全层

原版没有 CBF。新安全层位于“原版 B-spline/yaw 轨迹采样输出”与“Crazyflie 控制接口”之间：

- 输入：原版期望位置/速度/yaw、Isaac 实际状态、静态碰撞几何/安全激光、其他无人机实际状态。
- 输出：受限控制参考或急停/悬停命令。
- 不允许写回原版地图、网格归属、前沿、LKH cost、Kinodynamic A*、NLopt cost、yaw 轨迹或 FSM 任务状态。
- 安全层触发会独立记录原因、持续时间、原命令和受限命令。原版自身的轨迹碰撞检查和 swarm 重规划仍按基准运行。

隔离测试：在相同算法输入下启用/禁用安全层，规划层的所有序列化输出必须逐字节一致；只允许执行命令和安全遥测不同。

## 3. 必须保留的依赖

- Eigen 3（现有系统 3.4.0）
- PCL（现有系统 1.12.1）
- Armadillo（现有系统 10.8.2）
- 原仓库内置 LKH 与 LKH-3 源码
- NLopt 2.7.1（当前系统缺失；隔离安装到本工作区）
- ROS2 Humble 的 `rclcpp`、标准消息、TF2、rosidl 和 `pcl_conversions`
- Isaac Sim 5.1（使用 `/home/jiazheng/software/isaacsim`，不修改其安装内容）

## 4. 新工作区设计

计划包结构：

```text
ros2_original_fidelity_ws/
  FIDELITY_GAP_ANALYSIS.md
  BASELINE_SHA256SUMS.txt
  third_party/
    nlopt-2.7.1/            # 源码
    install/                # 隔离安装前缀
  src/
    racer_fidelity_msgs/    # 原 msg/srv 的 ROS2 等价定义
    racer_original_core/    # 从唯一基准复制并受指纹约束的算法源码
    racer_ros2_adapters/    # 参数、时间、通信、TF、传感器适配
    racer_isaac_bridge/     # Isaac/Crazyflie 控制与独立安全层
    racer_fidelity_tests/   # 同输入差分与集成测试
```

`racer_original_core` 中每个算法文件都记录其基准 SHA256。为了审计允许的修改，移植提交会维护 `PORTING_CHANGES.yaml`，每处差异必须分类为 `communication`、`time`、`sensor`、`frame`、`control` 或 `build`；没有分类的算法差异视为失败。

## 5. 验收门槛

### 5.1 同输入一致性

- 离散输出（ID、状态、tour、消息接受/拒绝、分支结果）：完全一致。
- 确定性浮点核心：绝对误差和相对误差均不超过 `1e-9`。
- NLopt/LKH：固定版本、seed、算法、输入文件和初值；tour 必须一致，控制点允许 `1e-7` 浮点误差。
- ROS2/Isaac 边界不参与算法 golden 结果。

### 5.2 Isaac Warehouse 多机验收

- 场景：Isaac Warehouse（优先 `warehouse_simple`，随后对可用的 warehouse loaded 场景复验）。
- 机型：`crazyflie_with_racer_dynamics`。
- 多机：5 架。
- 完成：所有 agent 按原版 FSM 进入 FINISH，且确实执行 IDLE 后返航到各自记录起点。
- 碰撞：Isaac contact report 为 0 次机体-环境碰撞、0 次机间碰撞。
- 最小安全距离：全程实际机间距离不小于配置门槛；报告同时给出观测最小值、发生时间和机对。
- 算法真实性：日志必须包含 HGrid 更新、LKH ATSP/ACVRP 求解、Kinodynamic A*、NLopt position/yaw 优化和成对协议确认的运行证据。
- 未通过任一项时继续定位和修复，不用提高完成覆盖率、跳过返航、降低安全距离或替换简化规划器来规避失败。

## 6. 实施顺序

本报告完成后按以下顺序实施：消息/构建基础 → 原版地图与 HGrid → 前沿/视点 → LKH → 协同协议 → Kinodynamic A* → B-spline/NLopt → yaw → FSM → Isaac 传感器/控制适配 → 独立安全层 → 差分测试 → Isaac 验收。
