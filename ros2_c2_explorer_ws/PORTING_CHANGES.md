# ROS 2 / Isaac 移植差异清单

本文列出与 `C2-Explorer` commit `fd1c76a49a4453f91da4984e123b56932c5382f3` 的有意差异。未列出的 `upstream` 文件必须逐字节一致。

## 原始算法源码中的五处审计差异

1. `plan_env/src/multi_map_manager.cpp`
   - ROS 1 消息字段 `voxel_occ_` 在 ROS 2 IDL 中不允许以下划线结尾，因此线上字段改名为 `voxel_occ`。
   - 内部 `ChunkData` 数据结构仍为 `voxel_occ_`；发送、校验和接收赋值语义不变。

2. `exploration_manager/src/c2_exploration_fsm.cpp`
   - ROS 1 `ros::Time` 消息字段可直接调用 `toSec()`；ROS 2 `builtin_interfaces/Time` 先由兼容构造函数包装为 `ros::Time`。
   - 比较阈值和轨迹时间值不变。

3. `exploration_manager/src/c2_exploration_manager.cpp`
   - 增加瞬时空 refined-viewpoint 列表保护。Isaac 点云可能恰好在代价矩阵构造与细化之间使单个前沿失效；原代码会索引空向量。
   - 正常候选非空时执行路径完全不变；空候选时使用该前沿原本的 top viewpoint，保持原规划意图并等待下一轮重规划。
   - 增加退化单点路径保护。A* 起终点重合且 `shortenPath()` 折叠为一个点时，原多项式生成器会写入零尺寸矩阵；现在进入 FSM 已有的 `FAIL` 重规划分支。含至少一个轨迹段的正常路径不变。

4. `plan_env/src/sdf_map.cpp`
   - 原 ROS 1 源码在 `MapParam` 分配后从未给 `use_swarm_tf_` 赋值，却在地图初始化中直接读取它；优化构建会因此触发未定义行为。
   - 显式初始化为 `false`，与原始 `partitioning/use_swarm_tf=false` 默认配置及当前统一世界坐标系一致。该防护只把预期默认分支确定化，不改变正常 C² 决策逻辑。

5. `active_perception/src/frontier_finder.cpp`
   - 原源码用 `auto` 保存 `es.eigenvalues().real()` 和 `es.eigenvectors().real()` 的 Eigen 惰性表达式，表达式引用的临时复数矩阵在下一语句前已销毁；ASan 在正式场景的前沿水平分裂路径中确认了 `stack-use-after-scope`。
   - 改为同尺寸 `Eigen::Vector2d`/`Eigen::Matrix2d` 值对象，只延长完全相同数值的生命周期；PCA、最大特征向量选择和前沿分裂逻辑不变。

## 不在 upstream 内的移植边界

- `include/ros/ros.h` 与 `port/ros_compat.cpp`：把源码使用的 ROS 1 Publisher、Subscriber、Timer、ServiceClient、Time 和日志 API 映射到 `rclcpp`。算法节点仍用单线程 executor。
- `port/map_ros_port.cpp`：用 Isaac 同一物理时域的世界系点云和里程计替代 ROS 1 message_filters 入口；后续融合调用原始 `SDFMap::inputPointCloud`。与 ROS 1 相同，占据栅格膨胀在点云回调内立即完成，ESDF 才延迟到 50 ms 定时器。
- `port/lkh_server_port.cpp`：提供 `/solve_tsp_N` 与 `/solve_acvrp_N` ROS 2 服务，后端仍为原始 LKH-3。类型 3 求解保持独立进程并在执行前强制 `PRECISION=1`；ROS 2 响应中的 `empty` 只承载 ROS 1 回调成功位，兼容层据此恢复同步 `call()` 的真假语义。
- `c2_explorer_msgs`：ROS 1 自定义消息/服务的 ROS 2 等价定义。时间使用 `builtin_interfaces/Time`，几何类型使用 ROS 2 `geometry_msgs`。
- `c2_explorer_isaac`：Isaac 深度/里程计、统一启动触发、`PositionCommand` 到 PhysX 控制的适配及只读验收指标。
- launch 中把每架 UAV 的传感器、命令和可视化话题隔离到 `/drone_i/*`，把 C² 状态、会合、轨迹、心跳和地图块消息映射到共享 `/c2_explorer/*` 话题。

## 仿真参数差异

- 地图范围取自仓库 USD 的可飞行空间；`ground_height=0.0`、规划 box 和 `virtual_ceil_height=8.6` 表示 Isaac 中飞行器中心的有效飞行体积。
- Isaac 深度最大有效范围为 4.6 m，`sdf_map.max_ray_length=4.5` 用原始 `inputPointCloud` 的“超量程端点为空闲”分支编码无返回射线；这是传感器边界适配。ROS 1 仿真使用 10 m 传感器，因此原值为 10 m。
- `map_ros.visualization_truncate_height` 仅跟随仓库高度，且不进入地图或规划决策。
- C² 占据概率参数已经逐项恢复为原版：`p_hit=0.90`、`p_miss=0.48`、`p_min=0.10`、`p_max=0.98`、`p_occ=0.80`、`signed_dist=true`。连通图、任务分区、会合协议、前沿、搜索和轨迹参数同样保持原始配置。
- `warehouse_loaded*` 只扩展 SDF 存储包络以覆盖偏置仓库坐标，不改变任务算法。
- 正式配置使用 1000 Hz PhysX、200 Hz odometry、30 Hz 640×480 深度；可通过环境变量降频做冒烟测试。
- `map_ros.coverage_diagnostic_period` 与 Isaac 验收统计只读，不反馈给规划器。

## 完成后的逻辑复审修正

2026-08-26 的源码/论文/运行时复审发现并修正会改变分支或时序的移植差异：占据概率与 signed ESDF 配置、LKH 服务失败返回语义、点云回调内障碍膨胀的执行顺序，以及原源码未初始化的 `use_swarm_tf_`。修正后 `tests/check_runtime_fidelity.py` 核对 13 个接口和 136 个 ROS 1 算法参数；完整结论见 `LOGIC_AUDIT.md`。
