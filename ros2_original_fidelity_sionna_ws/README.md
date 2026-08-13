# ROS1 原版一致的 RACER ROS2/C++ + Isaac Sim + Sionna RT 工作区

该目录是独立的通信环境仿真工作区，唯一算法基准为
`../original_racer_ros1_ws/src/RACER/swarm_exploration`。旧的
`ros2_3d_C_ws`、Python 版和通信环境版均不作为算法来源。

Sionna RT 只作为 RACER 机间 ROS2 消息之外的独立通信链路层：它根据
Isaac Sim 中无人机真值位姿和 Warehouse 几何计算链路，通信代理再施加传播时延、
抖动、带宽、分包、丢包、重传、TTL 和有限队列。HGrid、前沿与视点选择、LKH、
协同协议、Kinodynamic A*、NLopt B-spline、yaw 和 FSM 不读取链路真值，也没有
为适应通信环境而采用简化规划算法。详细边界和消息映射见
`SIONNA_INTEGRATION_DESIGN.md`，实测数据见 `SIONNA_VALIDATION_REPORT.md`。

## 首次安装与运行

Sionna RT 安装到本目录的 `.sionna_runtime`，不覆盖系统 Python 或 Isaac Sim：

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/ros2_original_fidelity_sionna_ws
./setup_sionna_env.sh
./prepare_warehouse_simple_sionna_scene.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
```

默认启动 5 架 `crazyflie_with_racer_dynamics`，使用在线 Sionna RT 路径求解：

```bash
./run_warehouse_simple_sionna.sh
```

带 Isaac Sim 可视化并采用限制绘制频率/地图点数的流畅交互配置：

```bash
./run_warehouse_simple_sionna_visualization.sh
```

运行参数可通过环境变量覆盖，例如：

```bash
RACER_FIDELITY_DRONE_COUNT=5 \
RACER_FIDELITY_DURATION=900 \
RACER_MAPPING_COVERAGE_TARGET=0.95 \
RACER_COMMUNICATION_MODE=sionna \
./run_warehouse_simple_sionna.sh
```

## 含工业 AP 的 1800 秒配对实验

`warehouse_simple_with_industrial_ap.usda` 在天花板中央加入可见工业 Wi-Fi AP；
对应的 Sionna 场景也包含 AP 金属外壳及固定相位中心。以下入口依次运行两组：

1. `distributed`：保留同一个含 AP 实体的 RF 场景，但只允许 UAV↔UAV；
2. `ap_assisted`：保留 UAV↔UAV，并增加 UAV↔AP 上行/下行。AP 接收全局消息流，
   只向尚未由直链收到该消息的 UAV 转发，代理在交付 RACER 前去重。

两组均使用 Sionna RT、5 架 `crazyflie_with_racer_dynamics`、相同随机种子和原始
RACER 参数；每组在原始任务完成或 1800 秒仿真时间时停止：

```bash
./run_ap_mapping_ab_experiment.sh
```

输出目录 `experiments/warehouse_simple_ap_ab_<时间>/` 中包含两组完整日志、最终
JSON、每秒覆盖率历史、0.5 秒采样轨迹、`coverage_curve.csv`、`comparison.json`
和 `comparison.md`。原 UAV 直链使用逐链路独立的确定性随机流，新增 AP 链路不会
仅因消耗额外随机数而改变对照组的直链丢包序列。

`sionna` 是默认且严格的在线射线追踪模式；初始化或 PathSolver 失败时测试失败，
不会退化到解析距离模型。可选的 `sionna_hybrid` 模式使用预生成射频缓存以提高
长时可视化帧率：

```bash
./generate_warehouse_simple_radio_cache.sh
RACER_COMMUNICATION_MODE=sionna_hybrid ./run_warehouse_simple_sionna_visualization.sh
```

逐模块审计和移植约束见 `FIDELITY_GAP_ANALYSIS.md`；每一处允许的移植边界
改动见 `PORTING_CHANGES.yaml`。`racer_original_core/upstream` 当前包含完整的
原版 HGrid、前沿/视点、LKH ATSP/ACVRP、协同协议、Kinodynamic A*、NLopt
B-spline、yaw 和 FSM 源码。完整性测试会检查全部 467 个上游文件，只允许两处
ROS2 消息 API 转换以及两处有明确复现证据的未定义行为修复；测试会把这些补丁
规范化后再与 ROS1 基准逐字节比较，其他差异一律失败。

## 依赖隔离

- ROS2 Humble、Eigen、PCL、Armadillo 使用现有系统安装。
- NLopt 2.7.1 安装在本目录的 `third_party/install`，没有覆盖系统库。
- LKH/LKH-3 直接编译原仓库内置源码。
- Isaac Sim 使用 `/home/jiazheng/software/isaacsim`，没有修改其安装内容。

## 构建和对照测试

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/ros2_original_fidelity_sionna_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DBUILD_TESTING=ON
colcon test --packages-select racer_fidelity_tests --event-handlers console_direct+
colcon test-result --verbose
```

`racer_source_fidelity` 检查整个算法源码树；
`racer_bspline_same_input` 分别从 ROS1 基准目录和移植目录编译原版
`non_uniform_bspline.cpp`，输入同一组控制约束，并要求输出逐字节一致。
`racer_lkh_same_input` 同样分别编译两棵源码树的原版 LKH，对完全相同的
ATSP 矩阵和 seed 比较代价与完整 tour。
`racer_safety_layer_isolation` 验证 CBF 会约束危险执行命令、不会修改输入，且
safety lidar 没有通往规划点云的反馈路径；其中还重放窄间隙中两侧净空约束互相
冲突的输入，要求最近障碍具有最终制动优先级。
`racer_wire_schema_fidelity` 检查 13 个消息和 2 个服务的字段、类型、顺序与常量，
只允许 ROS2 的 `builtin_interfaces/Time` 和 rosidl 合法字段名转换。
`racer_callback_serialization` 检查原版 `ros::spin()` 的单线程回调队列语义没有被
ROS2 多线程 executor 改变，并确认阻塞式 LKH 服务使用独立的纯通信节点。
`racer_degenerate_hover_regression` 重放“下一视点与当前位置重合”的故障输入，
验证仍由原版五次多项式生成有限的双段 hover 轨迹，避免 0 段矩阵越界。
`racer_trigger_readiness` 保证启动消息仅在 5 机里程计、每机 25 帧规划点云和
全部 FSM 订阅均就绪后发布，避免低传感器频率诊断时个别原版 FSM 过早结束。
`racer_isaac_graph_readiness` 保证 Isaac 仅在每架机的里程计和规划点云都发现原版
规划节点与就绪触发器两个订阅者后才重置初态并开始实验计时；DDS 发现超时会明确
失败，不能产生“通信未连接但计时完成”的伪结果。
`racer_completion_monitor` 验证前 4 架 FINISH 不会关闭仿真，只有第 5 架原版 FSM
也记录 FINISH 后才发送结束通知。

## 无通信损伤的 Warehouse Simple 基准回归

以下入口保留用于与原工作区进行理想通信基准回归；包含 Sionna RT 的测试应使用
本文开头的 `run_warehouse_simple_sionna*.sh`。默认组合为 5 架
`crazyflie_with_racer_dynamics.usd`、900 秒、`warehouse_simple.usd`：

```bash
./src/racer_isaac_adapter/scripts/run_warehouse_simple.sh
```

带 Isaac 可视化：

```bash
./src/racer_isaac_adapter/scripts/run_warehouse_simple_visualization.sh
```

可视化模式将 viewport 刷新固定在独立的 wall-time 频率，并限制累计地图绘制点，
避免物理 1000 Hz 循环和五路深度相机每步强制重绘造成鼠标视角严重卡顿。可按需设置：

```bash
RACER_INTERACTIVE_RENDER_HZ=30 \
RACER_VISUALIZATION_MAX_MAP_POINTS=12000 \
./src/racer_isaac_adapter/scripts/run_warehouse_simple_visualization.sh
```

测试结果写入 `validation/warehouse_simple_result.json`，ROS2 与 Isaac 的完整日志
分别写入 `validation/warehouse_simple_launch.log` 和
`validation/warehouse_simple_isaac.log`。正式验收必须同时满足五机正常进入原版
FINISH、零接触碰撞、实际机间最小距离不小于 1.0 m、障碍物净空为正。
完成监视器只增量读取日志；它在五个原版 FSM 均已经记录 FINISH 后通知 Isaac
关闭，不参与规划，也不会因长时日志增长而反复扫描整个文件。

按建图覆盖率或时限停止的独立测试可使用：

```bash
RACER_MAPPING_COVERAGE_TARGET=0.95 \
RACER_STOP_ON_COMPLETION=0 \
RACER_FIDELITY_DURATION=900 \
./src/racer_isaac_adapter/scripts/run_warehouse_simple.sh
```

覆盖率是每架机原始 SDFMap 规划盒内非 UNKNOWN 体素数除以总规划体素数；停止值取
五个经原版 chunk 协议融合的地图中最高值，并在结果中同时保留每架机的值。统计器
只读占据缓存并发布私有诊断话题，不写地图、不向规划器提供输入。

默认正式验收保持原仿真器的 1000 Hz dynamics、200 Hz odometry、640×480/30 Hz
深度相机。定位长时任务时可临时使用 `RACER_PHYSICS_RATE_HZ`、
`RACER_SENSOR_RATE_HZ`、`RACER_DEPTH_WIDTH` 和 `RACER_DEPTH_HEIGHT` 加速；这几项
只改变 Isaac 数值积分和传感器接口，正式结果仍应回到默认值复验。

## 安全层隔离

原版规划只接收前向深度相机点云。360° safety lidar、CBF、停止距离约束、
flight-volume barrier、针对细货架/顶灯的 PhysX 机体扫掠、contact report 和紧急分离
只作用于 Isaac 执行命令，
不会写回 SDFMap、前沿、HGrid、LKH、Kinodynamic A*、NLopt、yaw 或 FSM。
扫掠采用包含可视旋翼的 0.332 m 包络；窄间隙约束按远到近投影，保留最近表面的
最终权威，避免较远的对向表面把命令重新推回接触面。
