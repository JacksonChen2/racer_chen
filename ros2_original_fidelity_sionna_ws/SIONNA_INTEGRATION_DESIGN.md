# 原版一致 RACER 与 Sionna RT 通信边界设计

基线工作区：`../ros2_original_fidelity_ws`  
唯一算法基准：`../original_racer_ros1_ws/src/RACER/swarm_exploration`

## 1. 不允许改变的算法模块

下列模块继续由 `racer_original_core/upstream` 原样编译，通信扩展不修改其源码、
参数或回调内容：HGrid、前沿与视点选择、LKH ATSP/ACVRP、成对任务重分配、
Kinodynamic A*、NLopt B-spline、yaw 规划、轨迹服务器以及 FSM 的探索结束和返航逻辑。
现有 CBF、碰撞检测和紧急制动仍位于规划输出之后。

## 2. 原版与通信版的唯一运行时差异

| 边界 | 原版一致 ROS2 版 | Sionna RT 通信版 |
|---|---|---|
| 本机传感器与控制 | Isaac odom、深度点云、PositionCommand | 完全相同，不经过无线链路 |
| 多机消息发送 | DDS 共享 topic 直接交付 | 每架机发送到独立 TX topic |
| 无线传播 | 理想、无时延 | Sionna RT 根据 Warehouse 几何和无人机位姿计算每条有向链路 |
| 数据链路 | 无带宽、误包或队列约束 | SNR 驱动速率/PER，并施加序列化、时延、抖动、重传、TTL 和有限队列 |
| 多机消息接收 | DDS 共享 topic | 代理投递到每架机独立 RX topic，再进入未改的原版回调 |
| 算法消息内容 | 原版 ROS1 schema | 字节级保持原消息序列化内容，不加算法字段、不改时间戳 |

## 3. 经过通信层的六类原版消息

1. `DroneState`：协同状态广播；
2. `PairOpt`：成对任务重分配请求；
3. `PairOptResponse`：重分配响应；
4. `Bspline`：多机轨迹广播；
5. `ChunkStamps`：地图块清单；
6. `ChunkData`：地图块数据。

代理只认识 ROS 类型名、发送者编号和序列化字节数。它不反序列化、检查或更改
算法字段。原版消息自身的 `from_drone_id`、`to_drone_id`、stamp、重复发送、过期
过滤和地图缺块补发协议继续决定接收后的行为。

## 4. Sionna RT 与链路模型

- Warehouse USD 由 Isaac Python 展开后导出为 Z-up、米制 Mitsuba XML/PLY。
- 每架无人机同时是 2.4 GHz、20 MHz、垂直极化的各向同性收发节点。
- Sionna RT 计算 LoS、镜面反射和折射路径，输出 path gain、RSS、SNR 和 Doppler。
- `sionna_hybrid` 使用射线追踪缓存作为快速基线，并以在线精确求解修正；
  `sionna` 只接受在线精确/当前 Sionna 结果；`ideal` 仅用于通信代理同输入回归。
- PHY/MAC 映射参数都在 `config/warehouse_simple_communication.yaml` 中，可针对实际
  无线电标定；它们不是 RACER 算法参数。

## 5. 验证门槛

- 继承基线全部 source-fidelity 和同输入测试；
- 理想通信模式下，六类消息逐字节、逐顺序交付；
- 固定 seed 下，弱 SNR 必须产生更低交付率，队列和 TTL 必须可观测；
- Sionna RT 必须成功加载 Warehouse 场景，禁止静默退回距离模型；
- Isaac 联调必须同时观测原版 HGrid/LKH/Kinodynamic/NLopt/yaw/FSM 和
  `sionna_exact` 或 `sionna_cache_corrected` 链路；
- 完整验收仍要求 5/5 返航、零碰撞、最小机间距不小于 1 m、障碍净空为正。

