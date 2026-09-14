# ROS1 原版一致 RACER ROS2/Isaac 验证报告

更新日期：2026-08-09  
唯一算法基准：`../original_racer_ros1_ws/src/RACER/swarm_exploration`  
验证工作区：`ros2_original_fidelity_ws`

> 状态：算法移植、同输入回归和 Isaac Warehouse Simple 五机端到端验收均已通过。
> v16 未使用覆盖率阈值提前停止，由原版 FSM 在 677.69 s 自主完成探索和五机返航；
> 最终联合已知体素覆盖率 99.4784%，物理接触为 0，所有返航位置误差均小于原版
> 1.0 m 完成阈值。

## 1. 移植完整性

- `BASELINE_SHA256SUMS.txt` 固化 ROS1 基准的 467 个上游文件。
- `racer_original_core/upstream` 编译原版 HGrid、前沿/视点、LKH ATSP/ACVRP、
  成对协同与地图 chunk 协议、Kinodynamic A*、NLopt B-spline、yaw、FSM 和轨迹服务器。
- `racer_source_fidelity` 对 467 个文件逐一检查。仅允许并精确规范化：2 处 ROS2 API
  转换和 2 处有运行复现证据的未定义行为修复；其余源码差异会令测试失败。
- CBF、360° safety lidar、PhysX sweep、flight-volume barrier、contact report 位于
  规划输出之后；其点云不进入 SDFMap，也不改变规划决策。

逐模块的原版行为、旧 ROS2 简化版差异、移植约束和应比较输出见
`FIDELITY_GAP_ANALYSIS.md`；允许的所有修改见 `PORTING_CHANGES.yaml`。

## 2. 自动回归

当前结果：10/10 通过，0 error，0 failure，0 skipped。

| 测试 | 验证内容 | 结果 |
|---|---|---|
| `racer_source_fidelity` | 467 文件、2 API 改动、2 UB 修复白名单 | PASS |
| `racer_bspline_same_input` | ROS1/ROS2 B-spline 输出逐字节一致 | PASS |
| `racer_lkh_same_input` | 固定输入/seed，代价 173、tour 完全一致 | PASS |
| `racer_safety_layer_isolation` | CBF 有效、窄间隙最近约束优先、无规划反馈 | PASS |
| `racer_wire_schema_fidelity` | 13 msg + 2 srv 字段/类型/顺序一致 | PASS |
| `racer_callback_serialization` | 原版单线程回调与阻塞服务语义 | PASS |
| `racer_degenerate_hover_regression` | 重合视点生成有限双段原版多项式 | PASS |
| `racer_trigger_readiness` | 5 机 odom、25 帧点云和 FSM 订阅屏障 | PASS |
| `racer_isaac_graph_readiness` | Isaac 在 DDS 全连接前不开始实验计时 | PASS |
| `racer_completion_monitor` | 仅在收齐 5 个原版 FINISH 后关闭 | PASS |

## 3. 长时故障定位与修复

### 3.1 约 90 秒规划器段错误

GDB 同时捕获两个原版探索节点在
`PolynomialTraj::waypointsTraj()` 的 `Ct(0,0)=1` 越界。调用链为
`planExploreMotion → planTrajToView → FastPlannerManager::planExploreTraj →
PolynomialTraj::waypointsTraj`。

输入根因：A* 回溯产生同一 start voxel 内的两个点；原版 `shortenPath` 将距离小于
1 mm 的起终点压成一个点，随后构造 0 段多项式矩阵。修复只覆盖该原版未定义输入：
把已选定的同位置/同 yaw hover 目标表达为三个重合点和两个正时长段，再走原版五次
多项式、NLopt B-spline 与 yaw 流水线。所有非退化输入仍执行原分支。

UBSan 还发现 `SDFMap::MapParam::use_swarm_tf_` 未初始化；将其初始化为 `false`，与
原本已设为单位阵的 map transform 一致。两项修复均受 source-fidelity 白名单约束。

### 3.2 DDS 启动丢失

一次 300 秒轮次在 Isaac 启动时看到所有 sensor publisher 的订阅数为 0，导致实验
时间推进但触发器没有点云。现采用两级屏障：Isaac 等待每架 odom/point cloud 的
必要订阅者；触发器再等待每架 25 帧点云和 5 个 FSM start 订阅。超时明确失败。

### 3.3 窄间隙接触

100 Hz、300 秒轮次在约 200–217 秒由第 4 架机产生 6 次环境接触。根因是两侧保守
净空在窄间隙中不可同时满足时，原安全投影按近到远执行，较远表面会在最后把命令
推回最近接触面。安全层已改为远到近投影，使最近表面拥有最终权威；扫掠球同时从
0.284 m primitive 近似提升为包含旋翼的 0.332 m 包络。该修复只作用于执行层。

### 3.4 点模型规划边界与 Isaac 旋翼包络不一致

v3 中第 3 架机在约 6.42 m 路径后停于东墙前，原版 FSM 持续报告
`Replan: collision detected`。日志证明原版前沿模块选择的下一视点为
`(9.09776, -10.0172, 2.32698)`；Warehouse 东墙碰撞面约为 `x=9.37 m`，而
0.332 m 旋翼包络要求机心 `x<=9.038 m`。旧配置仍允许点模型在 `x<9.2 m`
规划，因而安全层只能永久制动，任务无法完成。

修复仅校准 Isaac Warehouse 的可达机心坐标盒：X `[-10.0, 9.0]`、Y
`[-11.9, 17.6]`、Z `[0.4, 8.6]`。边界由测量碰撞面减去/加上旋翼包络并向
0.1 m SDF 网格内部取整；原版地图更新、前沿/HGrid、LKH、Kinodynamic A*、
NLopt、yaw 和 FSM 实现及参数均未替换。新增回归会验证规划盒不包含物理不可达
机心，同时 safety lidar/PhysX 点仍不会反馈给规划器。

### 3.5 稀疏诊断点云破坏已知自由空间连通性

v4 越过了 v3 的东墙视点，但使用的 800-ray 诊断配置仅把 160×120 深度帧中的
约 4.3% 样本送入 0.1 m SDF。约 193 s 后第 2、3 架分别停在
`(-8.853,-6.311,8.198)` 和 `(4.614,-4.860,7.643)`；旧轨迹已结束、控制指令
接近零，而非乐观原版 A* 对 8.7 m/11.4 m 远端视点累计返回 `No path` 超过
1400 次。稀疏射线之间残留的 UNKNOWN 体素使已飞过区域不连通。

正常入口现保留原版 640×480、`skip_pixel=2` 网格的全部最多 76,800 条射线；
低 ray budget 明确定义为仅诊断用途。v5 先以 160×120 全 18,644 样本、10 Hz
验证空间密度修复，算法代码与非乐观 A* 判定保持不变。

### 3.6 ROS1 throttle 调用点语义

v5 首批无人机进入原版 `FINISH` 后，日志持续输出 `state: FINISH`，却没有紧随其后的
`finish exploration.`。根因是 ROS2 兼容层最初让所有 throttle 宏共用一个全局时间戳，
而 ROS1 的 throttle 状态按宏调用点独立。兼容层现按 `文件:行号` 保存节流状态；完成
观察器同时接受两个均由未改 FSM 产生的 FINISH 证据。该修改只修复日志/观察语义，
不参与状态转移或规划决策。

### 3.7 独立安全制动后的执行状态恢复

v5 在约 500 s 前保持 0 接触，且第 1、3 架已按原版 FSM 进入 IDLE、返航和
FINISH；但第 4 架的 Isaac 实体约位于 `(-0.457, 3.794, 0.844)` 时，原版理想跟踪
FSM 已从约 `(2.819, 8.313, 7.915)` 的未执行 B-spline 状态继续规划。其根因是
CBF/PhysX 安全层只约束执行端，而原版默认控制器能够理想跟踪，安全制动后没有
“轨迹未被实体执行”的接口。该轮因此判为 FAIL，不能用零碰撞掩盖任务未真实执行。

新增的 ROS2/Isaac 控制边界只在 PositionCommand 与实测 odometry 的误差连续 1.0 s
超过 1.2 m 且原版轨迹速度已低于 0.25 m/s，或误差无论速度均已超过 3.0 m 时，
先急停再通知 FSM 丢弃未执行轨迹。该判定把仍在运动的正常动态滞后与 v5 中
近零速微小轨迹、约 8.5 m 的真实状态分离区分开。FSM 仅保留原版已选的
`next_pos_`/`next_yaw_`，复用原版碰撞重规划分支并从实测位置、速度重新调用同一
Kinodynamic A*、NLopt 和 yaw 流水线；回调中禁止更新前沿、重选视点或修改目标。
一次通知后仅在误差重新降到 0.6 m 以下时重新武装，避免同一旧轨迹在替代轨迹
尚未发布时重复挤占原版串行回调队列。

### 3.8 物理安全绕行后的返航自由空间断裂

v12 中整机 PhysX 扫掠屏障已将环境接触降为 0，第 2 架机按原版 FSM
返回自己的起点，物理误差 0.496 m 并进入 FINISH。但第 5 架机在约 447.5 s
进入原版返航分支后，持续在未改的 `planTrajToView`/非乐观 A* 中报
`No path to next viewpoint`；停止时距自己起点 5.613 m。算法选择的返航目标仍是
触发时记录的 `start_pos_`，阻塞来自 Isaac 安全层绕开理想轨迹后，实际飞过的
自由通道没有被深度射线稳定保留，于是 UNKNOWN 体素切断了回家路径。

初版修复在每帧深度融合后立即写入实测通道。v13 运行至 604.07 s 仍为 0 接触，
但 UNKNOWN 邻域把这些管壁识别成新前沿；593.47→604.07 s 短窗口内第 1/2/5 架
航程增量为 0，No-path 由 1535 增至 1812，未出现任何返航，v13 因而主动终止并
判为 FAIL。

修订后的边界在探索阶段只缓存实测里程计，不改占据图。仅在原版 FSM 已设置
`go_back_`、即将调用未改 `planTrajToView` 时，才将本机 0.30 m 历史通道写为自由并
同步运行原版障碍膨胀/ESDF。该半径小于 PhysX 证实无碰撞通过的 0.332 m 整机
包络；通道不进入 map-chunk，因此其他仍在探索的无人机不会看到人工前沿。返航目标、
A* UNKNOWN 判定、路径搜索和优化器仍为原版。

## 4. Isaac 运行记录

| 轮次 | 配置 | 结果 |
|---|---|---|
| `post_degenerate_fix_130s` | 5 机，200 Hz physics，5 Hz，160×120，800 rays，130 s | 原版流水线通过；0 碰撞；最小机间距 1.2267 m；最小净空 0.1329 m；未要求完成 |
| `acceptance_300s_hq_clean` | 5 机，100 Hz physics，5 Hz，160×120，800 rays，300 s | 6 接触；最小机间距 1.1734 m；未完成；FAIL，并据此修复安全层 |
| `acceptance_900s_safety_v2` | 5 机，200 Hz physics，5 Hz，160×120，800 rays，运行至 307.1 s | 0 接触；第 3 架从约 234 s 被重复计算的固定净空困住，持续 No path；FAIL，并据此去除双计静态余量 |
| `acceptance_900s_safety_v3` | 5 机，200 Hz physics，5 Hz，160×120，800 rays，运行至约 58 s | 0 接触；第 3 架被原版选择的 `x=9.09776` 视点与 0.332 m 实体包络夹在东墙前；FAIL，并据此校准可达机心坐标盒 |
| `acceptance_900s_safety_v4` | 5 机，200 Hz physics，5 Hz，160×120，800 rays，运行至约 217 s | 0 接触；第 2、3 架因稀疏点云留下 UNKNOWN 断面而长期无路径；FAIL |
| `acceptance_900s_safety_v5` | 5 机，200 Hz physics，10 Hz，160×120，全 18,644 深度样本，运行至约 500 s | 0 接触；第 1、3 架正常返航完成，但第 4 架出现约 8.5 m 规划/实体状态分离；FAIL |
| `acceptance_900s_safety_v6` | 5 机，200 Hz physics，10 Hz，160×120，全深度网格 | 约 8 s 主动终止；发现执行层 0.85 m/s 上限低于原版 `manager.max_vel=1.5`，正常飞行即产生 0.68–0.86 m 跟踪误差；FAIL，并据此对齐控制接口速度约束 |
| `acceptance_900s_safety_v7` | 5 机，200 Hz physics，10 Hz，全 160×120 深度 | 不足 10 s 主动终止；同一未执行轨迹在恢复冷却后重复上报，14 次重规划造成回调拥塞；FAIL |
| `acceptance_900s_safety_v8` | 5 机，200 Hz physics，10 Hz，全 160×120 深度 | 不足 10 s 主动终止；不同有效运动轨迹仍产生 11 次大误差恢复，造成协同/LKH 回调积压和物理时钟停滞；FAIL |
| `acceptance_900s_safety_v9` | 5 机，200 Hz physics，10 Hz，全 160×120 深度 | 约 8 s 主动终止；恢复事件为 0，但 RELIABLE 稠密点云在算法消费积压时反向阻塞 Isaac 物理生产者；FAIL |
| `acceptance_900s_safety_v10` | 5 机，Warehouse Simple，Crazyflie dynamics，全深度网格，最多 900 s | 在 7.5 s 仿真时间由诊断人员主动中止；后续阶段探针证明并非死锁，而是无中间日志时的慢速正常计算，因此本次不计验收结果 |
| `stall_phase_probe_40s` | 同正式配置，加入只读阶段与 SIGUSR1 堆栈探针 | 已依次通过第 1500、2000 物理步的全部五机控制、PhysX scene query、`world.step` 与观测发布阶段；确认 v10 无死锁，主动结束探针 |
| `acceptance_900s_safety_v11` | 5 机，Warehouse Simple，Crazyflie dynamics，全深度网格，运行至约 140 s | Drone 5 在薄货架框架旁产生 2 次轻微接触，最大力 0.7175 N；最小机间距 1.7319 m；FAIL。定位为仅使用 `SweepHit.position` 丢失整机扫掠自由行程语义，新增 `SweepHit.distance` 直接屏障 |
| `acceptance_900s_safety_v12` | 5 机，Warehouse Simple，Crazyflie dynamics，全深度网格，运行至约 500 s | 0 接触；最小机间距不小于 1.565 m；障碍净空为正；第 2 架返航完成且误差 0.496 m，第 5 架返航因 UNKNOWN 通道断裂持续 No path，停止时返航误差 5.613 m；FAIL |
| `acceptance_900s_safety_v13` | 5 机，Warehouse Simple，Crazyflie dynamics，全深度网格，运行至 604.07 s | 0 接触；最小机间距 1.4670 m；最小障碍净空 0.04986 m；连续写入实测通道生成人工前沿，尾段 No-path 1812 次且无返航；FAIL |
| `acceptance_900s_safety_v14` | 5 机，Warehouse Simple，Crazyflie dynamics，全深度网格，运行至 382.5 s 后外部中断 | 0 接触；仅第 1 架 FINISH，第 5 架持续 No path；未生成最终结果；FAIL |
| `acceptance_900s_coverage95_v15` | 5 机，200 Hz physics，10 Hz，160×120 全深度网格，95% 或 900 s 停止 | 286.07 s 达到 95.1913%；0 接触；最小机间距 1.4010 m；最小净空 0.05025 m；原版流水线完整执行且无崩溃/LKH 失败；按覆盖率停止时 0/5 返航，覆盖率测试 PASS、完整任务验收 FAIL |
| `acceptance_900s_full_return_v16` | 5 机，200 Hz physics，10 Hz，160×120 全深度网格，禁用覆盖率提前停止，900 s 上限 | 原版 FSM 在 677.69 s 自主结束；覆盖率 99.4784%；5/5 `Go back`、5/5 `FINISH`；返航误差 0.785/0.742/0.411/0.493/0.508 m；0 接触；最小机间距 1.4204 m；最小净空 0.05026 m；LKH 失败/进程崩溃 0；PASS |

130 秒通过轮次的算法证据：前沿 1492、HGrid 1633、ATSP 2976、ACVRP 7843、
成对请求/响应 594/594、Kinodynamic 747、NLopt/yaw 1483/1483、LKH 失败 0、
进程崩溃 0。

v16 完整验收的算法证据：前沿更新 3435、HGrid tour 4088、ATSP 7650、ACVRP
32534、成对请求/响应 239/235、Kinodynamic 中间目标 2040、NLopt/yaw
3699/3699、LKH 失败 0、进程崩溃 0。五机各自覆盖率为 99.3371%、99.3374%、
99.3366%、99.3393%、99.4784%；由于 map chunk 已在各机间融合，联合值按最完整
的 peer-fused 地图取 99.4784%。完整机器可读结果见
`validation/acceptance_900s_full_return_v16/warehouse_simple_result.json`。

## 5. 最终门槛

最终结果必须同时满足：Isaac 正常退出；5/5 执行原版流水线；HGrid、前沿、LKH、
协同、Kinodynamic、NLopt、yaw 均有证据；LKH 失败和进程崩溃为 0；接触碰撞为 0；
最小机间距离不小于 1.0 m；障碍净空为正；5/5 记录 `Go back to` 和
`finish exploration`。任何一项失败均不得把移植标记为完成。
