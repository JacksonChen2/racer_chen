# RACER + Isaac Sim + Sionna RT 验证报告

更新日期：2026-08-10

## 结论

新的独立工作区已经完成构建、算法源码一致性检查、通信代理单元/端到端测试，
并在 Isaac Warehouse Simple 中用在线 Sionna RT 路径求解器完成了 3 机 60 秒、
5 机 30 秒集成测试以及 5 机 900 秒完整时限测试。原版 RACER 全部核心模块均被
实际调用，Isaac 接触碰撞为零。900 秒测试没有通过完整探索与返航验收：最终最高
覆盖率 73.436%，只有 1/5 架进入 FINISH 并返回。详细效率对照见
`COMMUNICATION_PERFORMANCE_COMPARISON.md`。

## 静态与同输入验证

- `racer_source_fidelity` 检查 ROS1 算法基准的 467 个文件，只允许已登记的
  ROS2 API 适配和两处未定义行为修复。
- 原版与移植版 LKH、B-spline 使用同输入测试验证输出一致。
- 通信层不反序列化或修改 RACER 消息，只传递序列化字节。
- `test_ideal_proxy.py` 验证跨机消息字段、顺序逐项保持且不会回送发送者。
- 全工作区测试结果：73 项，0 error，0 failure，0 skipped。

## Warehouse RF 场景

输入场景为实际使用的 `warehouse_simple.usd`。转换后的 Sionna 场景包含 1,877 个
网格代理、22,524 个三角形，材质分为 metal、concrete、ceiling_board 和
chipboard；边界约为 `[-12.04, -18.04, 0]` 至 `[11.96, 20.78, 9.30]` 米。

独立 PathSolver 冒烟测试使用 2 架无人机、20,000 rays、最大反射/绕射深度 2，
成功产生 67 条有效传播路径，复数通道系数全部有限，单次求解约 0.73 秒。

## Isaac Sim 在线集成结果

| 项目 | 3 机 / 60 s | 5 机 / 30 s |
|---|---:|---:|
| 通信模式 | 在线 `sionna` | 在线 `sionna` |
| 进入原版 FSM EXEC | 3/3 | 5/5 |
| Sionna 精确链路样本 | 3,468 | 5,600 |
| 已投递分组 | 4,365 | 7,877 |
| 平均投递时延 | 154.25 ms | 394.54 ms |
| LKH 失败 | 0 | 0 |
| Isaac 接触碰撞 | 0 | 0 |
| 最小机间距离 | 2.172 m | 1.921 m |
| 最小障碍净空 | 0.151 m | 0.129 m |
| 截止时融合覆盖率 | 11.53% | 7.16% |

3 机运行实际记录了前沿检测 326 次、HGrid 更新 325 次、Kinodynamic A* 98 次、
ACVRP 287 次、ATSP 626 次、NLopt/yaw 规划 316 次和 pair 协同请求/响应 29/16 次。
5 机运行记录了前沿检测 269 次、HGrid 更新 317 次、Kinodynamic A* 116 次、
ACVRP 69 次、ATSP 602 次、NLopt/yaw 规划 242 次和 pair 协同请求/响应 57/13 次。
这说明结果不是仅启动节点，而是原版规划和协同流水线确实在有损链路下执行。

通信统计中存在队列丢包和 TTL 超时，主要来自地图 chunk 的突发复制；轨迹和协同
消息具有更高优先级，有限队列不会无限占用内存。启动阶段首次 Sionna 求解完成前
的 `no_link` 也会单独计数，不会静默切换到自由空间解析模型。

## 验证产物

- `validation/isaac_sionna_3uav_60s_v3/warehouse_simple_result.json`
- `validation/isaac_sionna_5uav_30s_v1/warehouse_simple_result.json`
- `validation/comparison_5uav_sionna_full_v1/warehouse_simple_result.json`
- `validation/sionna_path_solver_smoke.log`

## 尚未完成的高强度验收

已经执行最长 900 秒的高强度测试。在线 Sionna RT、五机实际规划和零碰撞通过，
但 95% 覆盖与 5/5 FINISH/返航没有通过。结果位于
`validation/comparison_5uav_sionna_full_v1/warehouse_simple_result.json`。下一步应在
不修改 RACER 规划决策的前提下标定通信层容量和队列/地图传输模型，然后重复同一
900 秒验收。
