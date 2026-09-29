# C²-Explorer warehouse_full 通信对照实验

实验日期：2026-08-26
实验目录：`experiments/warehouse_full_c2_ideal_vs_sionna_60s_final2_20260826_2335`

## 结论

两个 60 s 配对实验均完整运行到设定时长，ROS/Isaac 进程无崩溃，结果文件的独立完整性检查均通过。理想通信传输了全部 68,236 个数据包；严格 Sionna 通信传输了 6,843/68,280 个数据包，PDR 为 10.022%，平均已送达包时延为 20 ms。对应的联合地图覆盖率分别为 2.4331% 和 1.8329%；Sionna 组比理想通信组低 0.6002 个百分点（相对低 24.67%）。

这是一组固定 60 s 的受控通信实验，不是完整仓库探索完成时间测试。该时段内只有 UAV 1 产生了显著位移，其余四架很快进入原 C² 的 `FINISH` 状态；因此结果可以验证两种通信链路下的可运行性和短时差异，不能单独代表五机完整探索性能。

## 受控条件

| 条件 | 数值 |
| --- | --- |
| 场景 | `warehouse_full_with_industrial_ap` |
| UAV 数量 | 5 |
| 仿真时长 | 60 s |
| 随机种子 | 42 |
| PhysX / 深度频率 | 50 Hz / 10 Hz |
| 每帧相机射线预算 | 76,800 |
| 起点布局 | 仓库四角加中心，最小初始机间距 19.9404 m |
| C² 算法参数 | 两组完全相同，保留原版 `communication.connection_threshold=5.0 m` |
| 唯一受控差异 | C² 原始八类机间消息的通信传输层 |

起点为 `[-26.6, 1.0, 0.75]`、`[5.6, 1.0, 0.75]`、`[-25.6, 29.7, 0.75]`、`[4.6, 29.7, 0.75]`、`[-11.5, 15.6, 0.75]`。

通信设置：

- 理想组：所有包即时送达，零丢包、零排队时延。
- Sionna 组：28 GHz、100 MHz、UAV 发射功率 23 dBm、4×4 UAV 阵列、分布式 UAV-to-UAV 拓扑；Sionna RT 最大深度 2、每源 100,000 条射线、0.5 Hz 精确求解和 10 Hz 链路发布；不允许解析路径损耗回退；无重传。
- 原始 C² 的 5 m 连通图阈值在两组均保持不变。理想通信指传输层可靠，并未扩大或删除算法内部的连通/会合判据。

## 实验结果

| 指标 | 理想通信 | Sionna 实际通信 |
| --- | ---: | ---: |
| 仿真时长 | 60.000 s | 60.000 s |
| 联合地图覆盖率 | 2.4331% | 1.8329% |
| UAV 1 路径长度 | 27.877 m | 45.691 m |
| UAV 2–5 路径长度 | 均小于 0.001 m | 均小于 0.001 m |
| 最小机间距 | 13.892 m | 15.396 m |
| 最小障碍物净距 | 0.0394 m | 0.0609 m |
| PhysX 接触事件 | 1 | 0 |
| 尝试发送包 | 68,236 | 68,280 |
| 已送达包 | 68,236 | 6,843 |
| 已判定丢弃包 | 0 | 61,432 |
| PDR | 100% | 10.022% |
| 平均送达时延 | 0 ms | 20 ms |
| 精确 Sionna RT 样本 | 0 | 11,600 |
| 连通图 / HGrid / 前沿游历 | 110 / 83 / 77 | 83 / 57 / 51 |
| 轨迹规划 | 62 | 54 |
| LKH 失败 / 进程崩溃 | 0 / 0 | 0 / 0 |
| 科学验收状态 | 未通过：1 次物理接触 | 通过 |

Sionna 组的 61,432 个已判定丢包由 2,320 个无链路丢包和 59,112 个 PER 丢包组成；停止仿真时另有 5 个包（584 bytes）仍在队列中，因此 `尝试发送 != 已送达 + 已判定丢弃`。Sionna 场景成功就绪，使用了 11,600 个精确 RT 样本，没有解析回退样本。

联合覆盖率定义为“已知 SDF voxel / 配置规划空间 voxel”，多机联合值取经过对等地图融合后的各 agent 地图最大值。理想组的一次接触最大接触力为 37.6703 N，因此其完整性检查通过，但严格的零碰撞科学验收为失败；不能隐藏或当作通信错误处理。

## C² 逻辑一致性复核

正式实验前后再次核对了 ROS 1 基准、论文链路和 ROS 2 运行参数：

- 405 个原始算法文件中 400 个逐字节一致；5 个文件只有已锁定 SHA-256 的消息字段/时间适配和健壮性保护。
- 13 个原始消息/服务接口字段等价，136 个 ROS 1 算法参数逐项相等；Isaac 特有的 5 个边界参数被单独审计。
- 正式实验确实经过原始连通图、HGrid/前沿游历、LKH 和轨迹规划调用链，两组均无 LKH 失败和进程崩溃。
- 复核修正了原源码的未初始化布尔值、Eigen 惰性表达式悬空引用和零段轨迹越界；这些修正只确定原本预期分支或处理退化输入，不改写正常 C² 算法数学与决策逻辑。

完整差异与证据见 `LOGIC_AUDIT.md`、`ALGORITHM_FIDELITY.md` 和 `PORTING_CHANGES.md`。

## 结果与复现入口

- 实验清单：`experiments/warehouse_full_c2_ideal_vs_sionna_60s_final2_20260826_2335/experiment_manifest.json`
- 对比摘要：`experiments/warehouse_full_c2_ideal_vs_sionna_60s_final2_20260826_2335/comparison_summary.json`
- 理想通信结果：`experiments/warehouse_full_c2_ideal_vs_sionna_60s_final2_20260826_2335/ideal_perfect_communication/warehouse_full_result.json`
- Sionna 通信结果：`experiments/warehouse_full_c2_ideal_vs_sionna_60s_final2_20260826_2335/sionna_actual_communication/warehouse_full_result.json`

重新运行同规格实验：

```bash
cd /home/jiazheng/RACER_warehouse_loaded_portable_20260805/racer_chen/ros2_c2_explorer_ws
C2_EXPERIMENT_DURATION=60 ./run_warehouse_full_communication_experiments.sh
```
