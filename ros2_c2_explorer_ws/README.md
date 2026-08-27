# C²-Explorer ROS 2 / Isaac Sim 源码保真复现

这是原始 ROS 1 `C2-Explorer` 的 ROS 2 Humble / Isaac Sim 复现工作区。C² 的任务表示、连通图、连续性驱动分配、去中心化会合优化、前沿/网格游历、LKH-3 求解和 B 样条轨迹规划继续使用原始 C++ 源码；ROS 2 兼容层只负责中间件、消息/服务和仿真输入输出。

复现位置：`racer_chen/ros2_c2_explorer_ws`

基准资料：

- 原始源码：[Robotics-STAR-Lab/C2-Explorer](https://github.com/Robotics-STAR-Lab/C2-Explorer)，commit `fd1c76a49a4453f91da4984e123b56932c5382f3`
- 论文 PDF 不随本仓库分发；用于复核的本地副本哈希如下。
- 论文 SHA-256：`0c50667957b13e8b2bbcc639d67d7b12eb5e7f0d4e3f1d3e255a73e4ba23f0df`

## 工作区结构

- `src/c2_explorer_core`：原始 C² 算法源码、ROS 1 API 兼容层、ROS 2 节点和 LKH-3 服务。
- `src/c2_explorer_msgs`：原始自定义消息/服务的 ROS 2 IDL 等价版本。
- `src/c2_explorer_isaac`：Isaac Sim 深度点云、里程计、PhysX 无人机控制、启动和验收层。
- `third_party/nlopt-2.7.1`：本地 NLopt；不写入系统目录。
- `tests/check_source_fidelity.py`：逐文件核对 vendored C² 源码与 ROS 1 基准。
- `tests/check_runtime_fidelity.py`：逐字段核对消息/服务，并逐项核对 ROS 1 算法参数。
- `ALGORITHM_FIDELITY.md`：论文模块到实现源码的对应关系。
- `PORTING_CHANGES.md`：所有有意移植差异及其原因。
- `LOGIC_AUDIT.md`：完成后的逻辑一致性复审、修正项和回归证据。

## 依赖

- Ubuntu 22.04、ROS 2 Humble
- Isaac Sim 5.1（默认 `~/software/isaacsim`，可用 `ISAAC_SIM_ROOT` 覆盖）
- Eigen、PCL、Armadillo、ROS 2 常用消息包
- 工作区已包含 NLopt 2.7.1 和原始 C² 所用 LKH-3 源码

## 编译

```bash
cd /path/to/racer_chen/ros2_c2_explorer_ws
./scripts/build_ros2.sh
```

脚本先把 NLopt 安装到本工作区的 `third_party/install`，再用 `colcon` 构建三个 ROS 2 包，不需要 `sudo make install`。

## 在 Isaac Sim 运行

默认是 5 架 UAV、`warehouse_simple.usd`、900 秒、无界面运行：

```bash
cd /path/to/racer_chen/ros2_c2_explorer_ws
ROS_DOMAIN_ID=181 ./scripts/run_isaac.sh
```

打开 Isaac 交互窗口和 C² 规划可视化：

```bash
ROS_DOMAIN_ID=181 C2_DRONE_COUNT=5 C2_DURATION=900 \
  ./scripts/run_isaac_visualization.sh
```

快速闭环测试（不要求在 30 秒内完成整张地图）：

```bash
ROS_DOMAIN_ID=181 C2_DRONE_COUNT=3 C2_DURATION=30 \
  C2_PHYSICS_RATE_HZ=100 C2_SENSOR_RATE_HZ=10 \
  C2_REQUIRE_COMPLETION=0 C2_STOP_ON_COMPLETION=0 \
  ./scripts/run_isaac.sh
```

## warehouse_full 理想通信/Sionna 配对实验

下面的入口按完全相同的场景、初始位置、随机种子和仿真频率依次运行两个实验，唯一受控变量是通信传输层：

```bash
cd /path/to/racer_chen/ros2_c2_explorer_ws
./setup_sionna_env.sh
C2_EXPERIMENT_DURATION=60 ./run_warehouse_full_communication_experiments.sh
```

运行 10-UAV 配对实验：

```bash
C2_EXPERIMENT_DRONE_COUNT=10 C2_EXPERIMENT_DURATION=60 \
  ./run_warehouse_full_communication_experiments.sh
```

第一个实验使用零时延、零丢包的理想广播；第二个实验使用严格 Sionna RT 射线追踪和 5G NR LDPC/CP-OFDM 链路抽象，不允许解析模型回退。每次运行会在已被 Git 忽略的 `experiments/` 下生成 manifest、日志、结果、完整性校验和对比 JSON。Sionna 通信节点复用同一仓库中 `ros2_pairwise_robust_racer_ws` 的已验证实现。

常用环境变量：

- `C2_DRONE_COUNT`：UAV 数，默认 5。
- `C2_DURATION`：最大仿真时长，默认 900 秒。
- `C2_SCENARIO`：`warehouse_simple`、`warehouse_loaded`、`warehouse_loaded_center` 或 `warehouse_full`。
- `C2_HEADLESS` / `C2_VISUALIZE`：无界面或交互可视化。
- `C2_REQUIRE_COMPLETION`：是否把全部 FSM 到达 `FINISH` 作为验收条件。
- `C2_STOP_ON_COMPLETION`：全部 FSM 完成后是否提前关闭 Isaac。
- `C2_DEBUG_OPT_OUTPUT`：输出会合协议诊断证据；默认 0，只影响日志，不参与算法决策。
- `C2_RESULT_DIR`：日志与 JSON 验收结果目录，默认 `validation/`。
- `C2_SCENE_USD` / `C2_VEHICLE_USD`：覆盖场景或无人机 USD。
- `C2_PHYSICS_RATE_HZ` / `C2_SENSOR_RATE_HZ`：物理和传感器频率；正式复现默认 1000/30 Hz。

建议为并行实验设置不同的 `ROS_DOMAIN_ID`，Fast DDS 可用范围为 0–232。

运行入口会自动完成以下流程：ROS 2 C² 节点与 TSP/ACVRP 服务启动 → Isaac 场景/无人机加载 → 里程计和点云就绪屏障 → 发布统一探索触发 → 原始 C² FSM 规划和下发 B 样条 → PhysX 飞行 → 输出碰撞、间距、覆盖率与算法证据。

## 验证

运行时字段/参数检查不需要外部文件；源码逐文件保真检查需要把上游指定 commit 另行克隆到工作区旁边：

```bash
python3 tests/check_runtime_fidelity.py

git clone https://github.com/Robotics-STAR-Lab/C2-Explorer ../C2-Explorer
git -C ../C2-Explorer checkout fd1c76a49a4453f91da4984e123b56932c5382f3
python3 tests/check_source_fidelity.py

source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 interface show c2_explorer_msgs/msg/MeetingOpt
ros2 launch c2_explorer_isaac c2_explorer_warehouse.launch.py \
  drone_count:=1 scenario:=warehouse_simple
```

详细一致性结论与边界见 `LOGIC_AUDIT.md`、`ALGORITHM_FIDELITY.md` 和 `PORTING_CHANGES.md`。运行时生成的 `validation/`、`experiments/`、`build/`、`install/` 和 `log/` 均不纳入版本控制。
