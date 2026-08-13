# RACER `warehouse_loaded` 可移植发布包

这个发布包把四类实现和运行入口放在同一个目录中：

1. `original_racer_ros1_ws/src/RACER`：原始 RACER ROS 1 源码；
2. `ros2_3d_C_ws`：复现的 C++ / ROS 2 三维 RACER；
3. `ros2_3d_py_ws`：复现的 Python / ROS 2 三维 RACER；
4. `ros2_3d_sionna_ws`：带 Sionna RT 通信信道与 C++ 通信模拟器的 RACER；
5. `warehouse_loaded.usd`、`ros2_3d_py_ws/warehouse.usd` 和
   `isaac_assets`：Warehouse 场景层、通信射线追踪代理和无人机模型。

所有测试、启动和 Isaac 可视化均由包内的 `racer` 入口完成，不需要再写脚本。

## 新电脑要求

ROS 2 / Isaac 主流程的验证组合是：

- Ubuntu 22.04 x86-64；
- ROS 2 Humble Desktop；
- Isaac Sim 5.1，且启用其 ROS 2 Humble bridge；
- 支持 RTX 的 NVIDIA GPU，建议显存至少 8 GB；
- 建议至少 32 GB 内存和 40 GB 可用磁盘；
- 第一次加载 Isaac Warehouse 资产和第一次安装 Sionna 时需要网络。

Isaac Sim、NVIDIA 驱动和 ROS 2 本身不包含在压缩包内。它们与硬件、驱动和
许可绑定，不能靠复制另一台电脑的 `install/` 或 Python 虚拟环境可靠运行。

如果 Isaac 不在 `~/isaacsim`，先设置：

```bash
export ISAAC_SIM_ROOT=/你的/isaacsim/绝对路径
```

## 解压后的首次使用

```bash
cd RACER_warehouse_loaded_portable_20260805
./racer doctor
./install_dependencies.sh
./racer setup comm
./racer build all
./racer test all
```

`run` 和 `visualize` 在发现对应工作区尚未构建时会自动首次构建，因此也可在
依赖齐全后直接启动。`setup comm` 会通过 pip 下载 Sionna RT 依赖。

## 运行与可视化

C++ / ROS 2 版，在 Isaac Sim 中显示三架无人机、青色已占用体素、彩色飞行
轨迹和当前规划路径：

```bash
./racer visualize cpp --duration 120 --drones 3
```

Python / ROS 2 版：

```bash
./racer visualize python --duration 120 --drones 3
```

带混合 Sionna 信道的 C++ 版：

```bash
./racer visualize comm --mode sionna_hybrid --duration 120 --drones 3
```

不打开窗口的正式运行：

```bash
./racer run cpp --duration 900 --drones 3
./racer run python --duration 900 --drones 3
./racer run comm --mode sionna_hybrid --duration 900 --drones 3
```

结果 JSON 写入 `results/`。正式验收阈值是 90% 已知体积覆盖率、零碰撞/接触、
所有无人机里程计与 agent 状态均在线；120 秒等短演示即使数据链正常，也可能因
没有达到 90% 而返回非零退出码。

快速 ROS-only 回归不启动 Isaac：

```bash
./racer mock cpp --duration 60
./racer mock python --duration 60
```

## 通信仿真模式

通信版中，每个 agent 的地图块、状态和任务分配消息都先序列化，再经过 C++
通信模拟器。Sionna 节点根据无人机真值位置计算链路增益、RSS、SNR、Doppler
和 LOS；C++ 层继续施加带宽、排队、时延、抖动、MTU、丢包、TTL 和重传。

- `ideal`：理想上界；
- `range_only`：仅距离门限；
- `sionna`：在线 Sionna 链路；
- `sionna_hybrid`：包内无线电地图缓存 + 低频精确修正，默认模式。

包内已带 `warehouse_loaded` 的 RF 代理网格和约 1.9 MB 的默认无线电缓存，普通
复现实验不需要重新转换 USD 或生成缓存。修改场景、材料、频率或任务边界后才需
运行通信工作区自带的准备工具。

## 原始 RACER

原始代码依赖 ROS Melodic/Noetic、catkin、NLopt 2.7.1、Armadillo 和 LKH-3，
它本身没有 Isaac Sim 接口，也不是 `warehouse_loaded` 的执行后端。它被完整保留
用于论文算法基线、代码比对和原始 RViz 仿真。推荐在 Ubuntu 20.04 + ROS Noetic
环境运行：

```bash
./racer build original
./racer run-original
```

ROS 2 C++、Python 和通信版才是本发布包中经过 `warehouse_loaded` / Isaac Sim
接口适配的实现。不要把原始 ROS 1 RViz demo 的结果写成 Isaac 验收结果。

## 可复现性边界

- USD 层引用 Isaac 5.1 Warehouse 内容；首次使用必须能访问相应 Isaac 资产。
- C++ 版的算法主体为 C++；Isaac SimulationApp/PhysX 传感器仍由包内 Python
  bridge 启动，这是 Isaac 5.1 API 的接口边界，不代表规划算法由 Python 执行。
- 通信版 RF 参数是可配置仿真假设，不是实测无线电标定值。
- 压缩包不包含本机生成的 `build/`、`install/`、`log/`、2.5 GB 运行日志或绑定
  Python ABI 的 Sionna 环境；新机首次构建能避免绝对路径和 ABI 污染。

各实现的算法细节、参数和历史验证记录见各工作区内的 README 与 `VALIDATION.md`。
