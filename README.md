# modified_RACER：ROS 2 + Python + Isaac Sim 多无人机协同探索

本仓库是 [Robotics-STAR-Lab/RACER](https://github.com/Robotics-STAR-Lab/RACER)
的 ROS 2、Python-first 与 NVIDIA Isaac Sim/Isaac Lab 迁移版本。

迁移目标是在不改变 RACER 核心算法数据流和约束含义的前提下，将原有
ROS 1、catkin 和 C++ 工程迁移到以下软件栈：

| 项目 | 目标环境 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 |
| ROS | ROS 2 Humble |
| ROS 侧 Python | Python 3.10 |
| 仿真器 | NVIDIA Isaac Sim 5.1 |
| Isaac 内部 Python | Isaac Sim 自带版本 |
| ROS 客户端 | `rclpy` |
| 构建系统 | `colcon`、`ament_python`、必要的 `ament_cmake` |
| 仿真通信 | Isaac Sim ROS 2 Bridge |
| 可视化 | RViz2 |

> 当前代码已经在 Ubuntu 22.04、ROS 2 Humble 和 Python 3.10 下通过构建、
> 算法回归、单机输入链路与三机 ROS 图烟雾测试。由于本机没有 ROS Noetic，
> 尚未执行 ROS 1/C++ 与 ROS 2/Python 的同输入数值对拍；真实 Isaac UAV
> 闭环也仍需使用具体 Stage 和飞行器资产验证。详细边界见
> [验证状态](docs/VALIDATION_STATUS.md)。在完成闭环验证前，请勿用于真实飞行。

## 1. 分支与上游基线

- `main`：完整保留的上游 RACER 参考版本。
- `migration/ros2-python-isaac`：ROS 2 + Python + Isaac Sim 迁移版本。
- 上游参考提交：`049c332e3634ef72d8beb155b4c13dc91ca52916`。
- 原始 `swarm_exploration/` 和 `uav_simulator/` 目录继续保留，用于逐项比较
  算法逻辑。
- 两个原始 ROS 1 目录包含 `COLCON_IGNORE`，不会进入新的 ROS 2 工作区构建。

建议所有迁移开发、构建和运行工作都在
`migration/ros2-python-isaac` 分支完成。

## 2. 迁移后的系统架构

```text
┌─────────────────────────────────────────────────────────────┐
│ Isaac Sim 5.1 / Isaac Lab                                  │
│                                                             │
│  UAV + RTX/Lidar Sensor + Vehicle Controller                │
│         │                         ▲                         │
│         ▼                         │                         │
│  Odometry / PointCloud       Pose / Twist / Accel           │
│         └──────── Isaac Sim ROS 2 Bridge ───────────────────┤
└──────────────────────────── DDS ─────────────────────────────┘
                              │
┌─────────────────────────────▼───────────────────────────────┐
│ Ubuntu 22.04 + ROS 2 Humble + Python 3.10                  │
│                                                             │
│  racer_ros                                                  │
│  ROS 2 I/O、探索状态机、轨迹服务器、Isaac 命令适配          │
│                         │                                   │
│                         ▼                                   │
│  racer_core                                                 │
│  地图、前沿、分区、搜索、优化、轨迹和协同算法                │
│                         │                                   │
│                         ▼                                   │
│  racer_interfaces + racer_bringup + RViz2                   │
└─────────────────────────────────────────────────────────────┘
```

ROS 2 Humble 使用 Python 3.10，而 Isaac Sim 使用自己的 Python。两个 Python
环境不互相安装或导入依赖，只通过 DDS 和 Isaac Sim ROS 2 Bridge 通信，从而
避免 Python ABI 冲突。

### 2.1 ROS 2 软件包

新的 colcon 工作区位于 `ros2_ws/`：

| 软件包 | 构建类型 | 作用 |
| --- | --- | --- |
| `racer_interfaces` | `ament_cmake` | 生成 RACER 的 29 个消息和 2 个服务 |
| `racer_core` | `ament_python` | 不依赖 ROS 的 Python 核心算法 |
| `racer_ros` | `ament_python` | `rclpy` 节点、消息转换和 ROS 2 通信 |
| `racer_bringup` | `ament_python` | 参数、单机/多机启动文件和 RViz2 |
| `racer_isaac` | `ament_python` | Isaac Sim Bridge 图和 Isaac Lab 接入资产 |

### 2.2 算法处理链

迁移后的单次规划流程为：

1. 接收无人机里程计和世界坐标系点云。
2. 通过概率栅格、光线投射、障碍膨胀和 ESDF 更新本地地图。
3. 在无人机之间异步交换地图分块和缺失分块索引。
4. 搜索、聚类和分割前沿，并生成满足视场与安全距离约束的视点。
5. 计算视点之间的路径、速度方向和航向转换代价。
6. 使用 LKH 求解访问次序，并进行去中心化任务分配。
7. 使用几何 A*、Kinodynamic A* 或拓扑 PRM 搜索路径。
8. 生成 B 样条并优化平滑度、障碍距离、速度和加速度约束。
9. 通过分层航向图优化信息增益和航向角速度。
10. 轨迹服务器持续输出位置、速度、加速度、航向和航向角速度。
11. Isaac 适配器将 RACER 指令转换为标准 Pose、Twist 和 Accel 消息。

详细的 C++ 到 Python 文件映射见
[CPP_PYTHON_MAPPING.md](docs/CPP_PYTHON_MAPPING.md)。

## 3. 获取迁移分支

```bash
git clone --branch migration/ros2-python-isaac \
  https://github.com/Kong-huihui/modified_RACER.git
cd modified_RACER
```

如果已经克隆了仓库：

```bash
git fetch origin
git switch migration/ros2-python-isaac
git pull --ff-only
```

## 4. 安装 Ubuntu 与 ROS 2 依赖

本节命令应在 Ubuntu 22.04 上执行。

首先安装 ROS 2 Humble Desktop。完成 ROS 官方安装后，安装工作区依赖：

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-numpy \
  python3-scipy \
  python3-rosdep \
  ros-humble-desktop \
  ros-humble-sensor-msgs-py
```

初始化 rosdep。若当前系统已经初始化，可跳过 `rosdep init`：

```bash
sudo rosdep init
rosdep update
```

每个新终端都需要加载 ROS 2：

```bash
source /opt/ros/humble/setup.bash
```

也可以写入 `~/.bashrc`：

```bash
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
```

## 5. 安装 LKH

RACER 使用 LKH 求解 TSP/多 TSP。为了保持与原系统一致，应安装 LKH，并
确保可执行文件名为 `LKH` 且位于 `PATH`：

```bash
command -v LKH
```

如果未找到 LKH，Python 代码会使用确定性的最近邻后备算法，以便完成接口
联调；该后备路径不代表与上游 RACER 数值等价。进行性能或闭环对比时必须
安装 LKH。

## 6. 构建 ROS 2 工作区

在仓库根目录执行：

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash

rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

以后每次打开新终端：

```bash
cd modified_RACER/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

开发阶段可以只构建指定软件包：

```bash
colcon build --symlink-install \
  --packages-select racer_interfaces racer_core racer_ros racer_bringup racer_isaac
```

## 7. 配置 Isaac Sim 5.1

### 7.1 通信原则

- Isaac Sim 使用自己的 Python 环境。
- 不要在 Isaac Sim 的 Python 中安装本工作区的 ROS 2 Python 包。
- 不要为了运行 Isaac 而修改系统 Python 3.10。
- Isaac 与外部 ROS 2 节点只通过 ROS 2 Bridge 通信。
- Isaac 和 ROS 2 终端必须使用相同的 `ROS_DOMAIN_ID`。

ROS 2 终端示例：

```bash
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source modified_RACER/ros2_ws/install/setup.bash
```

Linux 下若 Isaac 5.1 使用随 ROS 2 Bridge 扩展附带的 Humble 库，应在启动
Isaac 前设置下面的环境变量。将 `ISAAC_SIM_PATH` 换成实际安装目录，并且
不要在这个终端 `source /opt/ros/humble/setup.bash`，否则会把 Python 3.10
的系统 `rclpy` 混入 Isaac 的 Python 3.11：

```bash
export ISAAC_SIM_PATH=/home/user/isaacsim
export ROS_DISTRO=humble
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export LD_LIBRARY_PATH="${LD_LIBRARY_PATH}:${ISAAC_SIM_PATH}/exts/isaacsim.ros2.bridge/humble/lib"
"${ISAAC_SIM_PATH}/isaac-sim.sh"
```

### 7.2 创建 Isaac Bridge 图

1. 在 Isaac Sim 中启用 ROS 2 Bridge。
2. 为每架无人机准备一个 UAV prim 和一个激光雷达或深度传感器 prim。
3. 打开 Isaac Sim Script Editor。
4. 加载：

   ```text
   ros2_ws/src/racer_isaac/isaac/racer_bridge_graph.py
   ```

5. 修改脚本中的 `DRONES`，使 `base_prim` 和 `lidar_prim` 与实际 Stage 路径一致：

   ```python
   DRONES = [
       DroneBridge(1, "/World/drone_1", "/World/drone_1/Lidar"),
       DroneBridge(2, "/World/drone_2", "/World/drone_2/Lidar"),
       DroneBridge(3, "/World/drone_3", "/World/drone_3/Lidar"),
   ]
   ```

6. 在 Isaac Sim 内运行脚本。
7. 默认回调会把 `Twist` 的世界系线速度和角速度施加到 `base_prim` 的
   `RigidBodyAPI`；动力学四旋翼应以资产自己的控制器替换这个回调。

最后一步与具体无人机资产相关：Crazyflie、四旋翼 articulation、Isaac Lab
管理器任务和运动学测试模型的执行器接口并不相同。此适配只位于仿真边界，
不改变 RACER 的规划逻辑。

### 7.3 坐标系要求

RACER 地图默认使用 `world` 坐标系。输入点云必须已经转换到 `world`，否则
应在 Isaac Bridge 图或传感器发布链中完成坐标变换。

默认 TF 约定：

```text
world
└── drone_<id>/base_link
    └── drone_<id>/lidar
```

完整接口配置位于
`ros2_ws/src/racer_bringup/config/isaac_topics.yaml`。

## 8. 运行单无人机探索

### 8.1 启动 Isaac Sim

先打开配置好的 Stage，启用 ROS 2 Bridge，然后开始播放仿真。

### 8.2 启动 RACER

打开新的 Ubuntu 终端：

```bash
cd modified_RACER/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch racer_bringup single_drone.launch.py \
  drone_id:=1 \
  namespace:=drone_1 \
  use_sim_time:=true
```

启动文件会创建：

- `/drone_1/exploration_node`
- `/drone_1/trajectory_server`
- `/drone_1/isaac_command_adapter`

### 8.3 启动 RViz2

```bash
ros2 launch racer_bringup rviz.launch.py
```

RViz2 默认显示：

- 世界坐标网格；
- TF；
- `/drone_1/pointcloud`；
- `/drone_1/planning/markers`；
- RACER 路径和前沿视点。

### 8.4 触发探索

只有在里程计和点云都到达后，状态机才会进入等待触发状态。

可以使用 RViz2 的 “2D Goal Pose” 工具，也可以命令行发送：

```bash
ros2 topic pub --once \
  /move_base_simple/goal \
  geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}"
```

触发消息只负责启动探索；RACER 会自行选择前沿目标，不使用消息中的位置作为
最终规划目标。

## 9. 运行多无人机探索

确保 Isaac Stage 中已经为每架无人机创建对应的 Bridge 图和传感器话题，然后：

```bash
cd modified_RACER/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0

ros2 launch racer_bringup swarm_exploration.launch.py \
  drone_count:=3 \
  use_sim_time:=true
```

多机启动文件使用 1 开始的无人机编号：

```text
/drone_1
/drone_2
/drone_3
```

分别触发各无人机：

```bash
ros2 topic pub --once /drone_1/trigger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}"

ros2 topic pub --once /drone_2/trigger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}"

ros2 topic pub --once /drone_3/trigger geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}"
```

每架无人机独立运行地图、任务分配、规划和轨迹执行节点。共享数据通过以下
全局话题异步交换，不存在中央规划器：

- `/swarm_expl/drone_state`
- `/planning/swarm_traj`
- `/multi_map_manager/chunk_data`
- `/multi_map_manager/chunk_stamps`

## 10. ROS 2 话题接口

### 10.1 每架无人机的局部话题

以下名称位于 `/drone_<id>` 命名空间：

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `odometry` | `nav_msgs/msg/Odometry` | Isaac → RACER | 无人机状态 |
| `pointcloud` | `sensor_msgs/msg/PointCloud2` | Isaac → RACER | 世界坐标系点云 |
| `planning/bspline` | `racer_interfaces/msg/Bspline` | 规划器 → 轨迹服务器 | B 样条轨迹 |
| `position_cmd` | `racer_interfaces/msg/PositionCommand` | 轨迹服务器 → 适配器 | 完整状态指令 |
| `isaac/velocity_command` | `geometry_msgs/msg/TwistStamped` | RACER → Isaac | 速度与航向角速度 |
| `isaac/acceleration_command` | `geometry_msgs/msg/AccelStamped` | RACER → Isaac | 控制加速度 |
| `isaac/pose_command` | `geometry_msgs/msg/PoseStamped` | RACER → Isaac | 目标位姿 |
| `planning/markers` | `visualization_msgs/msg/MarkerArray` | RACER → RViz2 | 路径与前沿 |

传感器输入采用 ROS 2 sensor-data QoS；轨迹、命令和协同消息采用可靠通信。

### 10.2 检查话题

```bash
ros2 topic list
ros2 topic hz /drone_1/odometry
ros2 topic hz /drone_1/pointcloud
ros2 topic echo /drone_1/position_cmd
ros2 topic echo /swarm_expl/drone_state
```

完整话题表见 [TOPIC_CONTRACT.md](docs/TOPIC_CONTRACT.md)。

## 11. 参数配置

主配置文件：

```text
ros2_ws/src/racer_bringup/config/racer.yaml
```

主要参数组：

| 参数组 | 作用 |
| --- | --- |
| `fsm.*` | 重规划周期、同步周期和状态机频率 |
| `map.*` | 地图大小、分辨率、概率更新、膨胀和边界 |
| `multi_map.*` | 多机地图分块大小 |
| `manager.*` | 速度、加速度、控制点距离和规划开关 |
| `frontier.*` | 前沿聚类、视点半径、可见数量和完成阈值 |
| `perception.*` | 相机视场角和最大感知距离 |
| `heading.*` | 航向候选、信息增益和角速度惩罚 |
| `optimization.*` | 平滑、距离、可行性权重及迭代上限 |

修改参数时应保持与原 ROS 1 参数含义一致。ROS 1 到 ROS 2 的名称映射见
[ROS1_ROS2_MAPPING.md](docs/ROS1_ROS2_MAPPING.md)。

临时覆盖参数示例：

```bash
ros2 run racer_ros exploration_node --ros-args \
  -p drone_id:=1 \
  -p map.resolution:=0.15 \
  -p manager.max_velocity:=1.5
```

## 12. Python 开发说明

### 12.1 核心算法与 ROS 分离

`racer_core` 不导入 `rclpy` 或任何 ROS 消息。算法输入输出使用 NumPy 数组和
Python dataclass。ROS 消息只允许出现在 `racer_ros` 中。

这种边界用于：

- 独立比较 C++ 和 Python 数值结果；
- 避免 ROS 回调改变算法实现；
- 允许未来在记录数据上离线回放；
- 避免 Isaac Sim Python 与 ROS Python 混用。

### 12.2 逻辑迁移约束

继续开发时应遵守：

1. 不随意改变上游状态机转换条件。
2. 不改变前沿、视点、路径和轨迹代价项的物理含义。
3. 不删除速度、加速度、航向角速度、碰撞距离和地图边界约束。
4. ROS 1 参数重命名时同步更新映射文档。
5. 仿真资产差异只能在 `racer_isaac` 或命令适配器处理。
6. 算法改变必须与语言/通信迁移分开提交，避免无法进行基线对比。

### 12.3 自定义消息字段

ROS 2 要求字段名为小写。以下字段只进行了语法必要的重命名：

| ROS 1 | ROS 2 |
| --- | --- |
| `Kp` / `Kd` | `kp` / `kd` |
| `Kp_yaw` / `Kd_yaw` | `kp_yaw` / `kd_yaw` |
| `kR` / `kOm` | `kr` / `kom` |
| `voxel_occ_` | `voxel_occ` |

字段类型和算法含义不变。

## 13. 验证建议

虽然本次迁移按要求未在本机验证，但在合并到 `main` 前建议依次完成：

### 13.1 构建与接口

```bash
colcon build --symlink-install
source install/setup.bash
ros2 interface show racer_interfaces/msg/Bspline
ros2 interface show racer_interfaces/msg/PositionCommand
```

### 13.2 单元与数值对比

使用相同输入分别运行 ROS 1/C++ 和 ROS 2/Python，比较：

- 射线经过的体素索引；
- 占用概率和 ESDF；
- 前沿单元、聚类和视点；
- A* 路径长度；
- TSP 访问次序；
- B 样条控制点和采样状态；
- 航向序列；
- 速度、加速度及最小障碍距离。

### 13.3 Isaac 单机闭环

确认：

- `/clock` 单调更新；
- 里程计和点云时间戳使用仿真时间；
- RACER 指令能够到达 UAV 控制器；
- UAV 实际状态能够跟踪 `PositionCommand`；
- 重规划不会造成轨迹跳变；
- 无碰撞、越界、速度或加速度超限。

### 13.4 多机闭环

至少验证：

- 三架 UAV 异步启动；
- 地图分块丢失后的索引检测与重发；
- 通信延迟和短时中断；
- 前沿任务没有长期重复分配；
- 共享轨迹参与碰撞约束；
- 所有无人机完成探索后正常停止。

## 14. 常见问题

### 14.1 `colcon` 尝试构建 ROS 1 包

确认以下文件存在：

```text
swarm_exploration/COLCON_IGNORE
uav_simulator/COLCON_IGNORE
```

并从 `ros2_ws` 目录运行 `colcon build`。

### 14.2 找不到 `racer_interfaces`

```bash
cd modified_RACER/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racer_interfaces
source install/setup.bash
```

### 14.3 节点一直等待触发

先检查传感器：

```bash
ros2 topic hz /drone_1/odometry
ros2 topic hz /drone_1/pointcloud
```

没有里程计或有效点云时，探索状态机不会启动。

### 14.4 ROS 2 与 Isaac 看不到彼此

检查：

- 两边 `ROS_DOMAIN_ID` 是否一致；
- Isaac ROS 2 Bridge 是否启用；
- 仿真是否处于播放状态；
- Bridge 图中的话题名和命名空间是否正确；
- 防火墙和 DDS 网络接口设置；
- ROS 2 节点是否设置 `use_sim_time:=true`。

### 14.5 有轨迹但无人机不运动

检查 `/drone_1/position_cmd` 和 `/drone_1/isaac/velocity_command`。如果都有
数据，说明 RACER 与外部适配器已经工作，需要检查 Isaac 图中
`SubscribeVelocity` 到具体 UAV 控制器的连接。

### 14.6 点云存在但没有前沿

检查：

- 点云是否在 `world` 坐标系；
- `map.box_min` 和 `map.box_max` 是否覆盖场景；
- `map.ground_height` 是否正确；
- 点云距离是否超过 `map.max_ray_length`；
- `frontier.cluster_min` 与地图分辨率是否匹配。

## 15. 仓库目录

```text
modified_RACER/
├── docs/                       # 架构、迁移、接口和验证文档
├── ros2_ws/
│   └── src/
│       ├── racer_interfaces/   # ROS 2 消息与服务
│       ├── racer_core/         # Python 核心算法
│       ├── racer_ros/          # rclpy 节点
│       ├── racer_bringup/      # launch、参数与 RViz2
│       └── racer_isaac/        # Isaac Sim/Lab 接入
├── swarm_exploration/          # 上游 ROS 1/C++ 参考代码
├── uav_simulator/              # 上游仿真器参考代码
└── README.md
```

## 16. 迁移文档

- [ROS 2 架构](docs/ARCHITECTURE_ROS2.md)
- [构建与运行](docs/BUILD_AND_RUN.md)
- [Isaac Sim/Isaac Lab 接入](docs/ISAAC_SIM.md)
- [ROS 1 到 ROS 2 映射](docs/ROS1_ROS2_MAPPING.md)
- [C++ 到 Python 映射](docs/CPP_PYTHON_MAPPING.md)
- [话题契约](docs/TOPIC_CONTRACT.md)
- [迁移计划](docs/MIGRATION_PLAN.md)
- [迁移日志](docs/MIGRATION_LOG.md)
- [验证状态](docs/VALIDATION_STATUS.md)

## 17. 上游项目与引用

本项目基于 Robotics-STAR-Lab 的 RACER：

- 上游仓库：<https://github.com/Robotics-STAR-Lab/RACER>
- 论文：*RACER: Rapid Collaborative Exploration with a Decentralized
  Multi-UAV System*

如果在研究中使用 RACER，请引用原论文：

```bibtex
@article{zhou2023racer,
  title={Racer: Rapid collaborative exploration with a decentralized multi-uav system},
  author={Zhou, Boyu and Xu, Hao and Shen, Shaojie},
  journal={IEEE Transactions on Robotics},
  year={2023},
  publisher={IEEE}
}
```

迁移版本不会替代上游作者的成果、署名或引用要求。涉及许可与发布时，请同时
检查上游仓库的最新说明。
