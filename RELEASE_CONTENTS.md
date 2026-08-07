# Release contents

| Directory | Role | Warehouse/Isaac | Communication model |
|---|---|---|---|
| `original_racer_ros1_ws/src/RACER` | Upstream ROS 1 baseline | No | Original asynchronous ROS transport only |
| `ros2_3d_C_ws` | C++17 ROS 2 3-D reproduction | Yes | Direct ROS 2 topics |
| `ros2_3d_py_ws` | Python ROS 2 3-D reproduction | Yes | Direct ROS 2 topics |
| `ros2_3d_sionna_ws` | Communication-aware C++ ROS 2 reproduction | Yes | Sionna RT plus C++ queue/loss emulator |
| `isaac_assets` | RACER SO(3) vehicle USD/URDF/configuration | Yes | N/A |

Generated build products, caches, virtual environments and run logs are
intentionally absent. Run `./racer doctor`, then use the bundled launcher.
