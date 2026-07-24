# C++ to Python mapping

The active Python modules are under `ros2_ws/src`. The original files remain
read-only references and are excluded from colcon.

| Original C++ area | Python destination |
| --- | --- |
| `bspline/non_uniform_bspline.*` | `racer_core/bspline.py` |
| `poly_traj/*` | `racer_core/polynomial.py` |
| `plan_env/raycast.*` | `racer_core/raycast.py` |
| `plan_env/sdf_map.*` | `racer_core/voxel_map.py` |
| `plan_env/edt_environment.*` | `racer_core/environment.py` |
| `path_searching/astar*` | `racer_core/search.py` |
| `path_searching/kinodynamic_astar.*` | `racer_core/kinodynamic.py` |
| `path_searching/topo_prm.*` | `racer_core/topology.py` |
| `active_perception/perception_utils.*` | `racer_core/perception.py` |
| `active_perception/frontier_finder.*` | `racer_core/frontier.py` |
| `active_perception/heading_planner.*` | `racer_core/heading.py` |
| `active_perception/traj_visibility.*` | `racer_core/visibility.py` |
| `active_perception/hgrid.*`, `uniform_grid.*` | `racer_core/partition.py` |
| `bspline_opt/bspline_optimizer.*` | `racer_core/optimizer.py` |
| `plan_manage/planner_manager.*` | `racer_core/planner.py` |
| `exploration_manager/fast_exploration_manager.*` | `racer_core/planner.py` |
| `exploration_manager/fast_exploration_fsm.*` | `racer_ros/exploration_node.py` |
| `plan_env/multi_map_manager.*` | `racer_core/multi_map.py`, `racer_ros/exploration_node.py` |
| LKH ROS wrappers | `racer_core/tsp.py`, `racer_ros/lkh_service.py` |
| `plan_manage/traj_server.*` | `racer_ros/trajectory_server.py` |
| `so3_control/*` | `racer_core/controller.py`, `racer_ros/isaac_command_adapter.py` |
| custom simulator | replaced by `racer_isaac` Bridge contract |

Generated ROS 1 message sources, catkin build output, bundled Boost.Odeint, and
the bundled LKH implementation are not application logic. ROS 2 regenerates
messages, Isaac Sim replaces the custom simulator, and Python invokes the same
external LKH executable and parameter files used by the original runtime.
