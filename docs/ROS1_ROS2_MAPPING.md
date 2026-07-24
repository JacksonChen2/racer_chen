# ROS 1 to ROS 2 mapping

## Runtime mapping

| ROS 1 component | ROS 2 component |
| --- | --- |
| `exploration_manager/exploration_node` | `racer_ros/exploration_node` |
| `plan_manage/traj_server` | `racer_ros/trajectory_server` |
| `lkh_tsp_solver/tsp_node` | `racer_ros/lkh_service` |
| `lkh_mtsp_solver/mtsp_node` | `racer_ros/lkh_service` |
| `map_generator/map_pub` | Isaac Sim point cloud publisher |
| `local_sensing/pcl_render_node` | Isaac Sim RTX sensor through ROS 2 Bridge |
| `so3_control` nodelet | `racer_core/PositionController` |
| `poscmd_2_odom` | `racer_ros/isaac_command_adapter` |
| ROS 1 simulator | Isaac Sim 5.1 |
| RViz | RViz2 |

## Communication rules

- Per-UAV topics are relative names inside `/drone_<id>`.
- Swarm exchange topics remain absolute under `/swarm_expl`.
- Map exchange remains absolute under `/multi_map_manager`.
- Sensor topics use sensor-data QoS.
- Commands, trajectories, and swarm state use reliable volatile QoS.
- `/clock` is supplied by Isaac Sim and every node declares `use_sim_time`.

## Interface package mapping

The ROS 1 packages `bspline`, `plan_env`, `exploration_manager`,
`quadrotor_msgs`, `multi_map_server`, `lkh_tsp_solver`, and
`lkh_mtsp_solver` previously owned custom interfaces. ROS 2 keeps the same
fields in the consolidated `racer_interfaces` package to avoid mixing
interface generation with Python algorithm packages.

The duplicated ROS 1 `quadrotor_msgs` trees are represented once. For
`PositionCommand`, the superset definition containing trajectory status is
used; all fields consumed by RACER retain their names and types.

ROS 2 interface naming rules require lower-case field names. The following
wire-level renames are therefore intentional: `Kp/Kd/Kp_yaw/Kd_yaw` become
`kp/kd/kp_yaw/kd_yaw`, `kR/kOm` become `kr/kom`, and `voxel_occ_` becomes
`voxel_occ`. Their type and meaning are unchanged.
