# ROS 2 topic contract

| Topic (per `/drone_<id>`) | Type | Direction | QoS |
| --- | --- | --- | --- |
| `odometry` | `nav_msgs/Odometry` | Isaac → RACER | sensor data |
| `pointcloud` | `sensor_msgs/PointCloud2` | Isaac → RACER | sensor data |
| `planning/bspline` | `racer_interfaces/Bspline` | planner → server | reliable |
| `position_cmd` | `racer_interfaces/PositionCommand` | server → adapter | reliable |
| `isaac/velocity_command` | `geometry_msgs/TwistStamped` | adapter → Isaac | reliable |
| `isaac/acceleration_command` | `geometry_msgs/AccelStamped` | adapter → Isaac | reliable |
| `isaac/pose_command` | `geometry_msgs/PoseStamped` | adapter → Isaac | reliable |
| `planning/markers` | `visualization_msgs/MarkerArray` | RACER → RViz2 | reliable |

| Shared topic | Type | Purpose |
| --- | --- | --- |
| `/clock` | `rosgraph_msgs/Clock` | simulator time |
| `/planning/swarm_traj` | `racer_interfaces/Bspline` | decentralized trajectory exchange |
| `/swarm_expl/drone_state` | `racer_interfaces/DroneState` | team state |
| `/multi_map_manager/chunk_data` | `racer_interfaces/ChunkData` | map delta exchange |
| `/multi_map_manager/chunk_stamps` | `racer_interfaces/ChunkStamps` | missing-chunk detection |

