# Isaac Sim 5.1 and Isaac Lab connection

## Separation of Python environments

Run ROS 2 Humble nodes in the Ubuntu shell and run Isaac scripts only in Isaac
Sim's bundled Python. Do not source Humble inside Isaac Sim. The ROS 2 Bridge
performs DDS communication between the processes.

## Stage setup

1. Enable the Isaac Sim ROS 2 Bridge and set the same `ROS_DOMAIN_ID` used by
   the Humble shell.
2. Add one vehicle prim and one lidar/depth sensor prim per UAV.
3. Copy or open the installed
   `share/racer_isaac/isaac/racer_bridge_graph.py` in Isaac's Script Editor.
4. Set each `DroneBridge` entry to the stage's base and lidar prim paths, then
   run the script.
5. Connect `SubscribeVelocity` outputs in each generated action graph to the
   selected vehicle/controller asset. This last connection is necessarily
   asset-specific: a Crazyflie rotor controller, a custom multirotor
   articulation and a kinematic test body have different inputs.
6. Ensure the point cloud is expressed in `world`, or provide the matching TF
   transform before RACER consumes it.

The graph publishes `/clock`, `/drone_<id>/odometry` and
`/drone_<id>/pointcloud`; it subscribes to
`/drone_<id>/isaac/velocity_command`. RACER also publishes pose and
acceleration commands for controllers that use those forms. The full contract
is in `racer_bringup/config/isaac_topics.yaml`.

## Isaac Lab

`isaac_lab_command_term.py` converts Bridge Twist output into the
`(vx, vy, vz, yaw_rate)` tensor commonly consumed by a manager-based command
term. Import it from Isaac's Python environment, attach the tensor to the
chosen vehicle action term, and keep ROS imports out of the Lab task.

The migration deliberately does not hard-code an aircraft asset or rotor
layout because those are not present in the upstream repository. Selecting an
Isaac asset changes only this adapter boundary, not RACER planning logic.

