# Build and run on the target machine

These commands are instructions for the required Ubuntu target; they were not
executed in the Windows checkout.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble Desktop and `ros-dev-tools`
- Python packages supplied by Ubuntu: NumPy and SciPy
- LKH available as `LKH` on `PATH` for upstream solver parity
- Isaac Sim 5.1 with ROS 2 Bridge enabled

```bash
sudo apt update
sudo apt install python3-numpy python3-scipy python3-colcon-common-extensions \
  ros-humble-desktop ros-humble-sensor-msgs-py

cd modified_RACER/ros2_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## Single UAV

```bash
ros2 launch racer_bringup single_drone.launch.py \
  drone_id:=1 namespace:=drone_1 use_sim_time:=true
```

Start RViz2 separately:

```bash
ros2 launch racer_bringup rviz.launch.py
```

After odometry and point cloud data are present, send a trigger:

```bash
ros2 topic pub --once /move_base_simple/goal geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: world}, pose: {orientation: {w: 1.0}}}"
```

## Swarm

```bash
ros2 launch racer_bringup swarm_exploration.launch.py \
  drone_count:=3 use_sim_time:=true
```

The corresponding Isaac stage must expose `/drone_1`, `/drone_2`, and
`/drone_3` topic sets. Trigger each planner on `/drone_<id>/trigger`.

## LKH

`racer_core.LkhSolver` invokes the same external LKH executable used upstream.
If `LKH` is absent it uses a deterministic nearest-neighbour fallback, which
is useful for integration but is not numerical parity. Install LKH before
closed-loop equivalence testing.

