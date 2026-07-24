from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory("racer_bringup")) / "config" / "racer.yaml")
    drone_id = LaunchConfiguration("drone_id")
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    common = [config, {"drone_id": drone_id, "use_sim_time": use_sim_time}]
    return LaunchDescription([
        DeclareLaunchArgument("drone_id", default_value="1"),
        DeclareLaunchArgument("namespace", default_value="drone_1"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="racer_ros", executable="exploration_node", namespace=namespace,
            name="exploration_node", output="screen", parameters=common,
            remappings=[("trigger", "/move_base_simple/goal")],
        ),
        Node(
            package="racer_ros", executable="trajectory_server", namespace=namespace,
            name="trajectory_server", output="screen", parameters=common,
        ),
        Node(
            package="racer_ros", executable="isaac_command_adapter", namespace=namespace,
            name="isaac_command_adapter", output="screen", parameters=common,
        ),
    ])

