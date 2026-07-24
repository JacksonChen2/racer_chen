from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _nodes(context):
    count = int(LaunchConfiguration("drone_count").perform(context))
    use_sim_time = LaunchConfiguration("use_sim_time")
    config = str(Path(get_package_share_directory("racer_bringup")) / "config" / "racer.yaml")
    actions = []
    for drone_id in range(1, count + 1):
        namespace = f"drone_{drone_id}"
        parameters = [
            config,
            {"drone_id": drone_id, "drone_count": count, "use_sim_time": use_sim_time},
        ]
        actions.extend([
            Node(
                package="racer_ros", executable="exploration_node", namespace=namespace,
                name="exploration_node", output="screen", parameters=parameters,
                remappings=[("trigger", f"/{namespace}/trigger")],
            ),
            Node(
                package="racer_ros", executable="trajectory_server", namespace=namespace,
                name="trajectory_server", output="screen", parameters=parameters,
            ),
            Node(
                package="racer_ros", executable="isaac_command_adapter", namespace=namespace,
                name="isaac_command_adapter", output="screen", parameters=parameters,
            ),
        ])
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("drone_count", default_value="3"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        OpaqueFunction(function=_nodes),
    ])

