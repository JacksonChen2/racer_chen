"""Launch a RACER fleet with either mock sensing or the Isaac adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context):
    package_share = Path(get_package_share_directory("racer_ros2"))
    config = str(package_share / "config" / "racer.yaml")
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    backend = LaunchConfiguration("backend").perform(context)
    duration = float(LaunchConfiguration("duration").perform(context))
    result_file = LaunchConfiguration("result_file").perform(context)
    run_monitor = (
        LaunchConfiguration("run_monitor").perform(context).lower()
        in ("true", "1", "yes")
    )
    actions = []
    if backend == "mock":
        actions.append(
            Node(
                package="racer_ros2",
                executable="racer_mock_sim",
                name="racer_mock_sim",
                output="screen",
                parameters=[config, {"drone_count": drone_count}],
            )
        )
    for drone_id in range(drone_count):
        actions.append(
            Node(
                package="racer_ros2",
                executable="racer_agent",
                name=f"racer_agent_{drone_id}",
                output="screen",
                parameters=[
                    config,
                    {
                        "drone_id": drone_id,
                        "drone_count": drone_count,
                    },
                ],
            )
        )
    if run_monitor:
        actions.append(
            Node(
                package="racer_ros2",
                executable="racer_monitor",
                name="racer_monitor",
                output="screen",
                parameters=[
                    {
                        "drone_count": drone_count,
                        "duration": duration,
                        "result_file": result_file,
                        "minimum_coverage": 0.55,
                        "minimum_inter_drone": 1.00,
                        "minimum_obstacle_clearance": 0.10,
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("drone_count", default_value="3"),
            DeclareLaunchArgument("duration", default_value="45.0"),
            DeclareLaunchArgument(
                "result_file", default_value="/tmp/racer_result.json"
            ),
            DeclareLaunchArgument("run_monitor", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
