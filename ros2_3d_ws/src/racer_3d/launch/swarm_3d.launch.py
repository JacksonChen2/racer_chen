"""Launch three RACER 3-D agents, optional mock plant, and acceptance monitor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from racer_3d.scenario import get_scenario


def _setup(context):
    share = Path(get_package_share_directory("racer_3d"))
    config = str(share / "config" / "racer_3d.yaml")
    backend = LaunchConfiguration("backend").perform(context)
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    duration = float(LaunchConfiguration("duration").perform(context))
    result_file = LaunchConfiguration("result_file").perform(context)
    scenario = get_scenario(
        LaunchConfiguration("scenario").perform(context)
    )
    scenario_parameters = {
        "scenario_name": scenario.name,
        "map_origin": list(scenario.map_min),
        "map_size": list(scenario.map_size),
        "start_positions": [
            value for point in scenario.starts for value in point
        ],
        "coarse_grid_size": list(scenario.coarse_grid_size),
    }
    run_monitor = (
        LaunchConfiguration("run_monitor").perform(context).lower()
        in ("1", "true", "yes")
    )
    actions = []
    if backend == "mock":
        actions.append(
            Node(
                package="racer_3d",
                executable="racer_3d_mock_sim",
                name="racer_3d_mock_sim",
                output="screen",
                parameters=[
                    config,
                    scenario_parameters,
                    {"drone_count": drone_count},
                ],
            )
        )
    for drone_id in range(drone_count):
        actions.append(
            Node(
                package="racer_3d",
                executable="racer_3d_agent",
                name=f"racer_3d_agent_{drone_id}",
                output="screen",
                parameters=[
                    config,
                    scenario_parameters,
                    {"drone_id": drone_id, "drone_count": drone_count},
                ],
            )
        )
    if run_monitor:
        actions.append(
            Node(
                package="racer_3d",
                executable="racer_3d_monitor",
                name="racer_3d_monitor",
                output="screen",
                parameters=[
                    config,
                    scenario_parameters,
                    {
                        "drone_count": drone_count,
                        "duration": duration,
                        "result_file": result_file,
                        "require_physics_backend": backend == "isaac",
                    },
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("drone_count", default_value="3"),
            DeclareLaunchArgument(
                "scenario", default_value="acceptance_15x9x2"
            ),
            DeclareLaunchArgument("duration", default_value="120.0"),
            DeclareLaunchArgument("run_monitor", default_value="true"),
            DeclareLaunchArgument(
                "result_file", default_value="/tmp/racer_3d_result.json"
            ),
            OpaqueFunction(function=_setup),
        ]
    )
