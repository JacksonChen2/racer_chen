"""Launch a RACER fleet with either mock sensing or the Isaac adapter."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from racer_ros2.scenario import DRONE_RADIUS, get_scenario


def _launch_setup(context):
    package_share = Path(get_package_share_directory("racer_ros2"))
    config = str(package_share / "config" / "racer.yaml")
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    backend = LaunchConfiguration("backend").perform(context)
    scenario_name = LaunchConfiguration("scenario").perform(context)
    scenario = get_scenario(scenario_name)
    duration = float(LaunchConfiguration("duration").perform(context))
    result_file = LaunchConfiguration("result_file").perform(context)
    minimum_coverage = float(
        LaunchConfiguration("minimum_coverage").perform(context)
    )
    run_monitor = (
        LaunchConfiguration("run_monitor").perform(context).lower()
        in ("true", "1", "yes")
    )
    actions = []
    flattened_starts = [
        coordinate
        for point in scenario.starts
        for coordinate in point
    ]
    scene_parameters = {
        "start_positions": flattened_starts,
        "map_origin": list(scenario.map_min),
        "map_size": list(scenario.map_size),
        "coarse_grid_size": 2.0 if scenario_name == "small" else 5.0,
        "planning_clearance": (
            0.60 if scenario_name == "long" else
            0.50 if scenario_name == "small" else 0.45
        ),
        "swarm_safe_distance": 1.20 if scenario_name == "long" else 0.70,
        "emergency_distance": 1.65 if scenario_name == "long" else 0.95,
        "robot_radius": DRONE_RADIUS,
        "obstacle_braking_margin": (
            0.45 if scenario_name == "long" else 0.65
        ),
        "flight_z": scenario.flight_z,
        "max_speed": (
            0.75 if scenario_name in ("small", "long") else 0.90
        ),
        # A single 6 m scan sees most of the 8 x 6 m scene. Keep the small
        # integration case active long enough to verify multi-UAV motion.
        "completion_coverage": 0.995 if scenario_name == "small" else 0.98,
    }
    if backend == "mock":
        actions.append(
            Node(
                package="racer_ros2",
                executable="racer_mock_sim",
                name="racer_mock_sim",
                output="screen",
                parameters=[
                    config,
                    scene_parameters,
                    {
                        "drone_count": drone_count,
                        "scenario": scenario_name,
                    },
                ],
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
                        **scene_parameters,
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
                        "minimum_coverage": minimum_coverage,
                        "minimum_inter_drone": 0.50,
                        "minimum_obstacle_clearance": 0.05,
                        "require_physics_backend": backend == "isaac",
                        "scenario": scenario_name,
                    }
                ],
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="mock"),
            DeclareLaunchArgument("scenario", default_value="small"),
            DeclareLaunchArgument("drone_count", default_value="3"),
            DeclareLaunchArgument("duration", default_value="45.0"),
            DeclareLaunchArgument("minimum_coverage", default_value="0.70"),
            DeclareLaunchArgument(
                "result_file", default_value="/tmp/racer_result.json"
            ),
            DeclareLaunchArgument("run_monitor", default_value="true"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
