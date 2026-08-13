"""Launch C++ RACER agents with the C++ mock backend or Isaac Sim."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


SCENARIOS = {
    "acceptance_15x9x2": {
        "map_origin": [-7.5, -4.5, 0.0],
        "map_size": [15.0, 9.0, 2.0],
        "starts": [-6.4, -3.2, 0.45, -6.4, 0.0, 1.0, -6.4, 3.2, 1.55],
        "coarse": [5.0, 4.5, 2.0],
        "truth_mode": "analytic_boxes",
    },
    "acceptance_20x50x3": {
        "map_origin": [-10.0, -25.0, 0.0],
        "map_size": [20.0, 50.0, 3.0],
        "starts": [-6.0, -23.0, 0.6, 0.0, -23.0, 1.5, 6.0, -23.0, 2.4],
        "coarse": [5.0, 10.0, 3.0],
        "truth_mode": "analytic_boxes",
    },
    "warehouse_simple": {
        "map_origin": [-10.2, -12.0, 0.0],
        "map_size": [19.4, 29.8, 9.0],
        "starts": [
            -6.0, -10.0, 0.6, 0.0, -10.0, 1.5, 6.0, -10.0, 2.4,
            -3.0, -10.0, 1.05, 3.0, -10.0, 1.95,
        ],
        "coarse": [4.85, 7.45, 4.5],
        "truth_mode": "observed_volume",
    },
    "warehouse_loaded": {
        "map_origin": [-26.8, 7.2, 0.0],
        "map_size": [32.6, 19.0, 8.5],
        "starts": [
            -24.0, 8.0, 0.8,
            -10.5, 8.0, 1.5,
            3.0, 8.0, 2.2,
        ],
        "coarse": [8.15, 4.75, 4.25],
        "truth_mode": "observed_volume",
    },
}


def _setup(context):
    share = Path(get_package_share_directory("racer_3d_cpp"))
    config = str(share / "config" / "racer_3d_cpp.yaml")
    backend = LaunchConfiguration("backend").perform(context)
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    duration = float(LaunchConfiguration("duration").perform(context))
    if backend not in ("mock", "isaac"):
        raise RuntimeError("backend must be 'mock' or 'isaac'")
    if drone_count <= 0:
        raise RuntimeError("drone_count must be positive")
    if duration <= 0.0:
        raise RuntimeError("duration must be positive")
    scenario_name = LaunchConfiguration("scenario").perform(context)
    if scenario_name not in SCENARIOS:
        raise RuntimeError(
            f"unknown scenario {scenario_name!r}; choose {sorted(SCENARIOS)}"
        )
    scenario = SCENARIOS[scenario_name]
    if len(scenario["starts"]) < 3 * drone_count:
        raise RuntimeError(
            f"{scenario_name} defines fewer than {drone_count} launch poses"
        )
    common = {
        "scenario_name": scenario_name,
        "map_origin": scenario["map_origin"],
        "map_size": scenario["map_size"],
        "start_positions": scenario["starts"],
        "coarse_grid_size": scenario["coarse"],
    }
    vehicle_model = LaunchConfiguration("vehicle_model").perform(context)
    if vehicle_model not in ("racer_so3", "crazyflie"):
        raise RuntimeError(
            "vehicle_model must be 'racer_so3' or 'crazyflie'"
        )
    if vehicle_model == "racer_so3":
        # The generated RACER model has a 0.644 m visual rotor span and a
        # 0.568 m collision span, substantially larger than Crazyflie.
        common.update({
            # Keep geometric margin beyond the 0.284 m collision radius and
            # reserve enough distance for the 0.98 kg SO3 plant to brake.
            "planning_clearance": 0.38,
            "control_clearance": 0.52,
            "swarm_safe_distance": 1.20,
            "emergency_distance": 1.35,
            "max_speed": 0.85,
            "max_acceleration": 0.80,
            "guaranteed_deceleration": 0.60,
            "safety_response_time": 0.20,
        })
    else:
        # Preserve the pre-migration Crazyflie comparison profile.
        common.update({
            "map_resolution": 0.20,
            "minimum_sensor_range": 0.0,
            "lidar_range": 7.0,
            "maximum_sensor_rays": 1200,
            "planning_clearance": 0.22,
            "control_clearance": 0.45,
            "swarm_safe_distance": 0.65,
            "emergency_distance": 0.80,
            "max_speed": 0.35,
            "max_acceleration": 1.40,
            "planning_period": 1.0,
            "pairwise_period": 3.0,
        })
    run_monitor = LaunchConfiguration("run_monitor").perform(context).lower() in (
        "1", "true", "yes",
    )
    actions = []
    if backend == "mock":
        actions.append(Node(
            package="racer_3d_cpp",
            executable="racer_3d_cpp_mock_sim",
            name="racer_3d_cpp_mock_sim",
            output="screen",
            parameters=[config, common, {"drone_count": drone_count}],
        ))
    for drone_id in range(drone_count):
        actions.append(Node(
            package="racer_3d_cpp",
            executable="racer_3d_cpp_agent",
            name=f"racer_3d_cpp_agent_{drone_id}",
            output="screen",
            parameters=[
                config, common,
                {
                    "drone_id": drone_id,
                    "drone_count": drone_count,
                    # Trajectory timestamps and peer expiry must follow
                    # Isaac physics time when the 200 Hz SO3 plant runs
                    # slower than real time.
                    "use_sim_time": backend == "isaac",
                },
            ],
        ))
    if run_monitor:
        monitor = Node(
            package="racer_3d_cpp",
            executable="racer_3d_cpp_monitor",
            name="racer_3d_cpp_monitor",
            output="screen",
            parameters=[
                config, common,
                {
                    "drone_count": drone_count,
                    "duration": duration,
                    "result_file": LaunchConfiguration("result_file").perform(context),
                    "truth_mode": scenario["truth_mode"],
                    "require_physics_backend": backend == "isaac",
                    "minimum_inter_drone": (
                        0.60 if vehicle_model == "racer_so3" else 0.35
                    ),
                },
            ],
        )
        actions.append(monitor)
        actions.append(RegisterEventHandler(
            OnProcessExit(
                target_action=monitor,
                on_exit=[
                    EmitEvent(event=Shutdown(reason="acceptance monitor exited"))
                ],
            )
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("backend", default_value="mock"),
        DeclareLaunchArgument("drone_count", default_value="3"),
        DeclareLaunchArgument("scenario", default_value="acceptance_15x9x2"),
        DeclareLaunchArgument("vehicle_model", default_value="racer_so3"),
        DeclareLaunchArgument("duration", default_value="120.0"),
        DeclareLaunchArgument("run_monitor", default_value="true"),
        DeclareLaunchArgument(
            "result_file", default_value="/tmp/racer_3d_cpp_result.json"
        ),
        OpaqueFunction(function=_setup),
    ])
