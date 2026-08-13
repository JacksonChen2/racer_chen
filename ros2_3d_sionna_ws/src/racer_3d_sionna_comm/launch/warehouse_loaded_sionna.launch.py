"""Communication-aware C++ RACER in the warehouse_loaded Isaac scene."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


WAREHOUSE = {
    "map_origin": [-26.8, 7.2, 0.0],
    "map_size": [32.6, 19.0, 8.5],
    "starts": [-24.0, 8.0, 0.8, -10.5, 8.0, 1.5, 3.0, 8.0, 2.2],
    "coarse": [8.15, 4.75, 4.25],
}


def as_bool(value: str) -> bool:
    return value.lower() in ("1", "true", "yes", "on")


def setup(context):
    share = Path(get_package_share_directory("racer_3d_sionna_comm"))
    agent_config = str(share / "config" / "warehouse_loaded_agent.yaml")
    communication_config = str(
        share / "config" / "warehouse_loaded_communication.yaml"
    )
    default_scene = share / "assets" / "warehouse_loaded_sionna" / "warehouse.xml"
    default_cache = share / "assets" / "warehouse_loaded_sionna" / "hybrid_radio_cache.npz"

    backend = LaunchConfiguration("backend").perform(context)
    if backend not in ("mock", "isaac"):
        raise RuntimeError("backend must be mock or isaac")
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    if drone_count <= 0 or drone_count > len(WAREHOUSE["starts"]) // 3:
        raise RuntimeError("warehouse_loaded supports one to three drones")
    duration = float(LaunchConfiguration("duration").perform(context))
    use_sim_time = backend == "isaac"
    scene_xml = LaunchConfiguration("sionna_scene_xml").perform(context)
    if not scene_xml:
        scene_xml = str(default_scene)
    communication_mode = LaunchConfiguration("communication_mode").perform(context)
    radio_cache = LaunchConfiguration("radio_map_cache").perform(context)
    if communication_mode != "sionna_hybrid":
        radio_cache = ""
    elif not radio_cache and default_cache.is_file():
        radio_cache = str(default_cache)
    require_sionna = as_bool(
        LaunchConfiguration("require_sionna").perform(context)
    )
    random_seed = int(LaunchConfiguration("random_seed").perform(context))

    common = {
        "scenario_name": "warehouse_loaded",
        "map_origin": WAREHOUSE["map_origin"],
        "map_size": WAREHOUSE["map_size"],
        "start_positions": WAREHOUSE["starts"],
        "coarse_grid_size": WAREHOUSE["coarse"],
        "drone_count": drone_count,
        "use_sim_time": use_sim_time,
    }
    actions = []
    if backend == "mock":
        actions.append(
            Node(
                package="racer_3d_sionna_comm",
                executable="racer_3d_sionna_mock_sim",
                name="racer_3d_sionna_mock_sim",
                output="screen",
                parameters=[agent_config, common],
            )
        )

    actions.append(
        Node(
            package="racer_3d_sionna_comm",
            executable="sionna_channel_node",
            name="racer_3d_sionna_channel",
            output="screen",
            parameters=[
                communication_config,
                {
                    **common,
                    "scene_xml": scene_xml,
                    "radio_map_cache": radio_cache,
                    "require_sionna": require_sionna,
                    "allow_analytic_fallback": not require_sionna,
                    "random_seed": random_seed,
                },
            ],
        )
    )
    actions.append(
        Node(
            package="racer_3d_sionna_comm",
            executable="racer_3d_communication_emulator",
            name="racer_3d_communication_emulator",
            output="screen",
            parameters=[
                communication_config,
                {
                    "mode": communication_mode,
                    "drone_count": drone_count,
                    "random_seed": random_seed,
                    "use_sim_time": use_sim_time,
                },
            ],
        )
    )
    for drone_id in range(drone_count):
        actions.append(
            Node(
                package="racer_3d_sionna_comm",
                executable="racer_3d_sionna_agent",
                name=f"racer_3d_sionna_agent_{drone_id}",
                output="screen",
                parameters=[agent_config, common, {"drone_id": drone_id}],
            )
        )

    if as_bool(LaunchConfiguration("run_monitor").perform(context)):
        monitor = Node(
            package="racer_3d_sionna_comm",
            executable="racer_3d_sionna_monitor",
            name="racer_3d_sionna_monitor",
            output="screen",
            parameters=[
                agent_config,
                common,
                {
                    "duration": duration,
                    "result_file": LaunchConfiguration("result_file").perform(context),
                    "truth_mode": "observed_volume",
                    "require_physics_backend": backend == "isaac",
                    "minimum_inter_drone": 0.60,
                },
            ],
        )
        actions.append(monitor)
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=monitor,
                    on_exit=[EmitEvent(event=Shutdown(reason="monitor exited"))],
                )
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("backend", default_value="isaac"),
            DeclareLaunchArgument("drone_count", default_value="3"),
            DeclareLaunchArgument("duration", default_value="900.0"),
            DeclareLaunchArgument("communication_mode", default_value="sionna_hybrid"),
            DeclareLaunchArgument("require_sionna", default_value="true"),
            DeclareLaunchArgument("random_seed", default_value="42"),
            DeclareLaunchArgument("sionna_scene_xml", default_value=""),
            DeclareLaunchArgument("radio_map_cache", default_value=""),
            DeclareLaunchArgument("run_monitor", default_value="true"),
            DeclareLaunchArgument(
                "result_file",
                default_value="/tmp/racer_warehouse_loaded_sionna_result.json",
            ),
            OpaqueFunction(function=setup),
        ]
    )
