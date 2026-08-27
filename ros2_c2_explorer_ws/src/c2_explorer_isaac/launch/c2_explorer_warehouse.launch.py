from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _direct_remappings():
    pairs = (
        ("/swarm_expl/drone_state_send", "/c2_explorer/swarm/drone_state"),
        ("/swarm_expl/drone_state_recv", "/c2_explorer/swarm/drone_state"),
        ("/swarm_expl/meeting_opt_send", "/c2_explorer/swarm/meeting_opt"),
        ("/swarm_expl/meeting_opt_recv", "/c2_explorer/swarm/meeting_opt"),
        ("/swarm_expl/meeting_opt_res_send", "/c2_explorer/swarm/meeting_opt_res"),
        ("/swarm_expl/meeting_opt_res_recv", "/c2_explorer/swarm/meeting_opt_res"),
        ("/swarm_expl/rendezvous_send", "/c2_explorer/swarm/rendezvous"),
        ("/swarm_expl/rendezvous_recv", "/c2_explorer/swarm/rendezvous"),
        ("/planning/swarm_traj_send", "/c2_explorer/swarm/trajectory"),
        ("/planning/swarm_traj_recv", "/c2_explorer/swarm/trajectory"),
        ("/communication/heartbeat_send", "/c2_explorer/swarm/heartbeat"),
        ("/communication/heartbeat_recv", "/c2_explorer/swarm/heartbeat"),
        ("/multi_map_manager/chunk_stamps_send", "/c2_explorer/map/chunk_stamps"),
        ("/multi_map_manager/chunk_stamps_recv", "/c2_explorer/map/chunk_stamps"),
        ("/multi_map_manager/chunk_data_send", "/c2_explorer/map/chunk_data"),
        ("/multi_map_manager/chunk_data_recv", "/c2_explorer/map/chunk_data"),
    )
    return list(pairs)


def _communication_remappings(zero_id, communication_mode):
    if communication_mode == "direct":
        return _direct_remappings()
    tx = f"/c2_communication/tx/drone_{zero_id}"
    rx = f"/c2_communication/rx/drone_{zero_id}"
    pairs = (
        ("/swarm_expl/drone_state_send", tx + "/drone_state"),
        ("/swarm_expl/drone_state_recv", rx + "/drone_state"),
        ("/swarm_expl/meeting_opt_send", tx + "/meeting_opt"),
        ("/swarm_expl/meeting_opt_recv", rx + "/meeting_opt"),
        ("/swarm_expl/meeting_opt_res_send", tx + "/meeting_opt_res"),
        ("/swarm_expl/meeting_opt_res_recv", rx + "/meeting_opt_res"),
        ("/swarm_expl/rendezvous_send", tx + "/rendezvous"),
        ("/swarm_expl/rendezvous_recv", rx + "/rendezvous"),
        ("/planning/swarm_traj_send", tx + "/trajectory"),
        ("/planning/swarm_traj_recv", rx + "/trajectory"),
        ("/communication/heartbeat_send", tx + "/heartbeat"),
        ("/communication/heartbeat_recv", rx + "/heartbeat"),
        ("/multi_map_manager/chunk_stamps_send", tx + "/chunk_stamps"),
        ("/multi_map_manager/chunk_stamps_recv", rx + "/chunk_stamps"),
        ("/multi_map_manager/chunk_data_send", tx + "/chunk_data"),
        ("/multi_map_manager/chunk_data_recv", rx + "/chunk_data"),
    )
    return list(pairs)


def _launch_nodes(context):
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    scenario = LaunchConfiguration("scenario").perform(context)
    communication_mode = LaunchConfiguration("communication_mode").perform(context)
    if communication_mode not in ("direct", "ideal", "sionna"):
        raise RuntimeError("communication_mode must be direct, ideal, or sionna")
    random_seed = int(LaunchConfiguration("random_seed").perform(context))
    max_retries = int(LaunchConfiguration("max_retries").perform(context))
    debug_opt_output = LaunchConfiguration("debug_opt_output").perform(context).lower() in (
        "1",
        "true",
        "yes",
        "on",
    )
    if scenario not in (
        "warehouse_simple", "warehouse_loaded", "warehouse_loaded_center",
        "warehouse_full",
    ):
        raise RuntimeError(f"unsupported warehouse scenario: {scenario}")
    lkh_dir = Path(LaunchConfiguration("lkh_dir").perform(context)).resolve()
    lkh_dir.mkdir(parents=True, exist_ok=True)
    params_file = str(
        Path(get_package_share_directory("c2_explorer_isaac"))
        / "config"
        / "c2_warehouse.yaml"
    )
    scenario_parameters = {}
    if scenario in ("warehouse_loaded", "warehouse_loaded_center"):
        # The upstream SDF buffer is centred on the ROS map origin.  Keep the
        # original algorithm intact and enlarge only its Isaac/world-frame
        # storage envelope so the offset rack zone fits in that buffer.
        scenario_parameters = {
            "sdf_map.map_size_x": 56.2,
            "sdf_map.map_size_y": 54.0,
            "sdf_map.map_size_z": 9.0,
            "sdf_map.virtual_ceil_height": 8.4,
            "sdf_map.box_min_x": -26.8,
            "sdf_map.box_min_y": 7.2,
            "sdf_map.box_min_z": 0.4,
            "sdf_map.box_max_x": 5.8,
            "sdf_map.box_max_y": 26.2,
            "sdf_map.box_max_z": 8.4,
        }
    if scenario == "warehouse_full":
        # Centred SDF storage is enlarged only enough to contain the northern
        # full-warehouse flight box. C2 weights and planning parameters remain
        # the audited ROS1 values.
        scenario_parameters = {
            "sdf_map.map_size_x": 56.2,
            "sdf_map.map_size_y": 64.0,
            "sdf_map.map_size_z": 9.0,
            "sdf_map.virtual_ceil_height": 8.4,
            "sdf_map.box_min_x": -27.0,
            "sdf_map.box_min_y": 0.6,
            "sdf_map.box_min_z": 0.4,
            "sdf_map.box_max_x": 6.0,
            "sdf_map.box_max_y": 30.6,
            "sdf_map.box_max_z": 8.4,
            "map_ros.coverage_diagnostic_period": 5.0,
        }
    # Keep the upstream 5 m CommunicationGraph threshold in both experiment
    # cases. It is C2 algorithm logic (and gates map-chunk exchange), whereas
    # ideal/Sionna below changes only the external packet transport.
    lkh_executable = str(
        Path(get_package_prefix("c2_explorer_core"))
        / "lib"
        / "c2_explorer_core"
        / "c2_explorer_lkh_cli"
    )
    nodes = []
    for zero_id in range(drone_count):
        drone_id = zero_id + 1
        prefix = f"/drone_{zero_id}"
        agent_parameters = [
            params_file,
            scenario_parameters,
            {
                "exploration.drone_id": drone_id,
                "exploration.drone_num": drone_count,
                "exploration.vis_drone_id": 1,
                "exploration.tsp_dir": str(lkh_dir),
                "exploration.mtsp_dir": str(lkh_dir),
                "traj_server.drone_id": drone_id,
                "traj_server.drone_num": drone_count,
                # Diagnostic logging only; the upstream parameter does not
                # participate in allocation or meeting-protocol decisions.
                "fsm.debug_opt_output": debug_opt_output,
            },
        ]
        agent_remaps = [
            ("/odom_world", prefix + "/odom"),
            ("/c2_explorer/sensor_points", prefix + "/points"),
            ("/move_base_simple/goal", "/c2_explorer/start"),
            ("/planning/replan", prefix + "/planning/replan"),
            ("/planning/new", prefix + "/planning/new"),
            ("/planning/bspline", prefix + "/planning/bspline"),
            ("/planning_vis/trajectory", prefix + "/planning_vis/trajectory"),
            ("/planning_vis/prediction", prefix + "/planning_vis/prediction"),
            ("/planning_vis/visib_constraint", prefix + "/planning_vis/visib_constraint"),
            ("/planning_vis/frontier", prefix + "/planning_vis/frontier"),
            ("/planning_vis/yaw", prefix + "/planning_vis/yaw"),
            ("/planning_vis/viewpoints", prefix + "/planning_vis/viewpoints"),
            ("/planning_vis/hgrid", prefix + "/planning_vis/hgrid"),
            ("/planning_vis/pose", prefix + "/planning_vis/pose"),
            ("/planning_vis/pose_array", prefix + "/planning_vis/pose_array"),
            ("/planning_vis/connectivity_graph", prefix + "/planning_vis/connectivity_graph"),
            ("/communication/graph_vis", prefix + "/planning_vis/communication_graph"),
            (f"/multi_map_manager/marker_{drone_id}", prefix + "/planning_vis/map_chunks"),
        ] + _communication_remappings(zero_id, communication_mode)
        nodes.append(
            Node(
                package="c2_explorer_core",
                executable="c2_explorer_node",
                name=f"c2_explorer_{drone_id}",
                output="screen",
                parameters=agent_parameters,
                remappings=agent_remaps,
            )
        )
        nodes.append(
            Node(
                package="c2_explorer_core",
                executable="c2_explorer_traj_server",
                name=f"c2_explorer_traj_server_{drone_id}",
                output="screen",
                parameters=agent_parameters,
                remappings=[
                    ("/odom_world", prefix + "/odom"),
                    ("/planning/bspline", prefix + "/planning/bspline"),
                    ("/planning/replan", prefix + "/planning/replan"),
                    ("/planning/new", prefix + "/planning/new"),
                    ("/position_cmd", prefix + "/position_cmd"),
                    ("/planning/position_cmd_vis", prefix + "/planning_vis/position_cmd"),
                    ("/planning/travel_traj", prefix + "/planning_vis/travel_traj"),
                    ("/loop_fusion/pg_T_vio", prefix + "/unused/pg_T_vio"),
                ],
            )
        )
        nodes.append(
            Node(
                package="c2_explorer_isaac",
                executable="position_command_adapter",
                name=f"c2_explorer_control_adapter_{drone_id}",
                output="screen",
                parameters=[
                    {
                        "use_sim_time": True,
                        "drone_id": drone_id,
                        "position_gain": 2.5,
                        "maximum_speed": 2.0,
                        "command_timeout": 0.25,
                        "tracking_error_threshold": 1.2,
                        "tracking_error_hold": 1.0,
                        "tracking_low_speed_threshold": 0.25,
                        "tracking_severe_error_threshold": 3.0,
                        "recovery_cooldown": 2.0,
                        "recovery_brake_time": 0.4,
                    }
                ],
            )
        )
        for problem_id, label in ((1, "tsp"), (2, "acvrp")):
            nodes.append(
                Node(
                    package="c2_explorer_core",
                    executable="c2_explorer_lkh_server",
                    name=f"c2_explorer_{label}_{drone_id}",
                    output="screen",
                    parameters=[
                        {
                            "use_sim_time": True,
                            "exploration.drone_id": drone_id,
                            "exploration.problem_id": problem_id,
                            "exploration.mtsp_dir": str(lkh_dir),
                            "exploration.lkh_executable": lkh_executable,
                        }
                    ],
                )
            )
    nodes.append(
        Node(
            package="c2_explorer_isaac",
            executable="exploration_trigger",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "delay": 5.0,
                    "repeats": 10,
                    "drone_count": drone_count,
                    "minimum_cloud_frames": 25,
                }
            ],
        )
    )
    if communication_mode != "direct":
        communication_config = str(
            Path(get_package_share_directory("c2_explorer_isaac"))
            / "config"
            / "c2_communication.yaml"
        )
        nodes.append(
            Node(
                package="c2_explorer_isaac",
                executable="c2_communication_proxy",
                name="c2_communication_proxy",
                output="screen",
                parameters=[
                    communication_config,
                    {
                        "use_sim_time": True,
                        "drone_count": drone_count,
                        "mode": communication_mode,
                        "random_seed": random_seed,
                        "max_retries": max_retries,
                    },
                ],
            )
        )
        if communication_mode == "sionna":
            nodes.append(
                Node(
                    package="c2_explorer_isaac",
                    executable="c2_sionna_channel.py",
                    name="racer_sionna_channel",
                    output="screen",
                    parameters=[
                        communication_config,
                        {
                            "use_sim_time": True,
                            "drone_count": drone_count,
                            "network_topology": "distributed",
                            "scene_xml": LaunchConfiguration(
                                "sionna_scene_xml"
                            ).perform(context),
                            "radio_map_cache": LaunchConfiguration(
                                "radio_map_cache"
                            ).perform(context),
                            "require_sionna": True,
                            "random_seed": random_seed,
                        },
                    ],
                )
            )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_count", default_value="5"),
            DeclareLaunchArgument("scenario", default_value="warehouse_simple"),
            DeclareLaunchArgument("communication_mode", default_value="direct"),
            DeclareLaunchArgument("sionna_scene_xml", default_value=""),
            DeclareLaunchArgument("radio_map_cache", default_value=""),
            DeclareLaunchArgument("random_seed", default_value="42"),
            DeclareLaunchArgument("max_retries", default_value="0"),
            DeclareLaunchArgument("debug_opt_output", default_value="false"),
            DeclareLaunchArgument(
                "lkh_dir", default_value="/tmp/c2_explorer_lkh"
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
