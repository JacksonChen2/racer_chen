from pathlib import Path

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _shared_remappings(zero_id):
    tx = f"/racer_sionna/tx/drone_{zero_id}"
    rx = f"/racer_sionna/rx/drone_{zero_id}"
    pairs = (
        ("/swarm_expl/drone_state_send", tx + "/drone_state"),
        ("/swarm_expl/drone_state_recv", rx + "/drone_state"),
        ("/swarm_expl/pair_opt_send", tx + "/pair_opt"),
        ("/swarm_expl/pair_opt_recv", rx + "/pair_opt"),
        ("/swarm_expl/pair_opt_res_send", tx + "/pair_opt_res"),
        ("/swarm_expl/pair_opt_res_recv", rx + "/pair_opt_res"),
        ("/planning/swarm_traj_send", tx + "/trajectory"),
        ("/planning/swarm_traj_recv", rx + "/trajectory"),
        ("/multi_map_manager/chunk_stamps_send", tx + "/chunk_stamps"),
        ("/multi_map_manager/chunk_stamps_recv", rx + "/chunk_stamps"),
        ("/multi_map_manager/chunk_data_send", tx + "/chunk_data"),
        ("/multi_map_manager/chunk_data_recv", rx + "/chunk_data"),
        ("/swarm_expl/global_assignment_send", tx + "/global_assignment"),
        ("/swarm_expl/global_assignment_recv", rx + "/global_assignment"),
    )
    return list(pairs)


def _launch_nodes(context):
    drone_count = int(LaunchConfiguration("drone_count").perform(context))
    trigger_minimum_cloud_frames = int(
        LaunchConfiguration("trigger_minimum_cloud_frames").perform(context)
    )
    trigger_delay_s = float(LaunchConfiguration("trigger_delay_s").perform(context))
    if trigger_minimum_cloud_frames < 0:
        raise RuntimeError("trigger_minimum_cloud_frames must be non-negative")
    if trigger_delay_s < 0.0:
        raise RuntimeError("trigger_delay_s must be non-negative")
    scenario = LaunchConfiguration("scenario").perform(context)
    if scenario not in (
        "warehouse_simple", "warehouse_loaded", "warehouse_loaded_center",
        "warehouse_loaded_full", "warehouse_full",
    ):
        raise RuntimeError(f"unsupported warehouse scenario: {scenario}")
    communication_mode = LaunchConfiguration("communication_mode").perform(context)
    network_topology = LaunchConfiguration("network_topology").perform(context)
    if network_topology not in (
        "distributed", "nearest_neighbors", "distance_radius", "ap_assisted",
        "bs_round_robin",
    ):
        raise RuntimeError(
            "network_topology must be distributed, nearest_neighbors, "
            "distance_radius, ap_assisted, or bs_round_robin"
        )
    nearest_neighbor_count = int(
        LaunchConfiguration("nearest_neighbor_count").perform(context)
    )
    lossless_nearest_neighbor_count = int(
        LaunchConfiguration("lossless_nearest_neighbor_count").perform(context)
    )
    lossless_communication_range_m = float(
        LaunchConfiguration("lossless_communication_range_m").perform(context)
    )
    lossless_control_only = LaunchConfiguration(
        "lossless_control_only"
    ).perform(context).lower() in ("1", "true", "yes", "on")
    directed_message_unicast = LaunchConfiguration(
        "directed_message_unicast"
    ).perform(context).lower() in ("1", "true", "yes", "on")
    chunk_data_pre_enqueue_dedup = LaunchConfiguration(
        "chunk_data_pre_enqueue_dedup"
    ).perform(context).lower() in ("1", "true", "yes", "on")
    chunk_data_max_pending_per_link = int(
        LaunchConfiguration("chunk_data_max_pending_per_link").perform(context)
    )
    communication_range_m = float(
        LaunchConfiguration("communication_range_m").perform(context)
    )
    ideal_coalesce_window_ms = float(
        LaunchConfiguration("ideal_coalesce_window_ms").perform(context)
    )
    require_sionna = LaunchConfiguration("require_sionna").perform(context).lower() in (
        "1", "true", "yes", "on"
    )
    lkh_dir = Path(LaunchConfiguration("lkh_dir").perform(context)).resolve()
    lkh_dir.mkdir(parents=True, exist_ok=True)
    params_file = str(
        Path(get_package_share_directory("racer_isaac_adapter"))
        / "config"
        / "original_warehouse_simple.yaml"
    )
    scenario_parameters = {}
    ap_position = [0.0, 3.0, 8.10]
    if scenario in (
        "warehouse_loaded", "warehouse_loaded_center", "warehouse_loaded_full"
    ):
        # Coordinate/storage adaptation only; all upstream RACER planning
        # modules continue to consume the same parameters and code paths.
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
        # Sionna radio-map optimum across the complete factory footprint and
        # the loaded RACER rack zone.  The enclosure hangs from z=9.0 m and
        # retains the same 0.45 m mounting-to-phase-centre offset.
        ap_position = [-13.5, 16.0, 8.55]
    if scenario == "warehouse_loaded_full":
        # Expand both the storage volume and coverage box to the complete
        # enclosed factory.  The original rack-zone profile remains unchanged.
        scenario_parameters = {
            "sdf_map.map_size_x": 56.2,
            "sdf_map.map_size_y": 64.0,
            "sdf_map.map_size_z": 9.0,
            "sdf_map.virtual_ceil_height": 8.4,
            "sdf_map.box_min_x": -27.0,
            "sdf_map.box_min_y": -23.0,
            "sdf_map.box_min_z": 0.4,
            "sdf_map.box_max_x": 6.0,
            "sdf_map.box_max_y": 30.0,
            "sdf_map.box_max_z": 8.4,
        }
    if scenario == "warehouse_full":
        scenario_parameters = {
            "sdf_map.map_size_x": 56.2,
            "sdf_map.map_size_y": 54.0,
            "sdf_map.map_size_z": 9.0,
            "sdf_map.virtual_ceil_height": 8.4,
            "sdf_map.box_min_x": -27.0,
            "sdf_map.box_min_y": 0.6,
            "sdf_map.box_min_z": 0.4,
            "sdf_map.box_max_x": 6.0,
            "sdf_map.box_max_y": 30.6,
            "sdf_map.box_max_z": 8.4,
        }
        ap_position = [-10.02891489217081, 14.888611215255622, 7.55]

    ap_overrides = [
        LaunchConfiguration(name).perform(context)
        for name in ("ap_position_x", "ap_position_y", "ap_position_z")
    ]
    if any(ap_overrides):
        if not all(ap_overrides):
            raise RuntimeError("all three ap_position overrides must be supplied")
        ap_position = [float(value) for value in ap_overrides]
    ap_tx_power_dbm = float(
        LaunchConfiguration("ap_tx_power_dbm").perform(context)
    )
    uav_tx_power_dbm = float(
        LaunchConfiguration("uav_tx_power_dbm").perform(context)
    )
    carrier_frequency_hz = float(
        LaunchConfiguration("carrier_frequency_hz").perform(context)
    )
    bs_max_retries = int(
        LaunchConfiguration("bs_max_retries").perform(context)
    )
    max_retries = int(LaunchConfiguration("max_retries").perform(context))
    fixed_mcs_index = int(
        LaunchConfiguration("fixed_mcs_index").perform(context)
    )
    random_seed = int(LaunchConfiguration("random_seed").perform(context))
    algorithm_variant = LaunchConfiguration("algorithm_variant").perform(context)
    if algorithm_variant not in ("original", "oracle"):
        raise RuntimeError("algorithm_variant must be original or oracle")
    exploration_assignment_mode = LaunchConfiguration(
        "exploration_assignment_mode"
    ).perform(context)
    if exploration_assignment_mode not in (
        "original", "local_component", "global_cooperative"
    ):
        raise RuntimeError(
            "exploration_assignment_mode must be original, local_component, or global_cooperative"
        )
    lkh_executable = str(
        Path(get_package_prefix("racer_original_core"))
        / "lib"
        / "racer_original_core"
        / "racer_original_lkh_cli"
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
                "exploration.random_seed": random_seed,
                "topo_prm.random_seed": random_seed,
                "traj_server.drone_id": drone_id,
                "traj_server.drone_num": drone_count,
                "multi_map_manager.chunk_size": 50 if algorithm_variant == "oracle" else 200,
                "fsm.repeat_send_num": 1 if algorithm_variant == "oracle" else 10,
                "oracle_shared_map.stamp_period": 0.05,
                "oracle_shared_map.chunk_period": 0.02,
                "oracle_shared_map.stamp_throttle": 0.05,
                "oracle_global.assignment_interval": 0.5,
                "oracle_global.balance_weight_m": 1.0,
                "exploration_assignment_mode": exploration_assignment_mode,
                "global_cooperative.assignment_interval": 0.5,
                "global_cooperative.state_freshness": 0.75,
                "global_cooperative.nominal_velocity": 1.5,
                "global_cooperative.gain_scale": 0.001,
                "global_cooperative.utility_epsilon": 0.1,
                "global_cooperative.lambda_overlap": 2.0,
                "global_cooperative.lambda_age": 0.05,
                "global_cooperative.overlap_distance": 6.0,
                "global_cooperative.cluster_distance": 3.0,
                "global_cooperative.distance_min": 8.0,
                "global_cooperative.distance_max": 45.0,
                "global_cooperative.distance_gamma": 0.7,
                "global_cooperative.distance_fallback_when_idle": True,
                "global_cooperative.finish_confirmation_cycles": 6,
                "global_cooperative.local_failures_before_report": 2,
                "global_cooperative.unreachable_distinct_uavs": 2,
                "global_cooperative.unreachable_global_failures": 4,
                "global_cooperative.unreachable_retry_s": 20.0,
            },
        ]
        agent_remaps = [
            ("/odom_world", prefix + "/odom"),
            ("/racer/sensor_points", prefix + "/points"),
            ("/racer/tracking_lost", prefix + "/tracking_lost"),
            ("/move_base_simple/goal", "/racer/start"),
            ("/planning/replan", prefix + "/planning/replan"),
            ("/planning/new", prefix + "/planning/new"),
            ("/planning/bspline", prefix + "/planning/bspline"),
            ("/planning_vis/trajectory", prefix + "/planning_vis/trajectory"),
            ("/planning_vis/topo_path", prefix + "/planning_vis/topo_path"),
            ("/planning_vis/prediction", prefix + "/planning_vis/prediction"),
            ("/planning_vis/visib_constraint", prefix + "/planning_vis/visib_constraint"),
            ("/planning_vis/frontier", prefix + "/planning_vis/frontier"),
            ("/planning_vis/yaw", prefix + "/planning_vis/yaw"),
            ("/planning_vis/viewpoints", prefix + "/planning_vis/viewpoints"),
            ("/swarm_expl/grid_tour_send", prefix + "/planning_vis/grid_tour"),
            ("/swarm_expl/hgrid_send", prefix + "/planning_vis/hgrid"),
            (f"/multi_map_manager/marker_{drone_id}", prefix + "/planning_vis/map_chunks"),
        ] + _shared_remappings(zero_id)
        nodes.append(
            Node(
                package="racer_original_core",
                executable=("racer_oracle_exploration_node"
                            if algorithm_variant == "oracle"
                            else "racer_original_exploration_node"),
                name=(f"racer_oracle_exploration_{drone_id}"
                      if algorithm_variant == "oracle"
                      else f"racer_original_exploration_{drone_id}"),
                output="screen",
                parameters=agent_parameters,
                remappings=agent_remaps,
            )
        )
        nodes.append(
            Node(
                package="racer_original_core",
                executable="racer_original_traj_server",
                name=f"racer_original_traj_server_{drone_id}",
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
                package="racer_isaac_adapter",
                executable="position_command_adapter",
                name=f"racer_isaac_control_adapter_{drone_id}",
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
                    package="racer_original_core",
                    executable="racer_original_lkh_server",
                    name=f"racer_original_{label}_{drone_id}",
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
            package="racer_isaac_adapter",
            executable="exploration_trigger",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "delay": trigger_delay_s,
                    "repeats": 10,
                    "drone_count": drone_count,
                    "minimum_cloud_frames": trigger_minimum_cloud_frames,
                }
            ],
        )
    )
    communication_share = Path(get_package_share_directory("racer_sionna_comm"))
    communication_config = str(
        communication_share / "config" / "warehouse_simple_communication.yaml"
    )
    nodes.append(
        Node(
            package="racer_sionna_comm",
            executable="racer_sionna_communication_proxy",
            name="racer_sionna_communication_proxy",
            output="screen",
            parameters=[
                communication_config,
                {
                    "use_sim_time": True,
                    "drone_count": drone_count,
                    "mode": communication_mode,
                    "network_topology": network_topology,
                    "nearest_neighbor_count": nearest_neighbor_count,
                    "lossless_nearest_neighbor_count": lossless_nearest_neighbor_count,
                    "lossless_communication_range_m": lossless_communication_range_m,
                    "lossless_control_only": lossless_control_only,
                    "directed_message_unicast": directed_message_unicast,
                    "chunk_data_pre_enqueue_dedup": chunk_data_pre_enqueue_dedup,
                    "chunk_data_max_pending_per_link": chunk_data_max_pending_per_link,
                    "communication_range_m": communication_range_m,
                    "ideal_coalesce_window_ms": ideal_coalesce_window_ms,
                    "ap_tx_power_dbm": ap_tx_power_dbm,
                    "tx_power_dbm": uav_tx_power_dbm,
                    "carrier_frequency_hz": carrier_frequency_hz,
                    "max_retries": max_retries,
                    "bs_max_retries": bs_max_retries,
                    "fixed_mcs_index": fixed_mcs_index,
                    "random_seed": random_seed,
                },
            ],
        )
    )
    if communication_mode != "ideal":
        nodes.append(
            Node(
                package="racer_sionna_comm",
                executable="sionna_channel_node.py",
                name="racer_sionna_channel",
                output="screen",
                parameters=[
                    communication_config,
                    {
                        "use_sim_time": True,
                        "drone_count": drone_count,
                        "network_topology": network_topology,
                        "ap_position": ap_position,
                        "ap_tx_power_dbm": ap_tx_power_dbm,
                        "tx_power_dbm": uav_tx_power_dbm,
                        "carrier_frequency_hz": carrier_frequency_hz,
                        "random_seed": random_seed,
                        "scene_xml": LaunchConfiguration("sionna_scene_xml").perform(context),
                        "radio_map_cache": LaunchConfiguration("radio_map_cache").perform(context),
                        "require_sionna": require_sionna,
                    },
                ],
            )
        )
    return nodes


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("drone_count", default_value="5"),
            DeclareLaunchArgument(
                "trigger_minimum_cloud_frames", default_value="25"
            ),
            DeclareLaunchArgument("trigger_delay_s", default_value="5.0"),
            DeclareLaunchArgument("scenario", default_value="warehouse_simple"),
            DeclareLaunchArgument("communication_mode", default_value="sionna"),
            DeclareLaunchArgument("algorithm_variant", default_value="original"),
            DeclareLaunchArgument(
                "exploration_assignment_mode", default_value="original"
            ),
            DeclareLaunchArgument("network_topology", default_value="distributed"),
            DeclareLaunchArgument("nearest_neighbor_count", default_value="0"),
            DeclareLaunchArgument(
                "lossless_nearest_neighbor_count", default_value="0"
            ),
            DeclareLaunchArgument(
                "lossless_communication_range_m", default_value="0.0"
            ),
            DeclareLaunchArgument("lossless_control_only", default_value="false"),
            DeclareLaunchArgument("directed_message_unicast", default_value="false"),
            DeclareLaunchArgument(
                "chunk_data_pre_enqueue_dedup", default_value="false"
            ),
            DeclareLaunchArgument(
                "chunk_data_max_pending_per_link", default_value="0"
            ),
            DeclareLaunchArgument("communication_range_m", default_value="4.0"),
            DeclareLaunchArgument("ideal_coalesce_window_ms", default_value="20.0"),
            DeclareLaunchArgument("require_sionna", default_value="true"),
            DeclareLaunchArgument("sionna_scene_xml", default_value=""),
            DeclareLaunchArgument("radio_map_cache", default_value=""),
            DeclareLaunchArgument("ap_position_x", default_value=""),
            DeclareLaunchArgument("ap_position_y", default_value=""),
            DeclareLaunchArgument("ap_position_z", default_value=""),
            DeclareLaunchArgument("ap_tx_power_dbm", default_value="33.0"),
            DeclareLaunchArgument("uav_tx_power_dbm", default_value="23.0"),
            DeclareLaunchArgument(
                "carrier_frequency_hz", default_value="28000000000.0"
            ),
            DeclareLaunchArgument("max_retries", default_value="3"),
            DeclareLaunchArgument("bs_max_retries", default_value="3"),
            DeclareLaunchArgument("fixed_mcs_index", default_value="-1"),
            DeclareLaunchArgument("random_seed", default_value="42"),
            DeclareLaunchArgument(
                "lkh_dir", default_value="/tmp/racer_original_fidelity_lkh"
            ),
            OpaqueFunction(function=_launch_nodes),
        ]
    )
