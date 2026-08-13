#!/usr/bin/env python3
"""Isaac Sim boundary for the source-faithful ROS2/C++ RACER port.

This is the self-contained simulator bridge for the C++ RACER nodes.  Isaac
Sim exposes its application API in Python; exploration, mapping, allocation,
planning remain in the original RACER C++ sources.  The default
plant is the generated 0.98 kg RACER SO3 plus-quadrotor; the legacy Crazyflie
profile remains available for comparison.  The RACER profile follows the
upstream ROS 1 simulator rather than the paper's real vehicle: a 1 kHz plant,
200 Hz odometry/IMU, and a 640x480 30 Hz ideal pinhole depth camera.
"""

import argparse
import asyncio
from collections import deque
import faulthandler
import json
import math
import os
from pathlib import Path
import signal
import time
from typing import Sequence, Tuple


# Isaac Kit cannot be attached to by ptrace on all target machines.  Keep an
# in-process, read-only stack probe available for diagnosing a stalled bridge.
# SIGUSR1 does not alter simulation or RACER state; it only writes Python
# thread stacks to the existing Isaac log.
faulthandler.register(signal.SIGUSR1, all_threads=True)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=120.0)
    parser.add_argument(
        "--mapping-coverage-target",
        type=float,
        default=0.0,
        help=(
            "stop when the most complete shared RACER SDF map reaches this "
            "known-voxel fraction; zero disables coverage stopping"
        ),
    )
    parser.add_argument("--drone-count", type=int, default=3)
    parser.add_argument("--physics-rate-hz", type=float, default=1000.0)
    parser.add_argument("--sensor-rate-hz", type=float, default=30.0)
    parser.add_argument("--depth-width", type=int, default=640)
    parser.add_argument("--depth-height", type=int, default=480)
    parser.add_argument(
        "--scenario", default="acceptance_15x9x2"
    )
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--render-every", type=int, default=5)
    parser.add_argument("--diagnostics", action="store_true")
    parser.add_argument(
        "--animate-propellers",
        action=argparse.BooleanOptionalAction,
        default=None,
        help=(
            "drive render-only propeller Xforms from RACER motor RPM; defaults "
            "to enabled in the interactive viewport and disabled headlessly"
        ),
    )
    parser.add_argument(
        "--propeller-visual-hz",
        type=float,
        default=60.0,
        help="maximum USD propeller-transform update rate",
    )
    parser.add_argument(
        "--visualize-exploration",
        action="store_true",
        help=(
            "draw the shared occupied map, planned paths, and flown trails "
            "in the interactive Isaac Sim viewport"
        ),
    )
    parser.add_argument(
        "--record-trajectory-history",
        action="store_true",
        help=(
            "store sampled vehicle poses in the final result for later "
            "visual replay; disabled by default for metric-only headless runs"
        ),
    )
    parser.add_argument(
        "--visualization-max-map-points",
        type=int,
        default=40000,
        help="display-only occupied-voxel cap; does not change the RACER map",
    )
    parser.add_argument(
        "--interactive-render-hz",
        type=float,
        default=30.0,
        help=(
            "minimum wall-clock viewport refresh rate while interactive; "
            "keeps mouse camera input responsive when simulation is slower "
            "than real time"
        ),
    )
    parser.add_argument(
        "--camera-ray-budget",
        type=int,
        default=76800,
        help=(
            "maximum depth samples forwarded per frame; 76800 preserves every "
            "sample from the original 640x480 skip-pixel=2 grid"
        ),
    )
    parser.add_argument(
        "--vehicle-model",
        choices=("racer_so3", "crazyflie"),
        default="racer_so3",
        help="PhysX vehicle plant used below the C++ RACER velocity command",
    )
    parser.add_argument(
        "--vehicle-usd",
        type=Path,
        default=(
            Path(__file__).resolve().parents[3]
            / "isaac_assets"
            / "racer_so3_quadrotor"
            / "usd"
            / "racer_so3_quadrotor_flattened.usd"
        ),
        help="generated RACER SO3 USD asset",
    )
    parser.add_argument(
        "--scene-usd",
        type=Path,
        help=(
            "reference an external collision-enabled USD instead of building "
            "the deterministic acceptance obstacles"
        ),
    )
    parser.add_argument(
        "--starts",
        type=float,
        nargs="+",
        help="flat x y z launch positions; provide three values per vehicle",
    )
    parser.add_argument(
        "--control-probe",
        action="store_true",
        help="command one vehicle at 0.35 m/s for isolated 6-DOF testing",
    )
    parser.add_argument(
        "--control-probe-command",
        type=float,
        nargs=3,
        default=(0.30, 0.20, 0.10),
        metavar=("VX", "VY", "VZ"),
        help="world-frame velocity used with --control-probe",
    )
    return parser.parse_args()


ARGS = parse_arguments()
if ARGS.scene_usd is not None:
    ARGS.scene_usd = ARGS.scene_usd.expanduser().resolve()
    if not ARGS.scene_usd.is_file():
        raise SystemExit(f"scene USD does not exist: {ARGS.scene_usd}")
if ARGS.vehicle_model == "racer_so3":
    ARGS.vehicle_usd = ARGS.vehicle_usd.expanduser().resolve()
    if not ARGS.vehicle_usd.is_file():
        raise SystemExit(f"vehicle USD does not exist: {ARGS.vehicle_usd}")
if ARGS.camera_ray_budget <= 0:
    raise SystemExit("--camera-ray-budget must be positive")
if ARGS.visualization_max_map_points <= 0:
    raise SystemExit("--visualization-max-map-points must be positive")
if ARGS.interactive_render_hz <= 0.0:
    raise SystemExit("--interactive-render-hz must be positive")
if ARGS.propeller_visual_hz <= 0.0:
    raise SystemExit("--propeller-visual-hz must be positive")
if ARGS.headless and ARGS.visualize_exploration:
    raise SystemExit("--visualize-exploration requires an interactive window")
ANIMATE_PROPELLERS = (
    not ARGS.headless
    if ARGS.animate_propellers is None
    else bool(ARGS.animate_propellers)
)

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp(
    {
        "headless": ARGS.headless,
        "renderer": "RaytracedLighting",
        "width": 1280,
        "height": 720,
    }
)

from isaacsim.core.utils.extensions import enable_extension  # noqa: E402


enable_extension("isaacsim.ros2.bridge")
if ARGS.visualize_exploration:
    enable_extension("isaacsim.util.debug_draw")
simulation_app.update()

import numpy as np  # noqa: E402
import omni.syntheticdata as syntheticdata  # noqa: E402
import omni.usd  # noqa: E402
import rclpy  # noqa: E402
from geometry_msgs.msg import Twist  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from isaacsim.core.api.objects import FixedCuboid  # noqa: E402
from isaacsim.core.prims import SingleRigidPrim  # noqa: E402
from isaacsim.core.utils.viewports import set_camera_view  # noqa: E402
from isaacsim.sensors.camera import Camera  # noqa: E402
from isaacsim.sensors.physics import ContactSensor  # noqa: E402
from isaacsim.sensors.physx import RotatingLidarPhysX  # noqa: E402
from nav_msgs.msg import Odometry, Path as RosPath  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import Gf, PhysxSchema, Usd, UsdGeom, UsdLux, UsdPhysics  # noqa: E402
from rclpy.node import Node  # noqa: E402
from rclpy.qos import QoSProfile, ReliabilityPolicy  # noqa: E402
from rosgraph_msgs.msg import Clock  # noqa: E402
from sensor_msgs.msg import Imu, PointCloud2  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from visualization_msgs.msg import Marker  # noqa: E402

from crazyflie_cpp_bridge import (  # noqa: E402
    MASS as CRAZYFLIE_MASS,
    velocity_wrench,
)
from pointcloud_cpp_bridge import (  # noqa: E402
    create_xyzi_cloud,
    read_xyzi_cloud,
)
from racer_so3_cpp_bridge import (  # noqa: E402
    MASS as RACER_SO3_MASS,
    hover_rpm as racer_hover_rpm,
    velocity_motor_wrench,
)
from scenario_cpp_bridge import (  # noqa: E402
    DRONE_RADIUS,
    get_scenario,
    obstacle_clearance,
    pairwise_distances,
    point_box_signed_clearance,
)
from safety_cpp_bridge import (  # noqa: E402
    aabb_obstacle_filter,
    cbf_swarm_filter,
    flight_volume_filter,
    pointcloud_obstacle_filter,
    sweep_obstacle_filter,
)

if ARGS.visualize_exploration:
    from isaacsim.util.debug_draw import _debug_draw  # noqa: E402


SCENARIO = get_scenario(ARGS.scenario)
if ARGS.starts is None:
    if len(SCENARIO.starts) < ARGS.drone_count:
        raise SystemExit(
            f"scenario {SCENARIO.name!r} provides {len(SCENARIO.starts)} "
            f"starts, but {ARGS.drone_count} vehicles were requested"
        )
    STARTS = SCENARIO.starts[:ARGS.drone_count]
else:
    if len(ARGS.starts) != 3 * ARGS.drone_count:
        raise SystemExit(
            "--starts requires exactly three values per configured vehicle"
        )
    STARTS = tuple(
        tuple(float(value) for value in ARGS.starts[index:index + 3])
        for index in range(0, len(ARGS.starts), 3)
    )


if ARGS.physics_rate_hz <= 0.0 or ARGS.sensor_rate_hz <= 0.0:
    raise ValueError("physics and sensor rates must be positive")
if ARGS.depth_width < 64 or ARGS.depth_height < 48:
    raise ValueError("depth resolution is too small")
PHYSICS_DT = (
    1.0 / ARGS.physics_rate_hz
    if ARGS.vehicle_model == "racer_so3"
    else 0.02
)
ODOM_PERIOD = 1.0 / (200.0 if ARGS.vehicle_model == "racer_so3" else 50.0)
DEPTH_PERIOD = 1.0 / ARGS.sensor_rate_hz
BODY_SIZE = (0.16, 0.16, 0.06)
DEPTH_WIDTH = ARGS.depth_width
DEPTH_HEIGHT = ARGS.depth_height
DEPTH_SCALE_X = DEPTH_WIDTH / 640.0
DEPTH_SCALE_Y = DEPTH_HEIGHT / 480.0
DEPTH_FX = 387.229 * DEPTH_SCALE_X
DEPTH_FY = 387.229 * DEPTH_SCALE_Y
DEPTH_CX = 321.046 * DEPTH_SCALE_X
DEPTH_CY = 243.449 * DEPTH_SCALE_Y
MAPPING_FX = 385.69793701171875 * DEPTH_SCALE_X
MAPPING_FY = 385.69793701171875 * DEPTH_SCALE_Y
MAPPING_CX = 324.0879821777344 * DEPTH_SCALE_X
MAPPING_CY = 239.10362243652344 * DEPTH_SCALE_Y
DEPTH_RENDER_HORIZON = 5.0
DEPTH_MIN_RANGE = 0.2
DEPTH_MAP_RANGE = 4.6
MAPPING_MIN_RAY_LENGTH = 0.5
MAPPING_MAX_RAY_LENGTH = 4.5
DEPTH_FILTER_MARGIN = max(1, round(2 * min(DEPTH_SCALE_X, DEPTH_SCALE_Y)))
DEPTH_SKIP_PIXEL = max(1, round(2 * min(DEPTH_SCALE_X, DEPTH_SCALE_Y)))
CAMERA_TRANSLATION = np.zeros(3)
# The upstream ideal renderer has no carrier geometry.  The generated Isaac
# visual envelope reaches 0.26 + 0.062 m from the body origin, so returns
# inside that envelope are necessarily self returns and must not enter RACER.
SELF_FILTER_RADIUS = 0.322 + 0.01
LIDAR_TRANSLATION = np.asarray((0.0, 0.0, 0.075))
# Use the complete rendered rotor envelope for execution safety.  The seven
# PhysX collision primitives reach about 0.287 m, but a 0.284 m query sphere
# left no allowance for their corners or the visual propeller discs.  The
# planner still receives the unchanged camera point cloud and map inflation;
# this radius exists only below the planner at the Isaac actuator boundary.
VEHICLE_RADIUS = (
    SELF_FILTER_RADIUS if ARGS.vehicle_model == "racer_so3" else DRONE_RADIUS
)
# VEHICLE_RADIUS is the hard, conservative rotor envelope.  Speed-dependent
# stopping allowance is added by pointcloud_obstacle_filter at every control
# step; adding another fixed 0.236 m here double-counted that margin and made a
# physically traversable 0.67 m warehouse gap infeasible at zero speed.
OBSTACLE_CONTROL_CLEARANCE = max(0.30, VEHICLE_RADIUS)
SWARM_CONTROL_DISTANCE = max(0.55, 2.0 * VEHICLE_RADIUS + 0.632)
# The execution interface needs limited tracking authority above the unchanged
# original manager.max_vel=1.5. The planner feed-forward never exceeds that
# value; at most 0.5 m/s is available to the position feedback term to recover
# SO3 attitude/motor lag. Obstacle and peer filters below still reduce this
# actuator bound dynamically using measured velocity and braking distance.
SOURCE_MAX_SPEED = 2.0 if ARGS.vehicle_model == "racer_so3" else 0.35
SAFETY_POINT_MEMORY_SECONDS = 0.35
SAFETY_POINT_MEMORY_RADIUS = 2.4
SAFETY_POINT_VOXEL_SIZE = 0.08
SAFETY_POINT_LIMIT = 2000
SAFETY_LIDAR_POINT_LIMIT = 1200
SAFETY_LIDAR_HORIZONTAL_RESOLUTION_DEG = 2.5
SAFETY_LIDAR_VERTICAL_RESOLUTION_DEG = 5.0
# The lidar is deliberately coarse and can miss a thin shelf edge or ceiling
# lamp between beams.  A low-level rigid-body sweep closes that geometric gap
# without feeding privileged scene information into RACER's planning map.
SCENE_QUERY_PERIOD = 0.02
SCENE_QUERY_RANGE = 2.4
# The sphere itself already encloses the full rotor/arm collision geometry.
# Retain an additional free-travel reserve for PhysX contact offset, attitude
# lag and the 20 ms scene-query interval.  This is an actuator safety margin,
# not an occupancy-map inflation or a planner parameter.
SCENE_QUERY_CLEARANCE = 0.06
SOURCE_MAX_YAW_RATE = (
    math.radians(10.0) if ARGS.vehicle_model == "racer_so3" else 0.15
)
ISAAC_SIM_ROOT = Path(
    os.environ.get("ISAAC_SIM_ROOT", str(Path.home() / "isaacsim"))
)
CRAZYFLIE_ASSET = ISAAC_SIM_ROOT / (
    "extscache/"
    "omni.warp.core-1.8.2+lx64/warp/examples/assets/crazyflie.usd"
)
USE_REFERENCE_VISUAL = True
VISUAL_DRONE_COLORS = (
    (1.0, 0.16, 0.10, 1.0),
    (0.15, 1.0, 0.25, 1.0),
    (1.0, 0.82, 0.08, 1.0),
    (0.75, 0.25, 1.0, 1.0),
    (1.0, 0.48, 0.05, 1.0),
)
VISUAL_MAP_COLOR = (0.0, 0.72, 1.0, 0.62)


def _low_level_safety_description() -> str:
    if ARGS.scene_usd is None:
        return "AABB stopping-distance velocity barrier"
    if SCENARIO.safety_min is not None and SCENARIO.safety_max is not None:
        return (
            "depth plus 360-degree safety-lidar, PhysX rigid-body sweep, "
            "and flight-volume stopping-distance barriers"
        )
    return (
        "depth points plus PhysX rigid-body sweep stopping-distance "
        "velocity barrier"
    )


def _external_obstacle_filter(
    preferred: Sequence[float],
    position: Sequence[float],
    points_world: Sequence[Sequence[float]],
    current_velocity: Sequence[float],
    sweep_constraints=(),
) -> np.ndarray:
    result = pointcloud_obstacle_filter(
        preferred,
        position,
        points_world,
        clearance=OBSTACLE_CONTROL_CLEARANCE,
        speed_limit=SOURCE_MAX_SPEED,
        current_velocity=current_velocity,
    )
    if SCENARIO.safety_min is not None and SCENARIO.safety_max is not None:
        result = flight_volume_filter(
            result,
            position,
            SCENARIO.safety_min,
            SCENARIO.safety_max,
            clearance=OBSTACLE_CONTROL_CLEARANCE,
            speed_limit=SOURCE_MAX_SPEED,
            current_velocity=current_velocity,
        )
    result = sweep_obstacle_filter(
        result,
        sweep_constraints,
        speed_limit=SOURCE_MAX_SPEED,
        current_velocity=current_velocity,
        clearance=SCENE_QUERY_CLEARANCE,
    )
    return np.asarray(result, dtype=float)


def _backend_array_to_numpy(value) -> np.ndarray:
    if isinstance(value, np.ndarray):
        return value
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    if hasattr(value, "cpu"):
        host = value.cpu()
        if hasattr(host, "numpy"):
            return np.asarray(host.numpy())
    return np.asarray(value)


def _add_fixed_cube(
    world: World,
    path: str,
    center: Sequence[float],
    size: Sequence[float],
    color: Sequence[float],
) -> FixedCuboid:
    return world.scene.add(
        FixedCuboid(
            prim_path=path,
            name=path.rsplit("/", 1)[-1],
            position=np.asarray(center, dtype=float),
            scale=np.asarray(size, dtype=float),
            color=np.asarray(color, dtype=float),
        )
    )


def _add_crazyflie(
    world: World, stage, drone_id: int, start: Sequence[float]
) -> Tuple[
    SingleRigidPrim,
    RotatingLidarPhysX,
    None,
    Tuple[ContactSensor, ...],
]:
    root_path = f"/World/Drones/drone_{drone_id}"
    root = UsdGeom.Xform.Define(stage, root_path)
    UsdPhysics.RigidBodyAPI.Apply(root.GetPrim())
    UsdPhysics.MassAPI.Apply(root.GetPrim()).CreateMassAttr(CRAZYFLIE_MASS)
    rigid_api = PhysxSchema.PhysxRigidBodyAPI.Apply(root.GetPrim())
    rigid_api.CreateDisableGravityAttr(False)
    rigid_api.CreateLinearDampingAttr(0.02)
    rigid_api.CreateAngularDampingAttr(0.02)
    PhysxSchema.PhysxContactReportAPI.Apply(
        root.GetPrim()
    ).CreateThresholdAttr(0.0)

    collider = UsdGeom.Cube.Define(stage, root_path + "/body")
    collider.CreateSizeAttr(1.0)
    collider.AddScaleOp().Set(Gf.Vec3f(*BODY_SIZE))
    collider.CreateDisplayOpacityAttr([0.0])
    UsdPhysics.CollisionAPI.Apply(collider.GetPrim())
    PhysxSchema.PhysxContactReportAPI.Apply(
        collider.GetPrim()
    ).CreateThresholdAttr(0.0)

    if USE_REFERENCE_VISUAL and CRAZYFLIE_ASSET.is_file():
        visual = UsdGeom.Xform.Define(stage, root_path + "/crazyflie_visual")
        visual.GetPrim().GetReferences().AddReference(str(CRAZYFLIE_ASSET))
        visual.AddRotateXOp().Set(90.0)

    body = world.scene.add(
        SingleRigidPrim(
            prim_path=root_path,
            name=f"crazyflie_3d_{drone_id}",
            position=np.asarray(start, dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0)),
            mass=CRAZYFLIE_MASS,
            reset_xform_properties=True,
        )
    )
    lidar = world.scene.add(
        RotatingLidarPhysX(
            prim_path=root_path + "/lidar",
            name=f"lidar_3d_{drone_id}",
            translation=LIDAR_TRANSLATION,
            rotation_frequency=0.0,
            fov=(360.0, 120.0),
            resolution=(3.0, 5.0),
            # Exclude the Crazyflie's own 0.16 m collision body. Real vehicle
            # drivers apply the same body/self point-cloud mask.
            valid_range=(0.25, 7.0),
        )
    )
    contact = world.scene.add(
        ContactSensor(
            prim_path=root_path + "/body/contact_sensor",
            name=f"contact_3d_{drone_id}",
            dt=PHYSICS_DT,
            min_threshold=1.0e-4,
            max_threshold=1.0e6,
            radius=-1.0,
        )
    )
    return body, lidar, None, (contact,)


def _add_racer_so3(
    world: World, stage, drone_id: int, start: Sequence[float]
) -> Tuple[
    SingleRigidPrim,
    Camera,
    RotatingLidarPhysX,
    Tuple[ContactSensor, ...],
]:
    """Reference the generated RACER asset and attach runtime sensors."""

    root_path = f"/World/Drones/drone_{drone_id}"
    root = UsdGeom.Xform.Define(stage, root_path)
    root.GetPrim().GetReferences().AddReference(str(ARGS.vehicle_usd))
    body_path = root_path + "/base_link"
    body_prim = stage.GetPrimAtPath(body_path)
    if not body_prim.IsValid() or not body_prim.HasAPI(
        UsdPhysics.RigidBodyAPI
    ):
        raise RuntimeError(
            f"generated SO3 asset has no rigid body at {body_path}"
        )
    if ARGS.headless:
        # PhysX lidar still needs periodic Kit updates, but compiling three
        # copies of the imported OmniPBR materials can trigger a Vulkan
        # device-lost fault on the 8 GB test GPU. Keep all collision and
        # dynamics prims active while skipping only headless vehicle drawing.
        # The ROS1 ideal renderer never included the carrying vehicle in its
        # depth image, whereas an RTX camera at the source's zero translation
        # would otherwise start inside the imported fuselage mesh.
        UsdGeom.Imageable(
            stage.GetPrimAtPath(body_path + "/visuals")
        ).MakeInvisible()
    PhysxSchema.PhysxContactReportAPI.Apply(
        body_prim
    ).CreateThresholdAttr(0.0)

    body = world.scene.add(
        SingleRigidPrim(
            prim_path=body_path,
            name=f"racer_so3_3d_{drone_id}",
            position=np.asarray(start, dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0)),
            reset_xform_properties=True,
        )
    )
    depth_camera = world.scene.add(
        Camera(
            prim_path=body_path + "/depth_camera",
            name=f"racer_depth_camera_{drone_id}",
            frequency=ARGS.sensor_rate_hz,
            resolution=(DEPTH_WIDTH, DEPTH_HEIGHT),
            translation=CAMERA_TRANSLATION,
            # Identity in Isaac's robotics camera convention makes camera
            # +Z optical point along body +X, exactly matching cam02body in
            # the upstream pcl_render_node.
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0)),
        )
    )
    depth_camera.set_opencv_pinhole_properties(
        cx=DEPTH_CX,
        cy=DEPTH_CY,
        fx=DEPTH_FX,
        fy=DEPTH_FY,
        pinhole=[0.0] * 12,
    )
    depth_camera.set_clipping_range(
        # The mapper discards source measurements below 0.2 m. Matching that
        # usable near range also prevents imported self geometry inside the
        # ideal camera's blind zone from becoming a false obstacle.
        near_distance=DEPTH_MIN_RANGE,
        far_distance=DEPTH_RENDER_HORIZON,
    )
    # The upstream forward depth camera remains the exploration sensor. A
    # coarse full-sphere PhysX lidar supplies the near-field safety layer so
    # thin rack posts cannot disappear in lateral/rear camera blind zones.
    safety_lidar = world.scene.add(
        RotatingLidarPhysX(
            prim_path=body_path + "/safety_lidar",
            name=f"racer_safety_lidar_{drone_id}",
            translation=LIDAR_TRANSLATION,
            rotation_frequency=0.0,
            fov=(360.0, 180.0),
            resolution=(
                SAFETY_LIDAR_HORIZONTAL_RESOLUTION_DEG,
                SAFETY_LIDAR_VERTICAL_RESOLUTION_DEG,
            ),
            valid_range=(SELF_FILTER_RADIUS, SAFETY_POINT_MEMORY_RADIUS),
        )
    )

    collision_prims = [
        prim
        for prim in stage.Traverse()
        if str(prim.GetPath()).startswith(body_path + "/")
        and prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    if len(collision_prims) != 7:
        descendants = [
            (
                str(prim.GetPath()),
                prim.GetTypeName(),
                list(prim.GetAppliedSchemas()),
            )
            for prim in stage.Traverse()
            if str(prim.GetPath()).startswith(body_path + "/")
        ]
        raise RuntimeError(
            "generated SO3 asset must expose seven collision prims below "
            f"{body_path}, found "
            f"{[str(prim.GetPath()) for prim in collision_prims]}; "
            f"descendants={descendants}"
        )
    contacts = []
    for collision_index, collision_prim in enumerate(collision_prims):
        collision_path = str(collision_prim.GetPath())
        PhysxSchema.PhysxContactReportAPI.Apply(
            collision_prim
        ).CreateThresholdAttr(0.0)
        contacts.append(
            world.scene.add(
                ContactSensor(
                    prim_path=collision_path
                    + f"/contact_sensor_{collision_index}",
                    name=f"contact_3d_{drone_id}_{collision_index}",
                    dt=PHYSICS_DT,
                    min_threshold=1.0e-4,
                    max_threshold=1.0e6,
                    radius=-1.0,
                )
            )
        )
    return body, depth_camera, safety_lidar, tuple(contacts)


def _add_vehicle(
    world: World, stage, drone_id: int, start: Sequence[float]
) -> Tuple[SingleRigidPrim, object, object, Tuple[ContactSensor, ...]]:
    if ARGS.vehicle_model == "racer_so3":
        return _add_racer_so3(world, stage, drone_id, start)
    return _add_crazyflie(world, stage, drone_id, start)


def build_world():
    world = World(
        physics_dt=PHYSICS_DT,
        # Keep one physics step per control update. A larger render dt makes
        # World.step() perform unactuated gravity substeps on rendered frames.
        rendering_dt=PHYSICS_DT,
        stage_units_in_meters=1.0,
    )
    stage = omni.usd.get_context().get_stage()
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    colors = (
        (0.16, 0.18, 0.21),
        (0.30, 0.35, 0.42),
        (0.38, 0.31, 0.28),
    )
    if ARGS.scene_usd is None:
        for index, obstacle in enumerate(SCENARIO.obstacles):
            _add_fixed_cube(
                world,
                f"/World/Obstacles/{obstacle.name}_{index}",
                obstacle.center,
                obstacle.size,
                colors[min(2, index // 6)],
            )
    else:
        external = UsdGeom.Xform.Define(stage, "/World/ExternalScene")
        external.GetPrim().GetReferences().AddReference(str(ARGS.scene_usd))
    light = UsdLux.DistantLight.Define(stage, "/World/Sun")
    light.CreateIntensityAttr(2600.0)
    light.AddRotateXYZOp().Set(Gf.Vec3f(45.0, -25.0, 20.0))
    camera = UsdGeom.Camera.Define(stage, "/World/OverviewCamera")
    camera.AddTranslateOp().Set(Gf.Vec3d(0.0, -15.0, 12.0))
    camera.AddRotateXYZOp().Set(Gf.Vec3f(55.0, 0.0, 0.0))
    camera.CreateFocalLengthAttr(24.0)

    bodies, range_sensors, safety_sensors, contacts = [], [], [], []
    for drone_id, start in enumerate(STARTS):
        body, range_sensor, safety_sensor, vehicle_contacts = _add_vehicle(
            world, stage, drone_id, start
        )
        bodies.append(body)
        range_sensors.append(range_sensor)
        safety_sensors.append(safety_sensor)
        contacts.append(vehicle_contacts)
    world.reset()
    for body in bodies:
        body._rigid_prim_view.enable_gravities()
        if ARGS.vehicle_model == "racer_so3":
            imported_mass = float(
                _backend_array_to_numpy(
                    body._rigid_prim_view.get_masses()
                ).reshape(-1)[0]
            )
            if not math.isclose(
                imported_mass, RACER_SO3_MASS, rel_tol=1.0e-5
            ):
                raise RuntimeError(
                    f"SO3 USD mass mismatch: {imported_mass}"
                )
        # World.reset() advances initialization physics in Isaac Sim 5.1.
        # Clear that transient before the rotor controller starts.
        body.set_linear_velocity(np.zeros(3, dtype=float))
        body.set_angular_velocity(np.zeros(3, dtype=float))
    for range_sensor in range_sensors:
        if ARGS.vehicle_model == "racer_so3":
            range_sensor.add_distance_to_image_plane_to_frame()
        else:
            range_sensor.add_point_cloud_data_to_frame()
    for safety_sensor in safety_sensors:
        if safety_sensor is not None:
            safety_sensor.add_point_cloud_data_to_frame()
    for vehicle_contacts in contacts:
        for contact in vehicle_contacts:
            contact.add_raw_contact_data_to_frame()
    return world, bodies, range_sensors, safety_sensors, contacts


def _yaw_from_quaternion(quaternion: Sequence[float]) -> float:
    w, x, y, z = (float(value) for value in quaternion)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


class PropellerVisualAnimator:
    """Animate render-only blade Xforms without adding physics articulation."""

    DEGREES_PER_RPM_SECOND = 6.0
    AXIS_INDICES = {"X": 0, "Y": 1, "Z": 2}

    def __init__(
        self,
        stage,
        body_paths: Sequence[str],
        enabled: bool,
        update_hz: float,
        strict: bool = False,
    ) -> None:
        self.enabled = False
        self.update_hz = float(update_hz)
        self.update_period = 1.0 / self.update_hz
        self.accumulated_time = 0.0
        self.angles_deg = np.zeros((len(body_paths), 4), dtype=float)
        self.spin_signs = np.zeros((len(body_paths), 4), dtype=float)
        self.entries = []
        self.status = "disabled"
        if not enabled:
            return

        missing = []
        for drone_id, body_path in enumerate(body_paths):
            visuals = stage.GetPrimAtPath(body_path + "/visuals")
            if not visuals.IsValid():
                missing.append(f"{body_path}/visuals")
                continue
            candidates = []
            for prim in Usd.PrimRange(visuals):
                rotor_id_attr = prim.GetAttribute("racer:rotorId")
                if rotor_id_attr.IsValid() and rotor_id_attr.HasAuthoredValueOpinion():
                    candidates.append((int(rotor_id_attr.Get()), prim))
            candidates.sort(key=lambda item: item[0])
            if [item[0] for item in candidates] != [0, 1, 2, 3]:
                missing.append(
                    f"{body_path}/visuals rotor_ids="
                    f"{[item[0] for item in candidates]}"
                )
                continue

            for rotor_id, prim in candidates:
                sign_attr = prim.GetAttribute("racer:spinDirectionSign")
                axis_attr = prim.GetAttribute("racer:visualRotationAxis")
                op_attr = prim.GetAttribute("racer:visualRotationOp")
                sign = int(sign_attr.Get()) if sign_attr.IsValid() else 0
                axis = str(axis_attr.Get()).upper() if axis_attr.IsValid() else ""
                op_name = str(op_attr.Get()) if op_attr.IsValid() else ""
                rotation_attr = prim.GetAttribute(op_name)
                if sign not in (-1, 1):
                    raise RuntimeError(
                        f"invalid propeller spin sign at {prim.GetPath()}: {sign}"
                    )
                if axis not in self.AXIS_INDICES:
                    raise RuntimeError(
                        f"invalid propeller rotation axis at {prim.GetPath()}: {axis}"
                    )
                if not rotation_attr.IsValid() or rotation_attr.Get() is None:
                    raise RuntimeError(
                        f"propeller rotation op {op_name!r} is missing at "
                        f"{prim.GetPath()}"
                    )
                base_rotation = rotation_attr.Get()
                self.spin_signs[drone_id, rotor_id] = sign
                self.entries.append(
                    {
                        "drone_id": drone_id,
                        "rotor_id": rotor_id,
                        "path": str(prim.GetPath()),
                        "attribute": rotation_attr,
                        "axis_index": self.AXIS_INDICES[axis],
                        "base_rotation": tuple(float(value) for value in base_rotation),
                        "vector_type": type(base_rotation),
                    }
                )

        if missing:
            message = "propeller animation metadata unavailable: " + "; ".join(missing)
            if strict:
                raise RuntimeError(message)
            self.status = message
            self.entries = []
            return

        self.enabled = True
        self.status = "ready"
        self._write_angles()

    def _write_angles(self) -> None:
        for entry in self.entries:
            values = list(entry["base_rotation"])
            values[entry["axis_index"]] += self.angles_deg[
                entry["drone_id"], entry["rotor_id"]
            ]
            entry["attribute"].Set(entry["vector_type"](*values))

    def step(self, motor_rpms: Sequence[Sequence[float]], dt: float) -> None:
        if not self.enabled:
            return
        rpm = np.asarray(motor_rpms, dtype=float)
        if rpm.shape != self.angles_deg.shape:
            raise RuntimeError(
                f"propeller RPM shape {rpm.shape} does not match "
                f"visual shape {self.angles_deg.shape}"
            )
        self.angles_deg = np.remainder(
            self.angles_deg
            + self.spin_signs * rpm * self.DEGREES_PER_RPM_SECOND * float(dt),
            360.0,
        )
        self.accumulated_time += float(dt)
        if self.accumulated_time + 1.0e-12 < self.update_period:
            return
        self.accumulated_time %= self.update_period
        self._write_angles()

    def reset(self) -> None:
        if not self.enabled:
            return
        self.angles_deg.fill(0.0)
        self.accumulated_time = 0.0
        self._write_angles()

    def report(self) -> dict:
        return {
            "status": self.status,
            "enabled": self.enabled,
            "propeller_count": len(self.entries),
            "update_hz": self.update_hz,
            "degrees_per_rpm_second": self.DEGREES_PER_RPM_SECOND,
            "paths": [entry["path"] for entry in self.entries],
        }


class IsaacRacer3DBridge(Node):
    def __init__(self, bodies, range_sensors, safety_sensors, contacts) -> None:
        super().__init__("isaac_racer_3d_bridge")
        self.bodies = bodies
        self.range_sensors = range_sensors
        self.safety_sensors = safety_sensors
        self.contacts = contacts
        self.drone_count = len(bodies)
        self.commands = [np.zeros(3) for _ in bodies]
        self.applied_commands = [np.zeros(3) for _ in bodies]
        self.safety_points = [np.empty((0, 3), dtype=float) for _ in bodies]
        self.scene_query_points = [
            np.empty((0, 3), dtype=float) for _ in bodies
        ]
        self.scene_query_constraints = [[] for _ in bodies]
        self.last_scene_query = [-math.inf for _ in bodies]
        self.scene_query_updates = 0
        self.scene_query_hits = 0
        self.min_sweep_free_travel = math.inf
        self.scene_query = get_physx_scene_query_interface()
        # A single forward depth frame forgets a shelf as soon as it leaves
        # the camera frustum. Keep a short, world-frame, voxelized history so
        # the 1 kHz rigid-body controller can still brake beside/behind it.
        self.safety_point_history = [deque() for _ in bodies]
        self.yaw_commands = [0.0 for _ in bodies]
        self.yaw_targets = [0.0 for _ in bodies]
        self.positions = [np.zeros(3) for _ in bodies]
        self.velocities = [np.zeros(3) for _ in bodies]
        self.accelerations = [np.zeros(3) for _ in bodies]
        self.previous_velocities = [None for _ in bodies]
        self.path_lengths = [0.0 for _ in bodies]
        self.previous_positions = [None for _ in bodies]
        self.motor_thrusts = [np.zeros(4) for _ in bodies]
        self.motor_rpms = [
            np.full(4, racer_hover_rpm())
            if ARGS.vehicle_model == "racer_so3"
            else np.zeros(4)
            for _ in bodies
        ]
        self.propeller_visuals = PropellerVisualAnimator(
            omni.usd.get_context().get_stage(),
            [
                f"/World/Drones/drone_{drone_id}/base_link"
                for drone_id in range(self.drone_count)
            ],
            enabled=(ANIMATE_PROPELLERS and ARGS.vehicle_model == "racer_so3"),
            update_hz=ARGS.propeller_visual_hz,
            strict=(ARGS.animate_propellers is True),
        )
        print(
            "RACER_3D_PROPELLER_VISUALS "
            + json.dumps(self.propeller_visuals.report(), sort_keys=True),
            flush=True,
        )
        self.elapsed = 0.0
        self.last_odom = -math.inf
        self.last_depth = -math.inf
        self.collision_events = 0
        self.contact_active = [False for _ in bodies]
        self.max_contact_force = 0.0
        self.min_inter_drone = math.inf
        self.min_obstacle_clearance = math.inf
        self.cloud_frames = 0
        self.raw_diagnostics_printed = False
        self.control_steps = 0
        self.safety_interventions = 0
        self.mission_complete = False
        self.mapping_coverage = [None for _ in bodies]
        self.mapping_coverage_counts = [None for _ in bodies]
        self.mapping_coverage_history = []
        self.mapping_coverage_history_last_stamp = [
            -math.inf for _ in bodies
        ]
        self.mapping_coverage_target_reached = False
        self.trajectory_history = []
        self.last_trajectory_history_stamp = -math.inf
        self.debug_draw = None
        self.visual_map_points = np.empty((0, 3), dtype=np.float32)
        self.visual_paths = [np.empty((0, 3), dtype=np.float32) for _ in bodies]
        self.visual_markers = {}
        self.visual_trails = [
            [tuple(float(value) for value in STARTS[index])]
            for index in range(self.drone_count)
        ]
        self.visualization_dirty = False
        self.last_visualization_update = -math.inf
        self.last_trail_update = -math.inf
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        cloud_qos = QoSProfile(
            depth=5, reliability=ReliabilityPolicy.BEST_EFFORT
        )
        self.odom_publishers = []
        self.imu_publishers = []
        self.cloud_publishers = []
        for drone_id in range(self.drone_count):
            namespace = f"/drone_{drone_id}"
            self.odom_publishers.append(
                self.create_publisher(Odometry, namespace + "/odom", qos)
            )
            self.imu_publishers.append(
                self.create_publisher(Imu, namespace + "/imu", qos)
            )
            self.cloud_publishers.append(
                self.create_publisher(
                    PointCloud2, namespace + "/points", cloud_qos
                )
            )
            self.create_subscription(
                Twist,
                namespace + "/cmd_vel_3d",
                lambda message, index=drone_id: self._command(index, message),
                qos,
            )
        if ARGS.visualize_exploration:
            self.debug_draw = _debug_draw.acquire_debug_draw_interface()
            visual_qos = QoSProfile(
                depth=1, reliability=ReliabilityPolicy.RELIABLE
            )
            for drone_id in range(self.drone_count):
                for channel in (
                    "trajectory",
                    "frontier",
                    "viewpoints",
                    "travel_traj",
                    "position_cmd",
                    "map_chunks",
                ):
                    self.create_subscription(
                        Marker,
                        f"/drone_{drone_id}/planning_vis/{channel}",
                        lambda message, index=drone_id, name=channel:
                            self._visual_marker(index, name, message),
                        visual_qos,
                    )
        self.metrics_publisher = self.create_publisher(
            String, "/racer_3d/sim_metrics", qos
        )
        # The 200 Hz source-faithful plant can run slower than wall time.
        # Exploration trajectories and peer timeouts must therefore use the
        # physics clock, otherwise their time axis advances while the vehicle
        # has not yet completed the corresponding PhysX motion.
        self.clock_publisher = self.create_publisher(
            Clock, "/clock", QoSProfile(depth=10)
        )
        self.create_subscription(
            String,
            "/racer_3d/mission_complete",
            self._mission_complete,
            qos,
        )
        for drone_id in range(self.drone_count):
            self.create_subscription(
                String,
                (
                    f"/racer_original_exploration_{drone_id + 1}"
                    "/mapping_coverage"
                ),
                lambda message, index=drone_id:
                    self._mapping_coverage(index, message),
                qos,
            )
        self._read_physics(count_distance=False)

    def _visual_map(self, message: PointCloud2) -> None:
        points, _ = read_xyzi_cloud(message)
        if len(points) > ARGS.visualization_max_map_points:
            indices = np.linspace(
                0,
                len(points) - 1,
                ARGS.visualization_max_map_points,
                dtype=np.int64,
            )
            points = points[indices]
        self.visual_map_points = np.asarray(points, dtype=np.float32)
        self.visualization_dirty = True

    def _visual_path(self, drone_id: int, message: RosPath) -> None:
        self.visual_paths[drone_id] = np.asarray(
            [
                (
                    pose.pose.position.x,
                    pose.pose.position.y,
                    pose.pose.position.z,
                )
                for pose in message.poses
            ],
            dtype=np.float32,
        ).reshape((-1, 3))
        self.visualization_dirty = True

    def _visual_marker(
        self, drone_id: int, channel: str, message: Marker
    ) -> None:
        key = (drone_id, channel, message.ns, int(message.id))
        if message.action == Marker.DELETEALL:
            self.visual_markers = {
                item_key: value
                for item_key, value in self.visual_markers.items()
                if item_key[0] != drone_id or item_key[1] != channel
            }
        elif message.action == Marker.DELETE:
            self.visual_markers.pop(key, None)
        else:
            points = np.asarray(
                [(point.x, point.y, point.z) for point in message.points],
                dtype=np.float32,
            ).reshape((-1, 3))
            if not len(points) and message.type in (Marker.CUBE, Marker.SPHERE):
                points = np.asarray(
                    [[message.pose.position.x, message.pose.position.y,
                      message.pose.position.z]], dtype=np.float32
                )
            color = (
                float(message.color.r), float(message.color.g),
                float(message.color.b), max(0.15, float(message.color.a)),
            )
            self.visual_markers[key] = {
                "type": int(message.type),
                "points": points,
                "color": color,
                "width": max(1.0, 20.0 * float(message.scale.x)),
            }
        self.visualization_dirty = True

    @staticmethod
    def _line_segments(points: Sequence[Sequence[float]]):
        if len(points) < 2:
            return [], []
        values = np.asarray(points, dtype=float).reshape((-1, 3))
        return values[:-1].tolist(), values[1:].tolist()

    def update_visualization(self, force: bool = False) -> None:
        if self.debug_draw is None:
            return
        if self.elapsed - self.last_trail_update >= 0.20:
            for drone_id, position in enumerate(self.positions):
                self.visual_trails[drone_id].append(
                    tuple(float(value) for value in position)
                )
                if len(self.visual_trails[drone_id]) > 2500:
                    self.visual_trails[drone_id] = self.visual_trails[drone_id][
                        -2500:
                    ]
            self.last_trail_update = self.elapsed
            self.visualization_dirty = True
        if not self.visualization_dirty:
            return
        if not force and self.elapsed - self.last_visualization_update < 0.20:
            return

        self.debug_draw.clear_points()
        self.debug_draw.clear_lines()
        if len(self.visual_map_points):
            points = self.visual_map_points.tolist()
            self.debug_draw.draw_points(
                points,
                [VISUAL_MAP_COLOR] * len(points),
                [3.0] * len(points),
            )
        marker_points, marker_colors, marker_sizes = [], [], []
        for marker in self.visual_markers.values():
            if marker["type"] in (
                Marker.CUBE, Marker.SPHERE, Marker.CUBE_LIST,
                Marker.SPHERE_LIST, Marker.POINTS,
            ):
                values = marker["points"].tolist()
                marker_points.extend(values)
                marker_colors.extend([marker["color"]] * len(values))
                marker_sizes.extend([marker["width"]] * len(values))
        if marker_points:
            self.debug_draw.draw_points(
                marker_points, marker_colors, marker_sizes
            )
        drone_points = [position.tolist() for position in self.positions]
        drone_colors = [
            VISUAL_DRONE_COLORS[index % len(VISUAL_DRONE_COLORS)]
            for index in range(self.drone_count)
        ]
        self.debug_draw.draw_points(
            drone_points, drone_colors, [14.0] * self.drone_count
        )

        starts, ends, colors, widths = [], [], [], []
        for drone_id in range(self.drone_count):
            color = VISUAL_DRONE_COLORS[
                drone_id % len(VISUAL_DRONE_COLORS)
            ]
            trail_starts, trail_ends = self._line_segments(
                self.visual_trails[drone_id]
            )
            starts.extend(trail_starts)
            ends.extend(trail_ends)
            colors.extend(
                [(color[0], color[1], color[2], 0.55)]
                * len(trail_starts)
            )
            widths.extend([2.0] * len(trail_starts))
            path_starts, path_ends = self._line_segments(
                self.visual_paths[drone_id]
            )
            starts.extend(path_starts)
            ends.extend(path_ends)
            colors.extend([color] * len(path_starts))
            widths.extend([4.0] * len(path_starts))
        for marker in self.visual_markers.values():
            points = marker["points"]
            if marker["type"] == Marker.LINE_LIST:
                count = len(points) - len(points) % 2
                marker_starts = points[:count:2].tolist()
                marker_ends = points[1:count:2].tolist()
            elif marker["type"] == Marker.LINE_STRIP:
                marker_starts, marker_ends = self._line_segments(points)
            else:
                continue
            starts.extend(marker_starts)
            ends.extend(marker_ends)
            colors.extend([marker["color"]] * len(marker_starts))
            widths.extend([marker["width"]] * len(marker_starts))
        if starts:
            self.debug_draw.draw_lines(starts, ends, colors, widths)
        self.last_visualization_update = self.elapsed
        self.visualization_dirty = False

    def _command(self, drone_id: int, message: Twist) -> None:
        self.commands[drone_id] = np.asarray(
            (message.linear.x, message.linear.y, message.linear.z), dtype=float
        )
        self.yaw_targets[drone_id] = float(message.angular.z)

    def _mission_complete(self, message: String) -> None:
        self.mission_complete = message.data.strip().lower() == "true"

    def _mapping_coverage(self, drone_id: int, message: String) -> None:
        try:
            payload = json.loads(message.data)
            ratio = float(payload["ratio"])
            known = int(payload["known_voxels"])
            total = int(payload["total_voxels"])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError):
            return
        if not (0.0 <= ratio <= 1.0) or known < 0 or total <= 0:
            return
        self.mapping_coverage[drone_id] = ratio
        self.mapping_coverage_counts[drone_id] = {
            "known_voxels": known,
            "total_voxels": total,
        }
        if (
            self.elapsed
            - self.mapping_coverage_history_last_stamp[drone_id]
            >= 1.0
        ):
            self.mapping_coverage_history.append(
                {
                    "time_s": self.elapsed,
                    "drone_id": drone_id,
                    "ratio": ratio,
                    "known_voxels": known,
                    "total_voxels": total,
                }
            )
            self.mapping_coverage_history_last_stamp[drone_id] = self.elapsed
        valid = [value for value in self.mapping_coverage if value is not None]
        self.mapping_coverage_target_reached = bool(
            ARGS.mapping_coverage_target > 0.0
            and valid
            and max(valid) >= ARGS.mapping_coverage_target
        )

    def _query_external_safety_points(
        self,
        drone_id: int,
        position: Sequence[float],
        velocity: Sequence[float],
        command: Sequence[float],
    ) -> np.ndarray:
        """Sweep the real vehicle envelope through nearby PhysX geometry.

        This is an execution-only collision sensor.  Its hit points are never
        published on ``/racer/sensor_points`` and therefore cannot change any
        frontier, HGrid, LKH, kinodynamic, B-spline, yaw, or FSM decision.
        """

        directions = [
            np.asarray((1.0, 0.0, 0.0)),
            np.asarray((-1.0, 0.0, 0.0)),
            np.asarray((0.0, 1.0, 0.0)),
            np.asarray((0.0, -1.0, 0.0)),
            np.asarray((0.0, 0.0, 1.0)),
            np.asarray((0.0, 0.0, -1.0)),
        ]
        for candidate in (velocity, command):
            value = np.asarray(candidate, dtype=float)
            norm = float(np.linalg.norm(value))
            if norm > 1.0e-4:
                directions.append(value / norm)

        unique_directions = []
        seen = set()
        for direction in directions:
            key = tuple(np.round(direction, decimals=4))
            if key not in seen:
                seen.add(key)
                unique_directions.append(direction)

        own_prefix = f"/World/Drones/drone_{drone_id}/"
        hits = []
        constraints = []
        for direction in unique_directions:
            direction_hits = []

            def report(hit):
                rigid_body = str(hit.rigid_body)
                collision = str(getattr(hit, "collision", ""))
                if not (
                    rigid_body.startswith(own_prefix)
                    or collision.startswith(own_prefix)
                ):
                    point = np.asarray(hit.position, dtype=float)
                    if np.all(np.isfinite(point)):
                        hits.append(point)
                    distance = float(getattr(hit, "distance", math.inf))
                    if math.isfinite(distance) and distance >= 0.0:
                        direction_hits.append(distance)
                return True

            self.scene_query.sweep_sphere_all(
                VEHICLE_RADIUS,
                Gf.Vec3f(*np.asarray(position, dtype=float)),
                Gf.Vec3f(*direction),
                SCENE_QUERY_RANGE,
                report,
            )
            if direction_hits:
                nearest_travel = min(direction_hits)
                constraints.append(
                    (np.asarray(direction, dtype=float), nearest_travel)
                )
                self.min_sweep_free_travel = min(
                    self.min_sweep_free_travel, nearest_travel
                )

        self.scene_query_updates += 1
        self.scene_query_hits += len(hits)
        self.scene_query_constraints[drone_id] = constraints
        if not hits:
            return np.empty((0, 3), dtype=float)
        points = np.asarray(hits, dtype=float).reshape((-1, 3))
        voxels = np.floor(points / SAFETY_POINT_VOXEL_SIZE).astype(np.int64)
        _, unique_indices = np.unique(voxels, axis=0, return_index=True)
        return points[np.sort(unique_indices)]

    def _execution_safety_points(self, drone_id: int) -> np.ndarray:
        sources = (
            self.safety_points[drone_id],
            self.scene_query_points[drone_id],
        )
        nonempty = [points for points in sources if len(points)]
        if not nonempty:
            return np.empty((0, 3), dtype=float)
        return np.concatenate(nonempty, axis=0)

    def apply_motor_wrenches(self) -> None:
        self.control_steps += 1
        phase_checkpoint = (
            ARGS.diagnostics
            and (
                self.control_steps in (1, 10, 50, 250, 500, 1000, 1500)
                or self.control_steps % 500 == 0
            )
        )
        states = []
        for body in self.bodies:
            position, orientation = body.get_world_pose()
            states.append(
                (
                    np.asarray(position, dtype=float),
                    orientation,
                    np.asarray(body.get_linear_velocity(), dtype=float),
                )
            )
        for drone_id, body in enumerate(self.bodies):
            if phase_checkpoint:
                print(
                    f"RACER_3D_PHASE step={self.control_steps} "
                    f"phase=control_drone_{drone_id}_begin",
                    flush=True,
                )
            yaw_error = (
                self.yaw_targets[drone_id]
                - self.yaw_commands[drone_id]
                + math.pi
            ) % (2.0 * math.pi) - math.pi
            self.yaw_commands[drone_id] += float(
                np.clip(
                    yaw_error,
                    -SOURCE_MAX_YAW_RATE * PHYSICS_DT,
                    SOURCE_MAX_YAW_RATE * PHYSICS_DT,
                )
            )
            position, orientation, velocity = states[drone_id]
            if (
                ARGS.scene_usd is not None
                and self.elapsed - self.last_scene_query[drone_id]
                >= SCENE_QUERY_PERIOD - 1.0e-9
            ):
                self.scene_query_points[
                    drone_id
                ] = self._query_external_safety_points(
                    drone_id,
                    position,
                    velocity,
                    self.commands[drone_id],
                )
                self.last_scene_query[drone_id] = self.elapsed
            if phase_checkpoint:
                print(
                    f"RACER_3D_PHASE step={self.control_steps} "
                    f"phase=control_drone_{drone_id}_scene_query_done",
                    flush=True,
                )
            execution_safety_points = self._execution_safety_points(drone_id)
            applied_command = self.commands[drone_id]
            if ARGS.scene_usd is None:
                applied_command = np.asarray(
                    aabb_obstacle_filter(
                        applied_command,
                        position,
                        SCENARIO.obstacles,
                        clearance=OBSTACLE_CONTROL_CLEARANCE,
                        speed_limit=SOURCE_MAX_SPEED,
                        current_velocity=velocity,
                    ),
                    dtype=float,
                )
            else:
                applied_command = _external_obstacle_filter(
                    applied_command,
                    position,
                    execution_safety_points,
                    velocity,
                    self.scene_query_constraints[drone_id],
                )
            applied_command = np.asarray(
                cbf_swarm_filter(
                    applied_command,
                    position,
                    [
                        (peer_id, peer_position, peer_velocity)
                        for peer_id, (
                            peer_position,
                            _,
                            peer_velocity,
                        ) in enumerate(states)
                        if peer_id != drone_id
                    ],
                    safe_distance=SWARM_CONTROL_DISTANCE,
                    speed_limit=SOURCE_MAX_SPEED,
                    current_velocity=velocity,
                ),
                dtype=float,
            )
            # Pairwise projection can point toward a nearby wall; make the
            # obstacle barrier the final authority on the combined command.
            if ARGS.scene_usd is None:
                applied_command = np.asarray(
                    aabb_obstacle_filter(
                        applied_command,
                        position,
                        SCENARIO.obstacles,
                        clearance=OBSTACLE_CONTROL_CLEARANCE,
                        speed_limit=SOURCE_MAX_SPEED,
                        current_velocity=velocity,
                    ),
                    dtype=float,
                )
            else:
                applied_command = _external_obstacle_filter(
                    applied_command,
                    position,
                    execution_safety_points,
                    velocity,
                    self.scene_query_constraints[drone_id],
                )
            if float(
                np.linalg.norm(applied_command - self.commands[drone_id])
            ) > 1.0e-3:
                self.safety_interventions += 1
            self.applied_commands[drone_id] = applied_command
            if ARGS.vehicle_model == "racer_so3":
                wrench = velocity_motor_wrench(
                    applied_command,
                    velocity,
                    orientation,
                    body.get_angular_velocity(),
                    self.yaw_commands[drone_id],
                    self.motor_rpms[drone_id],
                    PHYSICS_DT,
                )
                self.motor_rpms[drone_id] = wrench.motor_rpm
            else:
                wrench = velocity_wrench(
                    applied_command,
                    velocity,
                    orientation,
                    body.get_angular_velocity(),
                    self.yaw_commands[drone_id],
                )
            # Tensor force commands are one-substep values. Submit force and
            # torque together so neither component overwrites the other.
            body._rigid_prim_view.apply_forces_and_torques_at_pos(
                forces=wrench.local_force.reshape((1, 3)).astype(np.float32),
                torques=wrench.local_torque.reshape((1, 3)).astype(np.float32),
                is_global=False,
            )
            self.motor_thrusts[drone_id] = wrench.motor_thrusts
            if (
                ARGS.diagnostics
                and drone_id == 0
                and self.control_steps in (1, 10, 50, 250, 500, 1000, 1500)
            ):
                diag_position, diag_orientation = body.get_world_pose()
                print(
                    "RACER_3D_CONTROL "
                    + json.dumps(
                        {
                            "step": self.control_steps,
                            "mass": float(
                                _backend_array_to_numpy(
                                    body._rigid_prim_view.get_masses()
                                ).reshape(-1)[0]
                            ),
                            "position": np.asarray(diag_position).tolist(),
                            "orientation_wxyz": np.asarray(
                                diag_orientation
                            ).tolist(),
                            "velocity": np.asarray(
                                velocity
                            ).tolist(),
                            "requested_command": self.commands[
                                drone_id
                            ].tolist(),
                            "applied_command": applied_command.tolist(),
                            "angular_velocity": np.asarray(
                                body.get_angular_velocity()
                            ).tolist(),
                            "local_force": wrench.local_force.tolist(),
                            "local_torque": wrench.local_torque.tolist(),
                            "motors": wrench.motor_thrusts.tolist(),
                            "motor_rpm": (
                                self.motor_rpms[drone_id].tolist()
                                if ARGS.vehicle_model == "racer_so3"
                                else None
                            ),
                            "visual_propeller_angles_deg": (
                                self.propeller_visuals.angles_deg[
                                    drone_id
                                ].tolist()
                                if self.propeller_visuals.enabled
                                else None
                            ),
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            if phase_checkpoint:
                print(
                    f"RACER_3D_PHASE step={self.control_steps} "
                    f"phase=control_drone_{drone_id}_done",
                    flush=True,
                )
        self.propeller_visuals.step(self.motor_rpms, PHYSICS_DT)
        if phase_checkpoint:
            print(
                f"RACER_3D_PHASE step={self.control_steps} "
                "phase=control_all_done",
                flush=True,
            )

    def _read_physics(self, count_distance: bool = True) -> None:
        for drone_id, body in enumerate(self.bodies):
            position, _ = body.get_world_pose()
            velocity = np.asarray(body.get_linear_velocity(), dtype=float)
            position = np.asarray(position, dtype=float)
            if count_distance and self.previous_positions[drone_id] is not None:
                step = float(
                    np.linalg.norm(
                        position - self.previous_positions[drone_id]
                    )
                )
                if step < 0.20:
                    self.path_lengths[drone_id] += step
            self.previous_positions[drone_id] = position.copy()
            previous_velocity = self.previous_velocities[drone_id]
            self.accelerations[drone_id] = (
                np.zeros(3, dtype=float)
                if previous_velocity is None
                else (velocity - previous_velocity) / PHYSICS_DT
            )
            self.previous_velocities[drone_id] = velocity.copy()
            self.positions[drone_id] = position
            self.velocities[drone_id] = velocity

    def _update_metrics(self) -> None:
        distances = list(pairwise_distances(self.positions))
        if distances:
            self.min_inter_drone = min(
                self.min_inter_drone, min(distances)
            )
        if ARGS.scene_usd is None:
            for position in self.positions:
                self.min_obstacle_clearance = min(
                    self.min_obstacle_clearance,
                    obstacle_clearance(
                        position, SCENARIO.obstacles
                    )
                    - VEHICLE_RADIUS,
                )
        else:
            for drone_id, position in enumerate(self.positions):
                points = self._execution_safety_points(drone_id)
                if len(points):
                    self.min_obstacle_clearance = min(
                        self.min_obstacle_clearance,
                        float(
                            np.min(
                                np.linalg.norm(points - position, axis=1)
                            )
                        )
                        - VEHICLE_RADIUS,
                    )
        for drone_id, sensors in enumerate(self.contacts):
            frames = [sensor.get_current_frame() for sensor in sensors]
            force = max(
                (float(frame.get("force", 0.0)) for frame in frames),
                default=0.0,
            )
            active = any(
                bool(frame.get("in_contact", False))
                and float(frame.get("force", 0.0)) > 1.0e-4
                for frame in frames
            )
            if active and not self.contact_active[drone_id]:
                self.collision_events += 1
                nearest_name = "external_usd"
                center_clearance = None
                body_clearance = None
                if ARGS.scene_usd is None:
                    clearances = [
                        point_box_signed_clearance(
                            self.positions[drone_id], obstacle
                        )
                        for obstacle in SCENARIO.obstacles
                    ]
                    nearest_index = int(np.argmin(clearances))
                    nearest_name = SCENARIO.obstacles[
                        nearest_index
                    ].name
                    center_clearance = clearances[nearest_index]
                    body_clearance = center_clearance - DRONE_RADIUS
                print(
                    "RACER_3D_CONTACT "
                    + json.dumps(
                        {
                            "drone_id": drone_id,
                            "elapsed": self.elapsed,
                            "position": self.positions[drone_id].tolist(),
                            "velocity": self.velocities[drone_id].tolist(),
                            "command": self.commands[drone_id].tolist(),
                            "applied_command": self.applied_commands[
                                drone_id
                            ].tolist(),
                            "force": force,
                            "sweep_constraints": [
                                {
                                    "direction": direction.tolist(),
                                    "free_travel": distance,
                                }
                                for direction, distance
                                in self.scene_query_constraints[drone_id]
                            ],
                            "nearest_obstacle": nearest_name,
                            "center_clearance": center_clearance,
                            "body_clearance": body_clearance,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
            self.contact_active[drone_id] = active
            self.max_contact_force = max(self.max_contact_force, force)

    def step_observations(self) -> None:
        self.elapsed += PHYSICS_DT
        seconds = int(math.floor(self.elapsed))
        nanoseconds = int(round((self.elapsed - seconds) * 1.0e9))
        if nanoseconds >= 1_000_000_000:
            seconds += 1
            nanoseconds -= 1_000_000_000
        clock = Clock()
        clock.clock.sec = seconds
        clock.clock.nanosec = nanoseconds
        self.clock_publisher.publish(clock)
        self._read_physics()
        self._update_metrics()
        # This bridge intentionally does not consume its own /clock topic.
        # Stamp every source message directly with the physics time just
        # published above so ROS2 agents using sim time see one time domain.
        stamp = clock.clock
        if self.elapsed - self.last_odom >= ODOM_PERIOD - 1.0e-9:
            self.last_odom = self.elapsed
            self._publish_odometry_and_imu(stamp)
        if self.elapsed - self.last_depth >= DEPTH_PERIOD - 1.0e-9:
            self.last_depth = self.elapsed
            self._publish_clouds(stamp)

    def depth_render_due(self) -> bool:
        return self.elapsed - self.last_depth >= DEPTH_PERIOD - PHYSICS_DT

    def _publish_odometry_and_imu(self, stamp) -> None:
        for drone_id, body in enumerate(self.bodies):
            position, orientation = body.get_world_pose()
            velocity = body.get_linear_velocity()
            angular = body.get_angular_velocity()
            message = Odometry()
            message.header.stamp = stamp
            message.header.frame_id = "map"
            message.child_frame_id = f"drone_{drone_id}/base_link"
            message.pose.pose.position.x = float(position[0])
            message.pose.pose.position.y = float(position[1])
            message.pose.pose.position.z = float(position[2])
            message.pose.pose.orientation.w = float(orientation[0])
            message.pose.pose.orientation.x = float(orientation[1])
            message.pose.pose.orientation.y = float(orientation[2])
            message.pose.pose.orientation.z = float(orientation[3])
            message.twist.twist.linear.x = float(velocity[0])
            message.twist.twist.linear.y = float(velocity[1])
            message.twist.twist.linear.z = float(velocity[2])
            message.twist.twist.angular.x = float(angular[0])
            message.twist.twist.angular.y = float(angular[1])
            message.twist.twist.angular.z = float(angular[2])
            self.odom_publishers[drone_id].publish(message)

            imu = Imu()
            imu.header.stamp = stamp
            imu.header.frame_id = f"drone_{drone_id}/base_link"
            imu.orientation = message.pose.pose.orientation
            imu.angular_velocity = message.twist.twist.angular
            acceleration = self.accelerations[drone_id]
            imu.linear_acceleration.x = float(acceleration[0])
            imu.linear_acceleration.y = float(acceleration[1])
            imu.linear_acceleration.z = float(acceleration[2])
            self.imu_publishers[drone_id].publish(imu)

    @staticmethod
    def _legacy_lidar_world_points(
        raw_points: np.ndarray,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        from crazyflie_cpp_bridge import quaternion_matrix

        values = np.asarray(raw_points, dtype=float).reshape((-1, 3))
        finite = np.all(np.isfinite(values), axis=1)
        values = values[finite]
        ranges = np.linalg.norm(values, axis=1)
        valid = (ranges > VEHICLE_RADIUS + 0.035) & (ranges <= 7.05)
        values, ranges = values[valid], ranges[valid]
        rotation = quaternion_matrix(orientation)
        sensor_origin = (
            np.asarray(position, dtype=float) + rotation @ LIDAR_TRANSLATION
        )
        world = values @ rotation.T + sensor_origin
        return world.astype(np.float32), ranges < 6.97

    @staticmethod
    def _safety_lidar_world_points(
        raw_points: np.ndarray,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> np.ndarray:
        from crazyflie_cpp_bridge import quaternion_matrix

        values = np.asarray(raw_points, dtype=float).reshape((-1, 3))
        finite = np.all(np.isfinite(values), axis=1)
        values = values[finite]
        ranges = np.linalg.norm(values, axis=1)
        valid = (
            (ranges > SELF_FILTER_RADIUS)
            & (ranges <= SAFETY_POINT_MEMORY_RADIUS + 0.05)
        )
        values = values[valid]
        ranges = ranges[valid]
        if len(values) > SAFETY_LIDAR_POINT_LIMIT:
            nearest = np.argpartition(
                ranges, SAFETY_LIDAR_POINT_LIMIT - 1
            )[:SAFETY_LIDAR_POINT_LIMIT]
            values = values[nearest]
        rotation = quaternion_matrix(orientation)
        sensor_origin = (
            np.asarray(position, dtype=float)
            + rotation @ LIDAR_TRANSLATION
        )
        return (
            values @ rotation.T + sensor_origin
        ).astype(np.float32)

    @staticmethod
    def _depth_world_points(
        depth_image: np.ndarray,
        position: Sequence[float],
        orientation: Sequence[float],
    ) -> Tuple[np.ndarray, np.ndarray]:
        """Project one upstream-equivalent depth frame into the map frame."""

        from crazyflie_cpp_bridge import quaternion_matrix

        depth = np.asarray(depth_image, dtype=np.float32).squeeze()
        if depth.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
            return np.empty((0, 3), dtype=np.float32), np.empty(0, dtype=bool)
        vv, uu = np.mgrid[
            DEPTH_FILTER_MARGIN:DEPTH_HEIGHT - DEPTH_FILTER_MARGIN:DEPTH_SKIP_PIXEL,
            DEPTH_FILTER_MARGIN:DEPTH_WIDTH - DEPTH_FILTER_MARGIN:DEPTH_SKIP_PIXEL,
        ]
        z_measured = depth[vv, uu].reshape(-1)
        u = uu.reshape(-1).astype(np.float32)
        v = vv.reshape(-1).astype(np.float32)

        # Optional diagnostic/performance thinning.  The normal default is at
        # least the complete 640x480, skip-pixel=2 upstream grid, so this branch
        # is not taken in source-fidelity runs.
        if len(z_measured) > ARGS.camera_ray_budget:
            selected = np.linspace(
                0, len(z_measured) - 1, ARGS.camera_ray_budget, dtype=np.int64
            )
            z_measured = z_measured[selected]
            u = u[selected]
            v = v[selected]
        finite_positive = np.isfinite(z_measured) & (z_measured > 0.0)
        keep = ~(finite_positive & (z_measured < DEPTH_MIN_RANGE))
        z_measured, u, v = z_measured[keep], u[keep], v[keep]
        hit = (
            np.isfinite(z_measured)
            & (z_measured > 0.0)
            & (z_measured <= DEPTH_MAP_RANGE)
        )
        z = np.where(hit, z_measured, DEPTH_MAP_RANGE).astype(np.float32)

        # Optical frame: +X right, +Y down, +Z forward.  Upstream cam02body
        # maps that to ROS FLU as [Z, -X, -Y] with zero translation.
        optical_x = (u - MAPPING_CX) * z / MAPPING_FX
        optical_y = (v - MAPPING_CY) * z / MAPPING_FY
        body_points = np.column_stack((z, -optical_x, -optical_y))
        if ARGS.vehicle_model == "racer_so3":
            not_self = (
                np.linalg.norm(body_points, axis=1) > SELF_FILTER_RADIUS
            )
            body_points = body_points[not_self]
            hit = hit[not_self]
        rotation = quaternion_matrix(orientation)
        world = (
            body_points @ rotation.T + np.asarray(position, dtype=float)
        )
        return world.astype(np.float32), hit

    def _publish_clouds(self, stamp) -> None:
        for drone_id, (body, range_sensor) in enumerate(
            zip(self.bodies, self.range_sensors)
        ):
            position, orientation = body.get_world_pose()
            if ARGS.vehicle_model == "racer_so3":
                raw = range_sensor.get_depth()
                if raw is None:
                    continue
                raw = _backend_array_to_numpy(raw)
                points, hit = self._depth_world_points(
                    raw, position, orientation
                )
                # Only the original forward depth-camera rays are published
                # to RACER. The 360-degree lidar belongs exclusively to the
                # independent low-level safety layer and must not alter map,
                # frontier, allocation or trajectory decisions.
                mapping_points = points
                mapping_hit = hit
                safety_hits = np.empty((0, 3), dtype=float)
                safety_frame = self.safety_sensors[
                    drone_id
                ].get_current_frame()
                raw_safety = safety_frame.get("point_cloud")
                if raw_safety is not None:
                    safety_hits = self._safety_lidar_world_points(
                        _backend_array_to_numpy(raw_safety),
                        position,
                        orientation,
                    )
            else:
                raw = range_sensor.get_current_frame().get("point_cloud")
                if raw is None:
                    continue
                raw = _backend_array_to_numpy(raw)
                points, hit = self._legacy_lidar_world_points(
                    raw, position, orientation
                )
                mapping_points = points
                mapping_hit = hit
                safety_hits = np.empty((0, 3), dtype=float)
            if len(points) < 12:
                continue
            camera_hits = np.asarray(mapping_points[mapping_hit], dtype=float)
            current_hits = (
                np.concatenate((camera_hits, safety_hits), axis=0)
                if len(safety_hits)
                else camera_hits
            )
            history = self.safety_point_history[drone_id]
            history.append((self.elapsed, current_hits))
            while (
                history
                and self.elapsed - history[0][0]
                > SAFETY_POINT_MEMORY_SECONDS
            ):
                history.popleft()
            nonempty_history = [
                entry[1] for entry in history if len(entry[1])
            ]
            remembered = (
                np.concatenate(nonempty_history, axis=0)
                if nonempty_history
                else np.empty((0, 3), dtype=float)
            )
            center = np.asarray(position, dtype=float)
            offsets = remembered - center
            valid = (
                np.all(np.isfinite(remembered), axis=1)
                & (
                    np.linalg.norm(offsets, axis=1)
                    <= SAFETY_POINT_MEMORY_RADIUS
                )
            )
            remembered = remembered[valid]
            if len(remembered):
                voxels = np.floor(
                    remembered / SAFETY_POINT_VOXEL_SIZE
                ).astype(np.int64)
                _, unique_indices = np.unique(
                    voxels, axis=0, return_index=True
                )
                remembered = remembered[np.sort(unique_indices)]
                if len(remembered) > SAFETY_POINT_LIMIT:
                    distances = np.linalg.norm(
                        remembered - center, axis=1
                    )
                    nearest = np.argpartition(
                        distances, SAFETY_POINT_LIMIT - 1
                    )[:SAFETY_POINT_LIMIT]
                    remembered = remembered[nearest]
            self.safety_points[drone_id] = remembered
            if self.debug_draw is not None and len(camera_hits):
                combined = np.concatenate(
                    (self.visual_map_points, camera_hits.astype(np.float32)),
                    axis=0,
                )
                voxels = np.floor(combined / 0.12).astype(np.int64)
                _, unique_indices = np.unique(
                    voxels, axis=0, return_index=True
                )
                combined = combined[np.sort(unique_indices)]
                if len(combined) > ARGS.visualization_max_map_points:
                    combined = combined[-ARGS.visualization_max_map_points:]
                self.visual_map_points = combined
                self.visualization_dirty = True
            if ARGS.diagnostics and not self.raw_diagnostics_printed:
                median_error = None
                if ARGS.scene_usd is None:
                    clearances = [
                        abs(
                            obstacle_clearance(
                                point, SCENARIO.obstacles
                            )
                        )
                        for point in points[::max(1, len(points) // 500)]
                    ]
                    median_error = float(np.median(clearances))
                print(
                    "RACER_3D_SENSOR_RAW "
                    + json.dumps(
                        {
                            "sensor": (
                                "depth_plus_360_safety_lidar"
                                if ARGS.vehicle_model == "racer_so3"
                                else "legacy_rotating_lidar"
                            ),
                            "raw_shape": list(raw.shape),
                            "world_count": len(mapping_points),
                            "camera_ray_budget": (
                                ARGS.camera_ray_budget
                                if ARGS.vehicle_model == "racer_so3"
                                else None
                            ),
                            "median_surface_error_m": median_error,
                        },
                        sort_keys=True,
                    ),
                    flush=True,
                )
                self.raw_diagnostics_printed = True
            self.cloud_publishers[drone_id].publish(
                create_xyzi_cloud(stamp, "world", mapping_points, mapping_hit)
            )
            self.cloud_frames += 1

    def publish_metrics(self) -> None:
        if (
            ARGS.record_trajectory_history
            and self.elapsed - self.last_trajectory_history_stamp >= 0.5
        ):
            self.trajectory_history.append(
                {
                    "time_s": self.elapsed,
                    "positions": [
                        position.tolist() for position in self.positions
                    ],
                }
            )
            self.last_trajectory_history_stamp = self.elapsed
        payload = {
            "backend": "isaac_sim_physx_3d",
            "scenario": SCENARIO.name,
            "scene_usd": (
                None if ARGS.scene_usd is None else str(ARGS.scene_usd)
            ),
            "vehicle_model": (
                "RACER SO3 0.98kg plus-quadrotor"
                if ARGS.vehicle_model == "racer_so3"
                else "Crazyflie 2.x 27g six-DOF rigid body"
            ),
            "vehicle_asset_usd": (
                str(ARGS.vehicle_usd)
                if ARGS.vehicle_model == "racer_so3"
                else str(CRAZYFLIE_ASSET)
            ),
            "motion_source": (
                "RACER kf/km mixer, first-order RPM, SO3 torque and drag"
                if ARGS.vehicle_model == "racer_so3"
                else "local rotor thrust and attitude torque"
            ),
            "sensor_source": (
                "Isaac ideal pinhole depth camera plus 360-degree "
                "near-field PhysX safety lidar"
                if ARGS.vehicle_model == "racer_so3"
                else "Isaac RotatingLidarPhysX point cloud"
            ),
            "sensor_parameters": (
                {
                    "width_px": DEPTH_WIDTH,
                    "height_px": DEPTH_HEIGHT,
                    "fx_px": DEPTH_FX,
                    "fy_px": DEPTH_FY,
                    "cx_px": DEPTH_CX,
                    "cy_px": DEPTH_CY,
                    "mapping_fx_px": MAPPING_FX,
                    "mapping_fy_px": MAPPING_FY,
                    "mapping_cx_px": MAPPING_CX,
                    "mapping_cy_px": MAPPING_CY,
                    "render_horizon_m": DEPTH_RENDER_HORIZON,
                    "map_min_range_m": DEPTH_MIN_RANGE,
                    "map_max_range_m": DEPTH_MAP_RANGE,
                    "mapping_min_ray_length_m": MAPPING_MIN_RAY_LENGTH,
                    "mapping_max_ray_length_m": MAPPING_MAX_RAY_LENGTH,
                    "rate_hz": 1.0 / DEPTH_PERIOD,
                    "skip_pixel": DEPTH_SKIP_PIXEL,
                    "point_cloud_ray_budget": ARGS.camera_ray_budget,
                    "self_return_filter_radius_m": SELF_FILTER_RADIUS,
                    "mount_translation_body_m": CAMERA_TRANSLATION.tolist(),
                    "safety_lidar_horizontal_fov_deg": 360.0,
                    "safety_lidar_vertical_fov_deg": 180.0,
                    "safety_lidar_horizontal_resolution_deg": (
                        SAFETY_LIDAR_HORIZONTAL_RESOLUTION_DEG
                    ),
                    "safety_lidar_vertical_resolution_deg": (
                        SAFETY_LIDAR_VERTICAL_RESOLUTION_DEG
                    ),
                    "safety_lidar_min_range_m": SELF_FILTER_RADIUS,
                    "safety_lidar_max_range_m": SAFETY_POINT_MEMORY_RADIUS,
                }
                if ARGS.vehicle_model == "racer_so3"
                else {
                    "type": "legacy_360x120_lidar",
                    "range_m": 7.0,
                }
            ),
            "clock_source": "Isaac physics /clock",
            "elapsed": self.elapsed,
            "collision_events": self.collision_events,
            "physics_contact_events": self.collision_events,
            "max_contact_force": self.max_contact_force,
            "min_inter_drone": (
                self.min_inter_drone
                if math.isfinite(self.min_inter_drone)
                else None
            ),
            "min_obstacle_clearance": (
                self.min_obstacle_clearance
                if math.isfinite(self.min_obstacle_clearance)
                else None
            ),
            "path_lengths": self.path_lengths,
            "start_positions": [list(point) for point in STARTS],
            "positions": [point.tolist() for point in self.positions],
            "motor_thrusts_n": [
                values.tolist() for values in self.motor_thrusts
            ],
            "motor_rpm": [
                values.tolist() for values in self.motor_rpms
            ],
            "vehicle_radius_m": VEHICLE_RADIUS,
            "physics_rate_hz": 1.0 / PHYSICS_DT,
            "odometry_rate_hz": 1.0 / ODOM_PERIOD,
            "imu_rate_hz": 1.0 / ODOM_PERIOD,
            "point_cloud_frames": self.cloud_frames,
            "safety_point_refresh_hz": 1.0 / DEPTH_PERIOD,
            "safety_interventions": self.safety_interventions,
            "scene_query_updates": self.scene_query_updates,
            "scene_query_hits": self.scene_query_hits,
            "min_sweep_free_travel": (
                self.min_sweep_free_travel
                if math.isfinite(self.min_sweep_free_travel)
                else None
            ),
            "low_level_safety": _low_level_safety_description(),
        }
        self.metrics_publisher.publish(
            String(
                data=json.dumps(
                    payload, separators=(",", ":"), allow_nan=False
                )
            )
        )


def main() -> None:
    world, bodies, range_sensors, safety_sensors, contacts = build_world()
    if ARGS.visualize_exploration:
        start_center = np.mean(np.asarray(STARTS, dtype=float), axis=0)
        camera_target = np.asarray(
            (start_center[0], start_center[1] + 15.0, 3.0), dtype=float
        )
        camera_eye = np.asarray(
            (start_center[0], start_center[1] - 25.0, 25.0), dtype=float
        )
        set_camera_view(camera_eye, camera_target)
        simulation_app.update()
    rclpy.init()
    bridge = IsaacRacer3DBridge(
        bodies, range_sensors, safety_sensors, contacts
    )
    if ARGS.vehicle_model == "racer_so3":
        async def wait_for_depth_products() -> None:
            await asyncio.gather(
                *(
                    syntheticdata.sensors.next_render_simulation_async(
                        camera.get_render_product_path(), 10
                    )
                    for camera in range_sensors
                )
            )

        # RTX render products compile asynchronously. Do not begin experiment
        # time until every camera has delivered a correctly sized depth image.
        simulation_app.run_coroutine(wait_for_depth_products())
        for drone_id, camera in enumerate(range_sensors):
            depth = camera.get_depth()
            if depth is None:
                raise RuntimeError(
                    f"depth camera {drone_id} did not produce a frame"
                )
            depth = _backend_array_to_numpy(depth).squeeze()
            if depth.shape != (DEPTH_HEIGHT, DEPTH_WIDTH):
                raise RuntimeError(
                    f"depth camera {drone_id} returned {depth.shape}, "
                    f"expected {(DEPTH_HEIGHT, DEPTH_WIDTH)}"
                )
            finite = depth[np.isfinite(depth) & (depth > 0.0)]
            print(
                "RACER_3D_DEPTH_READY "
                + json.dumps(
                    {
                        "drone_id": drone_id,
                        "shape": list(depth.shape),
                        "finite_pixels": int(len(finite)),
                        "minimum_depth_m": (
                            float(np.min(finite)) if len(finite) else None
                        ),
                        "maximum_depth_m": (
                            float(np.max(finite)) if len(finite) else None
                        ),
                    },
                    sort_keys=True,
                ),
                flush=True,
            )
        # Do not spend experiment time before DDS has connected both consumers
        # of every sensor stream: the unchanged exploration node and the
        # launch-readiness trigger.  This is strictly a ROS 2/Isaac transport
        # barrier; it publishes no planner input and changes no RACER decision.
        graph_deadline = time.monotonic() + 60.0
        last_graph_report = -math.inf
        while simulation_app.is_running():
            rclpy.spin_once(bridge, timeout_sec=0.05)
            odom_subscribers = [
                publisher.get_subscription_count()
                for publisher in bridge.odom_publishers
            ]
            cloud_subscribers = [
                publisher.get_subscription_count()
                for publisher in bridge.cloud_publishers
            ]
            graph_ready = all(
                count >= 2
                for count in odom_subscribers + cloud_subscribers
            )
            now_wall = time.monotonic()
            if graph_ready or now_wall - last_graph_report >= 1.0:
                graph_payload = {
                    "expected_subscribers_per_sensor": 2,
                    "metrics_subscribers": (
                        bridge.metrics_publisher.get_subscription_count()
                    ),
                    "odom_subscribers": odom_subscribers,
                    "point_cloud_subscribers": cloud_subscribers,
                    "ready": graph_ready,
                }
                print(
                    (
                        "RACER_3D_ROS_GRAPH "
                        if graph_ready
                        else "RACER_3D_ROS_GRAPH_WAIT "
                    )
                    + json.dumps(graph_payload, sort_keys=True),
                    flush=True,
                )
                last_graph_report = now_wall
            if graph_ready:
                break
            if now_wall >= graph_deadline:
                raise RuntimeError(
                    "ROS graph did not connect two odometry and point-cloud "
                    "subscribers for every vehicle within 60 seconds"
                )
    # Prime range sensors while each gravity-enabled vehicle is held by the
    # same motor/attitude controller used during the actual run.
    for _ in range(8):
        bridge.apply_motor_wrenches()
        world.step(render=False)
    # Sensor prims need warm-up physics, but that warm-up is not part of the
    # experiment. Restore the exact launch state after callbacks are active.
    for drone_id, body in enumerate(bodies):
        body.set_world_pose(
            position=np.asarray(STARTS[drone_id], dtype=float),
            orientation=np.asarray((1.0, 0.0, 0.0, 0.0), dtype=float),
        )
        body.set_linear_velocity(np.zeros(3, dtype=float))
        body.set_angular_velocity(np.zeros(3, dtype=float))
        bridge.previous_positions[drone_id] = np.asarray(
            STARTS[drone_id], dtype=float
        )
        bridge.previous_velocities[drone_id] = np.zeros(3, dtype=float)
        bridge.accelerations[drone_id] = np.zeros(3, dtype=float)
        bridge.contact_active[drone_id] = False
        if ARGS.vehicle_model == "racer_so3":
            bridge.motor_rpms[drone_id] = np.full(
                4, racer_hover_rpm(), dtype=float
            )
    bridge.propeller_visuals.reset()
    bridge.collision_events = 0
    bridge.max_contact_force = 0.0
    bridge.elapsed = 0.0
    bridge.last_odom = -math.inf
    bridge.last_depth = -math.inf
    bridge.cloud_frames = 0
    bridge.control_steps = 0
    bridge.safety_interventions = 0
    bridge.scene_query_updates = 0
    bridge.scene_query_hits = 0
    bridge.scene_query_points = [
        np.empty((0, 3), dtype=float) for _ in bodies
    ]
    bridge.last_scene_query = [-math.inf for _ in bodies]
    bridge.applied_commands = [np.zeros(3) for _ in bodies]
    bridge.positions = [
        np.asarray(point, dtype=float) for point in STARTS
    ]
    bridge.velocities = [np.zeros(3, dtype=float) for _ in bodies]
    bridge.min_inter_drone = math.inf
    bridge.min_obstacle_clearance = math.inf
    bridge.path_lengths = [0.0 for _ in bodies]
    bridge.update_visualization(force=True)
    if ARGS.control_probe:
        bridge.commands[0] = np.asarray(
            ARGS.control_probe_command, dtype=float
        )
        bridge.yaw_targets[0] = 2.0
    offscreen_camera_textures = []
    if ARGS.visualize_exploration and ARGS.vehicle_model == "racer_so3":
        for camera in range_sensors:
            render_product = getattr(camera, "_render_product", None)
            hydra_texture = getattr(
                render_product, "hydra_texture", None
            )
            if hydra_texture is not None:
                offscreen_camera_textures.append(hydra_texture)
        if len(offscreen_camera_textures) != len(range_sensors):
            raise RuntimeError(
                "interactive offscreen-camera gating requires one Hydra "
                "texture per depth camera; found "
                f"{len(offscreen_camera_textures)} for "
                f"{len(range_sensors)} cameras"
            )
        print(
            "RACER_3D_INTERACTIVE_CAMERA_GATING "
            + json.dumps(
                {
                    "camera_count": len(offscreen_camera_textures),
                    "mode": "sensor_frames_only",
                },
                sort_keys=True,
            ),
            flush=True,
        )
    offscreen_cameras_enabled = True
    frame = 0
    last_metrics = -math.inf
    print(
        f"RACER_3D_ISAAC_READY drones={len(bodies)} "
        f"duration={ARGS.duration:.1f} vehicle={ARGS.vehicle_model} "
        f"physics_hz={1.0 / PHYSICS_DT:.0f} motion=rotor_wrench "
        f"propeller_visuals={'on' if bridge.propeller_visuals.enabled else 'off'} "
        f"sensor={'DepthCamera+360SafetyLidar' if ARGS.vehicle_model == 'racer_so3' else 'RotatingLidarPhysX'} "
        f"scene={ARGS.scene_usd or SCENARIO.name}",
        flush=True,
    )
    if ARGS.visualize_exploration:
        print(
            "RACER_3D_VISUALIZATION_READY "
            "map=cyan trail/path=red,green,yellow,purple,orange "
            "camera=overview",
            flush=True,
        )
    try:
        last_render_wall = -math.inf
        render_report_wall = time.monotonic()
        render_report_frames = 0
        while (
            simulation_app.is_running()
            and bridge.elapsed < ARGS.duration
            and not bridge.mission_complete
            and not bridge.mapping_coverage_target_reached
        ):
            step_started = time.monotonic()
            rclpy.spin_once(bridge, timeout_sec=0.0)
            bridge.apply_motor_wrenches()
            phase_checkpoint = (
                ARGS.diagnostics
                and (
                    bridge.control_steps
                    in (1, 10, 50, 250, 500, 1000, 1500)
                    or bridge.control_steps % 500 == 0
                )
            )
            if phase_checkpoint:
                print(
                    f"RACER_3D_PHASE step={bridge.control_steps} "
                    "phase=before_world_step",
                    flush=True,
                )
            render_sensor = (
                bridge.depth_render_due()
                if ARGS.vehicle_model == "racer_so3"
                else frame % max(1, ARGS.render_every) == 0
            )
            # The 1 kHz five-vehicle plant can run much slower than wall
            # time.  If rendering is gated only by the 30 Hz *simulation*
            # sensor clock, Kit may process mouse input just once every few
            # wall seconds.  Keep sensor timing simulation-based, but service
            # the interactive viewport independently in wall time.
            render_interactive = (
                ARGS.visualize_exploration
                and step_started - last_render_wall
                >= 1.0 / ARGS.interactive_render_hz
            )
            render_frame = render_sensor or render_interactive
            # Extra wall-clock renders exist only to service the visible
            # viewport.  Pause all five offscreen RTX camera products on
            # those frames; re-enable them exactly when a simulation-time
            # depth sample is due.  This preserves 30 Hz sensor semantics
            # while avoiding five unnecessary 640x480 renders per UI frame.
            want_offscreen_cameras = render_sensor
            if (
                offscreen_camera_textures
                and want_offscreen_cameras != offscreen_cameras_enabled
            ):
                for texture in offscreen_camera_textures:
                    texture.set_updates_enabled(want_offscreen_cameras)
                offscreen_cameras_enabled = want_offscreen_cameras
            world.step(
                render=render_frame
            )
            if phase_checkpoint:
                print(
                    f"RACER_3D_PHASE step={bridge.control_steps} "
                    "phase=after_world_step",
                    flush=True,
                )
            if render_frame:
                # Schedule from frame start so rendering time itself counts
                # toward the wall-clock interval.  If a frame takes longer
                # than the target interval, the next loop services the UI
                # immediately instead of adding another full delay.
                last_render_wall = step_started
                if ARGS.visualize_exploration:
                    render_report_frames += 1
                    report_now = time.monotonic()
                    report_interval = report_now - render_report_wall
                    if report_interval >= 5.0:
                        print(
                            "RACER_3D_INTERACTIVE_RENDER "
                            + json.dumps(
                                {
                                    "measured_fps": (
                                        render_report_frames
                                        / report_interval
                                    ),
                                    "target_fps": (
                                        ARGS.interactive_render_hz
                                    ),
                                },
                                sort_keys=True,
                            ),
                            flush=True,
                        )
                        render_report_wall = report_now
                        render_report_frames = 0
            bridge.step_observations()
            if phase_checkpoint:
                print(
                    f"RACER_3D_PHASE step={bridge.control_steps} "
                    "phase=after_observations",
                    flush=True,
                )
            bridge.update_visualization()
            if bridge.elapsed - last_metrics >= 0.5:
                bridge.publish_metrics()
                last_metrics = bridge.elapsed
            frame += 1
            remaining = PHYSICS_DT - (time.monotonic() - step_started)
            if remaining > 0.0:
                time.sleep(remaining)
    finally:
        bridge.commands = [np.zeros(3) for _ in bridge.commands]
        bridge.publish_metrics()
        # Allow DDS to deliver the final simulator timestamp to the acceptance
        # monitor before tearing down the ROS context.
        for _ in range(5):
            rclpy.spin_once(bridge, timeout_sec=0.02)
        print(
            "RACER_3D_ISAAC_RESULT "
            + json.dumps(
                {
                    "backend": "isaac_sim_physx_3d",
                    "vehicle_model": ARGS.vehicle_model,
                    "elapsed": bridge.elapsed,
                    "vehicle_asset_usd": (
                        str(ARGS.vehicle_usd)
                        if ARGS.vehicle_model == "racer_so3"
                        else str(CRAZYFLIE_ASSET)
                    ),
                    "collision_events": bridge.collision_events,
                    "physics_contact_events": bridge.collision_events,
                    "max_contact_force": bridge.max_contact_force,
                    "min_inter_drone": (
                        bridge.min_inter_drone
                        if math.isfinite(bridge.min_inter_drone)
                        else None
                    ),
                    "min_obstacle_clearance": (
                        bridge.min_obstacle_clearance
                        if math.isfinite(bridge.min_obstacle_clearance)
                        else None
                    ),
                    "path_lengths": bridge.path_lengths,
                    "start_positions": [list(point) for point in STARTS],
                    "positions": [
                        point.tolist() for point in bridge.positions
                    ],
                    "point_cloud_frames": bridge.cloud_frames,
                    "mapping_coverage_definition": (
                        "known SDF voxels / configured planning-box voxels; "
                        "joint value is the maximum over peer-fused agent maps"
                    ),
                    "mapping_coverage_target": ARGS.mapping_coverage_target,
                    "mapping_coverage_target_reached": (
                        bridge.mapping_coverage_target_reached
                    ),
                    "mapping_coverage_per_agent": bridge.mapping_coverage,
                    "mapping_coverage_counts_per_agent": (
                        bridge.mapping_coverage_counts
                    ),
                    "mapping_coverage_history": (
                        bridge.mapping_coverage_history
                    ),
                    "trajectory_history": bridge.trajectory_history,
                    "mapping_coverage_joint": (
                        max(
                            value for value in bridge.mapping_coverage
                            if value is not None
                        )
                        if any(
                            value is not None
                            for value in bridge.mapping_coverage
                        )
                        else None
                    ),
                    "stop_reason": (
                        "mapping_coverage_target"
                        if bridge.mapping_coverage_target_reached
                        else "original_fsm_completion"
                        if bridge.mission_complete
                        else "duration"
                        if bridge.elapsed >= ARGS.duration
                        else "application_closed"
                    ),
                    "sensor_source": (
                        "upstream_depth_plus_360_safety_lidar"
                        if ARGS.vehicle_model == "racer_so3"
                        else "legacy_rotating_lidar"
                    ),
                    "physics_rate_hz": 1.0 / PHYSICS_DT,
                    "odometry_rate_hz": 1.0 / ODOM_PERIOD,
                    "sensor_rate_hz": 1.0 / DEPTH_PERIOD,
                    "camera_ray_budget": ARGS.camera_ray_budget,
                    "safety_point_refresh_hz": 1.0 / DEPTH_PERIOD,
                    "safety_interventions": bridge.safety_interventions,
                    "scene_query_updates": bridge.scene_query_updates,
                    "scene_query_hits": bridge.scene_query_hits,
                    "min_sweep_free_travel": (
                        bridge.min_sweep_free_travel
                        if math.isfinite(bridge.min_sweep_free_travel)
                        else None
                    ),
                    "low_level_safety": _low_level_safety_description(),
                },
                sort_keys=True,
                allow_nan=False,
            ),
            flush=True,
        )
        bridge.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        simulation_app.close()


if __name__ == "__main__":
    main()
