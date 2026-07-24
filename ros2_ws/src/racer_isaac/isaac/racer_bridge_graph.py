"""Create RACER ROS 2 Bridge graphs inside Isaac Sim 5.1.

Run with Isaac Sim's Script Editor after changing ``DRONES`` to the stage's
prim paths.  This file intentionally uses Isaac's bundled Python and does not
import the external ROS 2 Humble environment.  On Linux, start Isaac with its
bundled Humble library directory in ``LD_LIBRARY_PATH`` as documented in the
repository README.
"""

from __future__ import annotations

from dataclasses import dataclass
import math

import omni.kit.app
import omni.graph.core as og
import omni.usd
from pxr import Gf, UsdPhysics


@dataclass(frozen=True)
class DroneBridge:
    drone_id: int
    base_prim: str
    lidar_prim: str


# Stage-specific prim paths are the only values users must edit.
DRONES = [
    DroneBridge(1, "/World/drone_1", "/World/drone_1/Lidar"),
]
DOMAIN_ID = 0
_UPDATE_SUBSCRIPTIONS = []


def _set_if_present(controller, node_path: str, port: str, value) -> None:
    attribute = controller.attribute(f"{node_path}.inputs:{port}")
    if attribute.is_valid():
        attribute.set(value)


def create_bridge_graph(drone: DroneBridge) -> None:
    graph_path = f"/World/RACER_ROS2_Bridge_{drone.drone_id}"
    namespace = f"drone_{drone.drone_id}"
    controller = og.Controller
    keys = controller.Keys
    controller.edit(
        {"graph_path": graph_path, "evaluator_name": "execution"},
        {
            keys.CREATE_NODES: [
                ("Tick", "omni.graph.action.OnPlaybackTick"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("ReadTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("ReadLidar", "isaacsim.sensors.physx.IsaacReadLidarPointCloud"),
                ("PublishCloud", "isaacsim.ros2.bridge.ROS2PublishPointCloud"),
                ("SubscribeVelocity", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
            ],
            keys.CONNECT: [
                ("Tick.outputs:tick", "PublishClock.inputs:execIn"),
                ("Tick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("Tick.outputs:tick", "ReadLidar.inputs:execIn"),
                ("Tick.outputs:tick", "SubscribeVelocity.inputs:execIn"),
                ("ComputeOdom.outputs:execOut", "PublishOdom.inputs:execIn"),
                ("ReadLidar.outputs:execOut", "PublishCloud.inputs:execIn"),
                ("ReadTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadTime.outputs:simulationTime", "PublishCloud.inputs:timeStamp"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishCloud.inputs:context"),
                ("Context.outputs:context", "SubscribeVelocity.inputs:context"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ReadLidar.outputs:data", "PublishCloud.inputs:data"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", DOMAIN_ID),
                ("ComputeOdom.inputs:chassisPrim", [drone.base_prim]),
                ("ReadLidar.inputs:lidarPrim", [drone.lidar_prim]),
                ("PublishClock.inputs:topicName", "/clock"),
                ("PublishOdom.inputs:topicName", f"/{namespace}/odometry"),
                ("PublishOdom.inputs:odomFrameId", "world"),
                ("PublishOdom.inputs:chassisFrameId", f"{namespace}/base_link"),
                ("PublishCloud.inputs:topicName", f"/{namespace}/pointcloud"),
                ("PublishCloud.inputs:frameId", f"{namespace}/lidar"),
                ("SubscribeVelocity.inputs:topicName", f"/{namespace}/isaac/velocity_command"),
            ],
        },
    )
    # Node port names occasionally acquire additions between Isaac releases.
    # Optional settings are applied only when exposed by the installed node.
    _set_if_present(controller, f"{graph_path}/PublishOdom", "nodeNamespace", namespace)
    _set_if_present(controller, f"{graph_path}/PublishCloud", "nodeNamespace", namespace)
    _set_if_present(controller, f"{graph_path}/PublishOdom", "publishRawVelocities", True)

    # The default integration contract controls a rigid-body UAV in world
    # velocity.  Asset-specific multirotor dynamics can replace this callback
    # while keeping the ROS 2 Twist topic unchanged.
    linear_attribute = controller.attribute(
        f"{graph_path}/SubscribeVelocity.outputs:linearVelocity"
    )
    angular_attribute = controller.attribute(
        f"{graph_path}/SubscribeVelocity.outputs:angularVelocity"
    )

    def apply_velocity_command(_event) -> None:
        stage = omni.usd.get_context().get_stage()
        prim = stage.GetPrimAtPath(drone.base_prim)
        if not prim.IsValid():
            return
        rigid_body = UsdPhysics.RigidBodyAPI.Get(stage, drone.base_prim)
        if not rigid_body:
            rigid_body = UsdPhysics.RigidBodyAPI.Apply(prim)
        linear = linear_attribute.get() or (0.0, 0.0, 0.0)
        angular = angular_attribute.get() or (0.0, 0.0, 0.0)
        rigid_body.GetVelocityAttr().Set(Gf.Vec3f(*(float(value) for value in linear)))
        # USD stores angular velocity in degrees per second.
        rigid_body.GetAngularVelocityAttr().Set(
            Gf.Vec3f(*(math.degrees(float(value)) for value in angular))
        )

    subscription = (
        omni.kit.app.get_app()
        .get_update_event_stream()
        .create_subscription_to_pop(
            apply_velocity_command,
            name=f"RACER velocity controller {drone.drone_id}",
        )
    )
    _UPDATE_SUBSCRIPTIONS.append(subscription)


for configuration in DRONES:
    create_bridge_graph(configuration)

print(f"Created {len(DRONES)} RACER ROS 2 Bridge graph(s).")
