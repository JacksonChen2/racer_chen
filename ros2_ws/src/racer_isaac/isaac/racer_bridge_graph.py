"""Create RACER ROS 2 Bridge graphs inside Isaac Sim 5.1.

Run with Isaac Sim's Script Editor after changing ``DRONES`` to the stage's
prim paths.  This file intentionally uses Isaac's bundled Python and does not
import the external ROS 2 Humble environment.
"""

from __future__ import annotations

from dataclasses import dataclass

import omni.graph.core as og


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
                ("ReadLidar.outputs:bufferSize", "PublishCloud.inputs:bufferSize"),
                ("ReadTime.outputs:simulationTime", "PublishCloud.inputs:timeStamp"),
            ],
            keys.SET_VALUES: [
                ("Context.inputs:domain_id", DOMAIN_ID),
                ("ComputeOdom.inputs:chassisPrim", [drone.base_prim]),
                ("ReadLidar.inputs:lidarPrim", drone.lidar_prim),
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


for configuration in DRONES:
    create_bridge_graph(configuration)

print(f"Created {len(DRONES)} RACER ROS 2 Bridge graph(s).")

