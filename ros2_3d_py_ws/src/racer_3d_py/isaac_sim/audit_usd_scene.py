#!/usr/bin/env python3
"""Inspect an external USD and validate candidate Crazyflie start poses."""

import argparse
import json
from pathlib import Path


def parse_arguments():
    parser = argparse.ArgumentParser()
    parser.add_argument("scene_usd", type=Path)
    parser.add_argument("--starts", type=float, nargs="+", default=[])
    parser.add_argument("--clearance", type=float, default=0.18)
    return parser.parse_args()


ARGS = parse_arguments()
SCENE_USD = ARGS.scene_usd.expanduser().resolve()
if not SCENE_USD.is_file():
    raise SystemExit(f"scene USD does not exist: {SCENE_USD}")
if ARGS.starts and len(ARGS.starts) % 3:
    raise SystemExit("--starts requires complete x y z triples")

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import UsdGeom, UsdPhysics, UsdUtils  # noqa: E402


def main():
    world = World(
        physics_dt=0.02,
        rendering_dt=0.02,
        stage_units_in_meters=1.0,
    )
    stage = omni.usd.get_context().get_stage()
    external = UsdGeom.Xform.Define(stage, "/World/ExternalScene")
    external.GetPrim().GetReferences().AddReference(str(SCENE_USD))
    for _ in range(20):
        simulation_app.update()
    world.reset()
    for _ in range(5):
        world.step(render=False)

    prims = list(stage.Traverse())
    collisions = [
        prim for prim in prims if prim.HasAPI(UsdPhysics.CollisionAPI)
    ]
    rigid_bodies = [
        prim for prim in prims if prim.HasAPI(UsdPhysics.RigidBodyAPI)
    ]
    cache = UsdGeom.BBoxCache(
        0.0,
        [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
        useExtentsHint=True,
    )
    bounds = cache.ComputeWorldBound(external.GetPrim()).ComputeAlignedRange()
    _, assets, unresolved = UsdUtils.ComputeAllDependencies(str(SCENE_USD))

    probes = []
    query = get_physx_scene_query_interface()
    for index in range(0, len(ARGS.starts), 3):
        position = tuple(ARGS.starts[index:index + 3])
        hits = []

        def report(hit):
            hits.append(str(hit.rigid_body))
            return True

        query.overlap_box(
            carb.Float3(
                ARGS.clearance,
                ARGS.clearance,
                ARGS.clearance,
            ),
            carb.Float3(*position),
            carb.Float4(0.0, 0.0, 0.0, 1.0),
            report,
            False,
        )
        probes.append(
            {
                "position": position,
                "free": not hits,
                "overlaps": sorted(set(hits)),
            }
        )

    result = {
        "scene_usd": str(SCENE_USD),
        "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
        "meters_per_unit": UsdGeom.GetStageMetersPerUnit(stage),
        "bounds_min": list(bounds.GetMin()),
        "bounds_max": list(bounds.GetMax()),
        "prim_count": len(prims),
        "collision_prim_count": len(collisions),
        "rigid_body_count": len(rigid_bodies),
        "asset_dependency_count": len(assets),
        "unresolved_dependencies": [str(item) for item in unresolved],
        "start_probes": probes,
    }
    print("RACER_USD_AUDIT " + json.dumps(result, sort_keys=True), flush=True)


try:
    main()
finally:
    simulation_app.close()
