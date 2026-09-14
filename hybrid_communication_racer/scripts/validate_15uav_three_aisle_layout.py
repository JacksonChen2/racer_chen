#!/usr/bin/env python3
"""Validate the fixed 15-UAV, three-adjacent-aisle takeoff layout in Isaac Sim."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--positions", type=float, nargs="+",
        help="Optional flat x y z list; defaults to the fixed 15-UAV layout.",
    )
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import UsdGeom  # noqa: E402


# The centre of the scene (-10.03, 14.89) lies inside a rack.  The centre
# group therefore uses the nearest open aisle, centred near x=-7.2 m.  The
# groups centred near x=-13.2 m and x=-3.1 m occupy the immediately adjacent
# aisles; rack rows near x=-10.5 m and x=-5.6 m separate the three groups.
REGIONS = (
    {
        "name": "left_adjacent_aisle",
        "aisle_center_xy_m": (-13.20, 15.43),
        "separated_from_center_by_rack_x_m": -10.5,
        "uavs": (
            (-14.49, 14.91, 0.80),
            (-13.20, 14.91, 1.50),
            (-11.91, 14.91, 2.20),
            (-11.91, 16.20, 1.15),
            (-13.20, 16.20, 1.85),
        ),
    },
    {
        "name": "center_nearest_open_aisle",
        "aisle_center_xy_m": (-7.32, 15.96),
        "uavs": (
            (-7.80, 15.00, 0.80),
            (-6.60, 15.00, 1.50),
            (-7.80, 16.20, 2.20),
            (-6.60, 16.20, 1.15),
            (-7.80, 17.40, 1.85),
        ),
    },
    {
        "name": "right_adjacent_aisle",
        "aisle_center_xy_m": (-2.97, 15.04),
        "separated_from_center_by_rack_x_m": -5.6,
        "uavs": (
            (-3.75, 16.60, 0.80),
            (-2.45, 15.30, 1.50),
            (-2.45, 16.60, 2.20),
            (-3.75, 14.00, 1.15),
            (-3.75, 15.30, 1.85),
        ),
    },
)


def main() -> None:
    scene = ARGS.scene.expanduser().resolve()
    output = ARGS.output.expanduser().resolve()
    world = World(physics_dt=0.02, rendering_dt=0.02, stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    external = UsdGeom.Xform.Define(stage, "/World/ExternalScene")
    external.GetPrim().GetReferences().AddReference(str(scene))
    for _ in range(30):
        simulation_app.update()
    world.reset()
    for _ in range(5):
        world.step(render=False)

    query = get_physx_scene_query_interface()

    def clear_box(x: float, y: float, z: float, extent: float) -> bool:
        hit_found = False

        def report(_hit):
            nonlocal hit_found
            hit_found = True
            return False

        query.overlap_box(
            carb.Float3(extent, extent, extent),
            carb.Float3(x, y, z),
            carb.Float4(0.0, 0.0, 0.0, 1.0),
            report,
            False,
        )
        return not hit_found

    if ARGS.positions is not None:
        if len(ARGS.positions) % 3 != 0:
            raise SystemExit("--positions must contain a multiple of three values")
        starts = [ARGS.positions[index:index + 3] for index in range(0, len(ARGS.positions), 3)]
    else:
        starts = [list(position) for region in REGIONS for position in region["uavs"]]
    checks = []
    errors = []
    for drone_id, (x, y, z) in enumerate(starts, start=1):
        start_clear = clear_box(x, y, z, 0.45)
        vertical_samples = [
            sample_z for sample_z in (1.25, 1.75, 2.25, 2.75) if sample_z > z
        ]
        vertical_clear = all(clear_box(x, y, sample_z, 0.40) for sample_z in vertical_samples)
        checks.append(
            {
                "drone_id": drone_id,
                "position": [x, y, z],
                "start_box_clear": start_clear,
                "vertical_takeoff_corridor_clear": vertical_clear,
            }
        )
        if not start_clear or not vertical_clear:
            errors.append(f"UAV {drone_id} failed collision clearance")

    pairwise = [
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    ]
    minimum_spacing = min(pairwise)
    if minimum_spacing < 1.20:
        errors.append(f"minimum 3-D spacing {minimum_spacing:.3f} m is below 1.20 m")

    result = {
        "scene": str(scene),
        "layout": (
            "custom_validated_start_positions"
            if ARGS.positions is not None
            else "three_adjacent_aisles_five_uavs_each"
        ),
        "selection_rule": (
            "three adjacent open aisles around the scene centre, with one rack row "
            "between each side aisle and the centre aisle; five UAVs per aisle; "
            "0.45 m clear start box; 0.40 m clear vertical takeoff corridor through "
            "z=2.75 m; minimum 1.20 m 3-D start spacing"
        ),
        "drone_count": len(starts),
        "minimum_global_spacing_m": minimum_spacing,
        "regions": [] if ARGS.positions is not None else [
            {
                key: (list(value) if isinstance(value, tuple) else value)
                for key, value in region.items()
                if key != "uavs"
            }
            | {"uavs": [list(position) for position in region["uavs"]]}
            for region in REGIONS
        ],
        "collision_checks": checks,
        "start_positions": starts,
        "validation_ok": not errors,
        "validation_errors": errors,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)
    if errors:
        raise SystemExit("; ".join(errors))


try:
    main()
finally:
    simulation_app.close()
