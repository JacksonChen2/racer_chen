#!/usr/bin/env python3
"""Select two collision-checked UAV starts near each requested warehouse site."""

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
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import UsdGeom  # noqa: E402


SITES = (
    {"name": "site_1_northwest", "anchor": (-20.0, 26.0)},
    {"name": "site_2_southwest", "anchor": (-18.0, 5.0)},
    {
        "name": "site_3_center",
        "anchor": (-11.0, 15.0),
        "aisle_constraint": {
            "orientation": "vertical",
            "x_max": -11.5,
            "maximum_cross_aisle_offset_m": 0.25,
        },
    },
    {"name": "site_4_northeast", "anchor": (0.0, 27.0)},
    {"name": "site_5_southeast", "anchor": (-3.0, 5.0)},
)

START_Z = 0.75
GRID_STEP = 0.25
SEARCH_RADIUS = 2.0
START_CLEARANCE_EXTENT = 0.55
CORRIDOR_CLEARANCE_EXTENT = 0.40
CORRIDOR_HEIGHTS = (1.25, 1.75, 2.25, 2.75)
EXIT_OFFSET = 0.75
MINIMUM_CLEAR_EXITS = 3
MINIMUM_SPACING = 1.50
MAXIMUM_SPACING = 2.50


def frange(begin: float, end: float, step: float):
    count = int(round((end - begin) / step))
    for index in range(count + 1):
        yield begin + index * step


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

    selected_sites = []
    all_starts = []
    for site in SITES:
        anchor_x, anchor_y = site["anchor"]
        bounds = (
            anchor_x - SEARCH_RADIUS,
            anchor_x + SEARCH_RADIUS,
            anchor_y - SEARCH_RADIUS,
            anchor_y + SEARCH_RADIUS,
        )
        candidates = []
        for y in frange(bounds[2], bounds[3], GRID_STEP):
            for x in frange(bounds[0], bounds[1], GRID_STEP):
                aisle_constraint = site.get("aisle_constraint")
                if aisle_constraint and x > aisle_constraint["x_max"]:
                    continue
                if not clear_box(x, y, START_Z, START_CLEARANCE_EXTENT):
                    continue
                if not all(
                    clear_box(x, y, z, CORRIDOR_CLEARANCE_EXTENT)
                    for z in CORRIDOR_HEIGHTS
                ):
                    continue
                clear_exits = sum(
                    clear_box(x + dx, y + dy, 1.75, CORRIDOR_CLEARANCE_EXTENT)
                    for dx, dy in (
                        (EXIT_OFFSET, 0.0),
                        (-EXIT_OFFSET, 0.0),
                        (0.0, EXIT_OFFSET),
                        (0.0, -EXIT_OFFSET),
                    )
                )
                if clear_exits < MINIMUM_CLEAR_EXITS:
                    continue
                candidates.append(
                    {
                        "position": [x, y, START_Z],
                        "distance_to_requested_center_m": math.dist(
                            (x, y), site["anchor"]
                        ),
                        "clear_cardinal_exits": clear_exits,
                        "start_box_clear": True,
                        "vertical_takeoff_corridor_clear": True,
                    }
                )

        pairs = []
        for pair in itertools.combinations(candidates, 2):
            aisle_constraint = site.get("aisle_constraint")
            if (
                aisle_constraint
                and aisle_constraint["orientation"] == "vertical"
                and abs(pair[0]["position"][0] - pair[1]["position"][0])
                > aisle_constraint["maximum_cross_aisle_offset_m"]
            ):
                continue
            spacing = math.dist(pair[0]["position"], pair[1]["position"])
            if not MINIMUM_SPACING <= spacing <= MAXIMUM_SPACING:
                continue
            midpoint = (
                (pair[0]["position"][0] + pair[1]["position"][0]) / 2.0,
                (pair[0]["position"][1] + pair[1]["position"][1]) / 2.0,
            )
            distances = [item["distance_to_requested_center_m"] for item in pair]
            score = (
                math.dist(midpoint, site["anchor"]),
                max(distances),
                sum(distances),
                abs(spacing - MINIMUM_SPACING),
                -min(item["clear_cardinal_exits"] for item in pair),
                pair[0]["position"],
                pair[1]["position"],
            )
            pairs.append((score, pair, spacing, midpoint))

        if not pairs:
            raise RuntimeError(
                f"no valid two-UAV takeoff layout found for {site['name']} "
                f"from {len(candidates)} candidates"
            )
        pairs.sort(key=lambda item: item[0])
        score, pair, spacing, midpoint = pairs[0]
        starts = [item["position"] for item in pair]
        all_starts.extend(starts)
        selected_sites.append(
            {
                "name": site["name"],
                "requested_center_xy_m": list(site["anchor"]),
                "selected_midpoint_xy_m": list(midpoint),
                "search_bounds_xy_m": {
                    "x": [bounds[0], bounds[1]],
                    "y": [bounds[2], bounds[3]],
                },
                "aisle_constraint": site.get("aisle_constraint"),
                "candidate_count": len(candidates),
                "pair_spacing_m": spacing,
                "midpoint_offset_m": score[0],
                "uavs": list(pair),
            }
        )

    minimum_global_spacing = min(
        math.dist(left, right)
        for left, right in itertools.combinations(all_starts, 2)
    )
    result = {
        "scene": str(scene),
        "layout": "five_requested_sites_two_collision_checked_uavs_each",
        "selection_rule": (
            "0.55 m clear start box; 0.40 m clear vertical takeoff corridor "
            "through z=2.75 m; at least three clear cardinal exits; "
            "1.50-2.50 m within-site spacing; pair midpoint nearest requested center"
        ),
        "drone_count": 10,
        "start_height_m": START_Z,
        "minimum_global_spacing_m": minimum_global_spacing,
        "sites": selected_sites,
        "start_positions": all_starts,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


try:
    main()
finally:
    simulation_app.close()
