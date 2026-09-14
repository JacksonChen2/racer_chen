#!/usr/bin/env python3
"""Add one collision-checked UAV near each site in a validated 10-UAV layout."""

from __future__ import annotations

import argparse
import itertools
import json
import math
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--base-layout", type=Path, required=True)
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


START_Z = 0.75
GRID_STEP = 0.25
SEARCH_RADIUS = 3.0
START_CLEARANCE_EXTENT = 0.55
CORRIDOR_CLEARANCE_EXTENT = 0.40
CORRIDOR_HEIGHTS = (1.25, 1.75, 2.25, 2.75)
EXIT_OFFSET = 0.75
MINIMUM_CLEAR_EXITS = 3
MINIMUM_SPACING = 1.50
MAXIMUM_SITE_SPACING = 3.25


def frange(begin: float, end: float, step: float):
    count = int(round((end - begin) / step))
    for index in range(count + 1):
        yield begin + index * step


def main() -> None:
    scene = ARGS.scene.expanduser().resolve()
    base_path = ARGS.base_layout.expanduser().resolve()
    output = ARGS.output.expanduser().resolve()
    base = json.loads(base_path.read_text(encoding="utf-8"))
    if int(base.get("drone_count", 0)) != 10 or len(base["start_positions"]) != 10:
        raise ValueError("base layout must contain exactly 10 UAV starts")
    if len(base.get("regions", [])) != 5:
        raise ValueError("base layout must contain exactly five regions")

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

    original_starts = [[float(value) for value in point] for point in base["start_positions"]]
    additions: list[list[float]] = []
    extended_regions = []
    for region in base["regions"]:
        existing = [
            [float(value) for value in item["position"]]
            for item in region["uavs"]
        ]
        if len(existing) != 2:
            raise ValueError(f"{region['name']} does not contain two base UAVs")
        anchor_x, anchor_y = map(float, region["requested_center_xy_m"])
        candidates = []
        for y in frange(anchor_y - SEARCH_RADIUS, anchor_y + SEARCH_RADIUS, GRID_STEP):
            for x in frange(anchor_x - SEARCH_RADIUS, anchor_x + SEARCH_RADIUS, GRID_STEP):
                aisle = region.get("aisle_constraint")
                if aisle:
                    if x > float(aisle["x_max"]):
                        continue
                    if any(
                        abs(x - point[0])
                        > float(aisle["maximum_cross_aisle_offset_m"])
                        for point in existing
                    ):
                        continue
                point = [x, y, START_Z]
                if min(math.dist(point, other) for other in original_starts + additions) < MINIMUM_SPACING:
                    continue
                site_distances = [math.dist(point, other) for other in existing]
                if max(site_distances) > MAXIMUM_SITE_SPACING:
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
                        "position": point,
                        "distance_to_requested_center_m": math.dist(
                            (x, y), (anchor_x, anchor_y)
                        ),
                        "distance_to_existing_uavs_m": site_distances,
                        "clear_cardinal_exits": clear_exits,
                        "start_box_clear": True,
                        "vertical_takeoff_corridor_clear": True,
                    }
                )
        if not candidates:
            raise RuntimeError(f"no valid added start found near {region['name']}")
        candidates.sort(
            key=lambda item: (
                item["distance_to_requested_center_m"],
                max(item["distance_to_existing_uavs_m"]),
                -item["clear_cardinal_exits"],
                item["position"],
            )
        )
        selected = candidates[0]
        additions.append(selected["position"])
        extended = dict(region)
        extended["base_uavs"] = region["uavs"]
        extended["added_uav"] = selected
        extended["candidate_count"] = len(candidates)
        extended["uavs"] = [*region["uavs"], selected]
        extended_regions.append(extended)

    starts = original_starts + additions
    minimum_spacing = min(
        math.dist(left, right) for left, right in itertools.combinations(starts, 2)
    )
    result = {
        "scene": str(scene),
        "layout": "five_requested_sites_existing_10_plus_one_checked_uav_per_site",
        "selection_rule": (
            "preserve all 10 validated starts; add one UAV per site with a 0.55 m "
            "clear start box, 0.40 m clear vertical corridor through z=2.75 m, "
            "at least three clear cardinal exits, and at least 1.50 m global spacing"
        ),
        "base_layout": str(base_path),
        "drone_count": 15,
        "start_height_m": START_Z,
        "minimum_global_spacing_m": minimum_spacing,
        "validation_ok": minimum_spacing >= MINIMUM_SPACING,
        "regions": extended_regions,
        "original_start_positions": original_starts,
        "added_start_positions": additions,
        "start_positions": starts,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2), flush=True)


try:
    main()
finally:
    simulation_app.close()
