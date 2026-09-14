#!/usr/bin/env python3
"""Select three collision-checked takeoff points in each of five warehouse regions."""

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


REGIONS = (
    {
        "name": "southwest",
        "anchor": (-26.0, 1.3),
        "bounds": (-21.0, -18.0, 3.5, 7.5),
    },
    {
        # The original southeast corner starts produced an empty known-free
        # component. Search the previously validated corridor north of it.
        "name": "southeast",
        "anchor": (5.0, 1.3),
        "bounds": (-3.0, 0.0, 7.0, 9.0),
    },
    {
        "name": "northwest",
        "anchor": (-26.0, 29.8),
        "bounds": (-25.5, -21.0, 24.0, 28.5),
    },
    {
        "name": "northeast",
        "anchor": (5.0, 29.8),
        "bounds": (0.0, 4.0, 24.5, 29.0),
    },
    {
        "name": "center",
        "anchor": (-10.02891489217081, 14.888611215255622),
        "bounds": (-12.0, -7.5, 12.5, 17.5),
    },
)

START_Z = 0.75
GRID_STEP = 0.25
MINIMUM_SPACING = 1.50
MAXIMUM_SPACING = 3.25


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

    selected_regions = []
    all_starts = []
    for region in REGIONS:
        xmin, xmax, ymin, ymax = region["bounds"]
        anchor = region["anchor"]
        candidates = []
        for y in frange(ymin, ymax, GRID_STEP):
            for x in frange(xmin, xmax, GRID_STEP):
                if not clear_box(x, y, START_Z, 0.55):
                    continue
                if not all(
                    clear_box(x, y, z, 0.40)
                    for z in (1.25, 1.75, 2.25, 2.75)
                ):
                    continue
                exits = sum(
                    clear_box(x + dx, y + dy, 1.75, 0.40)
                    for dx, dy in (
                        (0.75, 0.0), (-0.75, 0.0),
                        (0.0, 0.75), (0.0, -0.75),
                    )
                )
                if exits < 3:
                    continue
                candidates.append(
                    {
                        "position": [x, y, START_Z],
                        "distance_to_anchor_m": math.dist((x, y), anchor),
                        "clear_cardinal_exits": exits,
                    }
                )

        # The nearest collision-checked candidates are sufficient and keep the
        # combinatorial triple search bounded and deterministic.
        candidates.sort(
            key=lambda candidate: (
                candidate["distance_to_anchor_m"],
                -candidate["clear_cardinal_exits"],
                candidate["position"],
            )
        )
        candidates = candidates[:120]
        triples = []
        for triple in itertools.combinations(candidates, 3):
            distances = [
                math.dist(left["position"], right["position"])
                for left, right in itertools.combinations(triple, 2)
            ]
            if min(distances) < MINIMUM_SPACING or max(distances) > MAXIMUM_SPACING:
                continue
            anchor_distances = [item["distance_to_anchor_m"] for item in triple]
            score = (
                max(anchor_distances),
                sum(anchor_distances),
                max(distances) - min(distances),
                -min(item["clear_cardinal_exits"] for item in triple),
            )
            triples.append((score, triple, distances))
        if not triples:
            raise RuntimeError(
                f"no valid three-UAV takeoff layout found for {region['name']} "
                f"from {len(candidates)} candidates"
            )
        triples.sort(key=lambda item: item[0])
        score, triple, distances = triples[0]
        starts = [item["position"] for item in triple]
        all_starts.extend(starts)
        selected_regions.append(
            {
                "name": region["name"],
                "anchor_xy_m": list(anchor),
                "search_bounds_xy_m": {
                    "x": [xmin, xmax],
                    "y": [ymin, ymax],
                },
                "candidate_count": len(candidates),
                "score": list(score),
                "pairwise_spacing_m": distances,
                "uavs": list(triple),
            }
        )

    minimum_global_spacing = min(
        math.dist(left, right)
        for left, right in itertools.combinations(all_starts, 2)
    )
    result = {
        "scene": str(scene),
        "layout": "five_regions_three_collision_checked_uavs_each",
        "selection_rule": (
            "0.55 m clear start box; 0.40 m clear vertical takeoff corridor "
            "through z=2.75 m; at least three clear cardinal exits; "
            "1.50-3.25 m within-site spacing"
        ),
        "drone_count": 15,
        "minimum_global_spacing_m": minimum_global_spacing,
        "regions": selected_regions,
        "start_positions": all_starts,
    }
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n")
    print(json.dumps(result, indent=2), flush=True)


try:
    main()
finally:
    simulation_app.close()
