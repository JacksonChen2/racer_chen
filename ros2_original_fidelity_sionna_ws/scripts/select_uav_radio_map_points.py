#!/usr/bin/env python3
"""Select collision-free UAV radio-map transmitter points in an Isaac USD."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--heights", type=float, nargs="+", required=True)
    parser.add_argument("--x-min", type=float, required=True)
    parser.add_argument("--x-max", type=float, required=True)
    parser.add_argument("--y-min", type=float, required=True)
    parser.add_argument("--y-max", type=float, required=True)
    parser.add_argument("--rows", type=int, default=5)
    parser.add_argument("--cols", type=int, default=5)
    parser.add_argument("--clearance", type=float, default=0.25)
    parser.add_argument("--search-step", type=float, default=0.5)
    parser.add_argument("--search-radius", type=float, default=4.0)
    return parser.parse_args()


ARGS = parse_args()

from isaacsim import SimulationApp  # noqa: E402


simulation_app = SimulationApp({"headless": True})

import carb  # noqa: E402
import numpy as np  # noqa: E402
import omni.usd  # noqa: E402
from isaacsim.core.api import World  # noqa: E402
from omni.physx import get_physx_scene_query_interface  # noqa: E402
from pxr import UsdGeom  # noqa: E402


def main() -> None:
    scene = ARGS.scene.expanduser().resolve()
    if not scene.is_file():
        raise FileNotFoundError(scene)
    if ARGS.rows < 1 or ARGS.cols < 1:
        raise ValueError("rows and cols must be positive")

    world = World(physics_dt=0.02, rendering_dt=0.02, stage_units_in_meters=1.0)
    stage = omni.usd.get_context().get_stage()
    external = UsdGeom.Xform.Define(stage, "/World/ExternalScene")
    external.GetPrim().GetReferences().AddReference(str(scene))
    for _ in range(30):
        simulation_app.update()
    world.reset()
    for _ in range(5):
        world.step(render=False)

    margin = max(ARGS.clearance + 0.15, 0.4)
    anchor_x = np.linspace(ARGS.x_min + margin, ARGS.x_max - margin, ARGS.cols)
    anchor_y = np.linspace(ARGS.y_min + margin, ARGS.y_max - margin, ARGS.rows)
    values = np.arange(-ARGS.search_radius, ARGS.search_radius + 1.0e-9, ARGS.search_step)
    offsets = sorted(
        ((float(dx), float(dy)) for dx in values for dy in values),
        key=lambda item: (item[0] ** 2 + item[1] ** 2, abs(item[1]), abs(item[0])),
    )
    query = get_physx_scene_query_interface()

    def collision_free(point: tuple[float, float, float]) -> bool:
        hit_found = False

        def report(_hit):
            nonlocal hit_found
            hit_found = True
            return False

        query.overlap_box(
            carb.Float3(ARGS.clearance, ARGS.clearance, ARGS.clearance),
            carb.Float3(*point),
            carb.Float4(0.0, 0.0, 0.0, 1.0),
            report,
            False,
        )
        return not hit_found

    by_height = []
    for height in sorted(set(ARGS.heights)):
        selected: list[dict] = []
        used_xy: list[tuple[float, float]] = []
        for row, y_anchor in enumerate(anchor_y):
            for col, x_anchor in enumerate(anchor_x):
                chosen = None
                attempts = 0
                for dx, dy in offsets:
                    x = float(x_anchor + dx)
                    y = float(y_anchor + dy)
                    if not (
                        ARGS.x_min + margin <= x <= ARGS.x_max - margin
                        and ARGS.y_min + margin <= y <= ARGS.y_max - margin
                    ):
                        continue
                    if any(np.hypot(x - px, y - py) < 0.75 for px, py in used_xy):
                        continue
                    attempts += 1
                    point = (x, y, float(height))
                    if collision_free(point):
                        chosen = point
                        break
                if chosen is None:
                    raise RuntimeError(
                        f"no free point near anchor ({x_anchor}, {y_anchor}, {height})"
                    )
                used_xy.append(chosen[:2])
                selected.append(
                    {
                        "id": row * ARGS.cols + col + 1,
                        "row": row,
                        "col": col,
                        "anchor": [float(x_anchor), float(y_anchor), float(height)],
                        "position": list(chosen),
                        "anchor_offset_m": float(
                            np.hypot(chosen[0] - x_anchor, chosen[1] - y_anchor)
                        ),
                        "attempts": attempts,
                    }
                )
        by_height.append({"height_m": float(height), "points": selected})
        print(f"height z={height:g}: selected {len(selected)} free points", flush=True)

    output = {
        "scene": str(scene),
        "bounds_m": [ARGS.x_min, ARGS.x_max, ARGS.y_min, ARGS.y_max],
        "grid_shape": [ARGS.rows, ARGS.cols],
        "clearance_half_extent_m": ARGS.clearance,
        "search_step_m": ARGS.search_step,
        "search_radius_m": ARGS.search_radius,
        "heights": by_height,
    }
    ARGS.output.parent.mkdir(parents=True, exist_ok=True)
    ARGS.output.write_text(json.dumps(output, indent=2) + "\n", encoding="utf-8")
    print(f"saved point selection to {ARGS.output}", flush=True)


try:
    main()
finally:
    simulation_app.close()

