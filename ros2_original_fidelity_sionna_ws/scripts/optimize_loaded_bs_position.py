#!/usr/bin/env python3
"""Select a ceiling BS position using 28 GHz Sionna radio maps."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


FULL_BOUNDS = (-27.0, 6.0, -23.0, 30.0)
ACTIVE_BOUNDS = (-26.8, 5.8, 7.2, 26.2)
CURRENT = (-10.5, 16.7, 8.1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--cell-size", type=float, default=1.0)
    return parser.parse_args()


def db(values: np.ndarray) -> np.ndarray:
    return 10.0 * np.log10(np.maximum(values, 1.0e-18))


def summarize(gain_db: np.ndarray) -> dict[str, float]:
    flat = gain_db.reshape(-1)
    return {
        "mean_db": float(np.mean(flat)),
        "p10_db": float(np.percentile(flat, 10)),
        "p25_db": float(np.percentile(flat, 25)),
        "p50_db": float(np.percentile(flat, 50)),
        "fraction_above_minus_140_db": float(np.mean(flat > -140.0)),
        "fraction_above_minus_120_db": float(np.mean(flat > -120.0)),
    }


def main() -> None:
    args = parse_args()
    from sionna.rt import PlanarArray, RadioMapSolver, Transmitter, load_scene

    scene = load_scene(str(args.scene.resolve()), merge_shapes=True)
    scene.frequency = 28.0e9
    scene.bandwidth = 100.0e6
    scene.tx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    transmitter = Transmitter(name="bs_candidate", position=list(CURRENT), power_dbm=33.0)
    scene.add(transmitter)
    solver = RadioMapSolver()

    xmin, xmax, ymin, ymax = FULL_BOUNDS
    center = [0.5 * (xmin + xmax), 0.5 * (ymin + ymax)]
    size = [xmax - xmin, ymax - ymin]
    candidates = {CURRENT}
    for z in (7.2, 7.7, 8.1):
        for y in (-18.0, -10.0, -2.0, 6.0, 14.0, 22.0, 28.0):
            for x in (-24.0, -17.0, -10.5, -4.0, 3.0):
                candidates.add((x, y, z))
    # Refine the strongest central-roof region from the coarse pass.  The
    # upper limit keeps the enclosure mounting plane below the 9 m ceiling.
    for z in np.arange(7.2, 8.41, 0.2):
        for y in np.arange(10.0, 18.01, 1.0):
            for x in np.arange(-13.5, -7.49, 1.0):
                candidates.add((round(float(x), 2), round(float(y), 2), round(float(z), 2)))
    for z in np.arange(7.8, 8.41, 0.2):
        for y in np.arange(14.0, 18.01, 0.5):
            for x in np.arange(-16.5, -12.49, 0.5):
                candidates.add((round(float(x), 2), round(float(y), 2), round(float(z), 2)))
    for y in np.arange(14.0, 18.01, 0.5):
        for x in np.arange(-16.5, -12.49, 0.5):
            candidates.add((round(float(x), 2), round(float(y), 2), 8.55))

    rows = []
    for index, position in enumerate(sorted(candidates, key=lambda p: (p[2], p[1], p[0]))):
        transmitter.position = list(position)
        height_maps = []
        centers = None
        for receiver_height in (1.0, 2.5, 4.5, 6.5):
            radio_map = solver(
                scene=scene,
                center=[center[0], center[1], receiver_height],
                orientation=[0.0, 0.0, 0.0],
                size=size,
                cell_size=[args.cell_size, args.cell_size],
                samples_per_tx=args.samples,
                max_depth=2,
                los=True,
                specular_reflection=True,
                diffuse_reflection=False,
                refraction=True,
                diffraction=False,
                seed=42,
            )
            height_maps.append(db(np.asarray(radio_map.path_gain)[0]))
            if centers is None:
                centers = np.asarray(radio_map.cell_centers)
        gains = np.stack(height_maps)
        x_grid = centers[:, :, 0]
        y_grid = centers[:, :, 1]
        axmin, axmax, aymin, aymax = ACTIVE_BOUNDS
        active_mask = (
            (x_grid >= axmin) & (x_grid <= axmax) & (y_grid >= aymin) & (y_grid <= aymax)
        )
        full = summarize(gains)
        active = summarize(gains[:, active_mask])
        # Favor uniform whole-factory service while retaining good service in
        # the rack-zone actually explored by Warehouse Loaded.
        score = (
            0.45 * full["fraction_above_minus_140_db"]
            + 0.20 * full["fraction_above_minus_120_db"]
            + 0.25 * active["fraction_above_minus_140_db"]
            + 0.10 * active["fraction_above_minus_120_db"]
        )
        row = {
            "x": position[0],
            "y": position[1],
            "phase_center_z": position[2],
            "model_z": position[2] + 0.45,
            "score": score,
            **{f"full_{key}": value for key, value in full.items()},
            **{f"active_{key}": value for key, value in active.items()},
        }
        rows.append(row)
        print(f"{index + 1}/{len(candidates)} position={position} score={score:.6f}", flush=True)

    rows.sort(key=lambda row: row["score"], reverse=True)
    args.output.mkdir(parents=True, exist_ok=True)
    with (args.output / "candidate_scores.csv").open("w", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)
    report = {
        "method": {
            "carrier_frequency_hz": 28.0e9,
            "bandwidth_hz": 100.0e6,
            "samples_per_map": args.samples,
            "max_depth": 2,
            "receiver_heights_m": [1.0, 2.5, 4.5, 6.5],
            "full_factory_bounds_m": FULL_BOUNDS,
            "active_rack_zone_bounds_m": ACTIVE_BOUNDS,
            "thresholds_db": [-140.0, -120.0],
        },
        "current": next(row for row in rows if (row["x"], row["y"], row["phase_center_z"]) == CURRENT),
        "recommended": rows[0],
        "top_10": rows[:10],
    }
    (args.output / "placement_report.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
