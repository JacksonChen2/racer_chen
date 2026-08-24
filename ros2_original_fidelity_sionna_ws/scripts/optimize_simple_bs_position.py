#!/usr/bin/env python3
"""Optimize the Warehouse Simple BS phase centre for multi-height coverage."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np


MAP_BOUNDS = (-10.0, 9.0, -11.9, 17.6)
UAV_HEIGHTS = (0.75, 1.5, 3.0, 5.0, 7.5)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--coarse-samples", type=int, default=50_000)
    parser.add_argument("--verify-samples", type=int, default=100_000)
    parser.add_argument("--cell-size", type=float, default=0.5)
    return parser.parse_args()


def as_numpy(value) -> np.ndarray:
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def unique_positions(positions) -> list[tuple[float, float, float]]:
    return sorted(
        {
            (round(float(x), 3), round(float(y), 3), round(float(z), 3))
            for x, y, z in positions
        },
        key=lambda p: (p[2], p[1], p[0]),
    )


def evaluate(
    scene_path: Path,
    positions: list[tuple[float, float, float]],
    samples: int,
    cell_size: float,
) -> list[dict[str, float]]:
    from sionna.rt import PlanarArray, RadioMapSolver, Transmitter, load_scene

    scene = load_scene(str(scene_path.resolve()), merge_shapes=True)
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
    for index, position in enumerate(positions):
        scene.add(
            Transmitter(
                name=f"candidate_{index}", position=position, power_dbm=33.0
            )
        )

    xmin, xmax, ymin, ymax = MAP_BOUNDS
    center_xy = [0.5 * (xmin + xmax), 0.5 * (ymin + ymax)]
    size_xy = [xmax - xmin, ymax - ymin]
    solver = RadioMapSolver()
    valid_by_height = []
    above_90_by_height = []
    above_100_by_height = []
    valid_mean_by_height = []

    for height in UAV_HEIGHTS:
        radio_map = solver(
            scene=scene,
            center=[center_xy[0], center_xy[1], height],
            orientation=[0.0, 0.0, 0.0],
            size=size_xy,
            cell_size=[cell_size, cell_size],
            samples_per_tx=samples,
            max_depth=2,
            los=True,
            specular_reflection=True,
            diffuse_reflection=False,
            refraction=True,
            diffraction=False,
            seed=42,
        )
        gain = np.asarray(as_numpy(radio_map.path_gain), dtype=np.float64)
        if gain.shape[0] != len(positions):
            raise RuntimeError(
                f"expected {len(positions)} transmitter maps, got {gain.shape}"
            )
        valid = gain > 1.0e-30
        power_dbm = 10.0 * np.log10(np.maximum(gain, 1.0e-30)) + 33.0
        valid_by_height.append(np.mean(valid, axis=(1, 2)))
        above_90_by_height.append(np.mean(power_dbm > -90.0, axis=(1, 2)))
        above_100_by_height.append(np.mean(power_dbm > -100.0, axis=(1, 2)))
        valid_mean_by_height.append(
            np.asarray(
                [
                    float(np.mean(values[mask])) if np.any(mask) else -300.0
                    for values, mask in zip(power_dbm, valid)
                ]
            )
        )
        print(f"evaluated z={height:g} m for {len(positions)} candidates", flush=True)

    valid_by_height = np.stack(valid_by_height, axis=1)
    above_90_by_height = np.stack(above_90_by_height, axis=1)
    above_100_by_height = np.stack(above_100_by_height, axis=1)
    valid_mean_by_height = np.stack(valid_mean_by_height, axis=1)
    rows = []
    for index, (x, y, z) in enumerate(positions):
        average_coverage = float(np.mean(above_100_by_height[index]))
        worst_coverage = float(np.min(above_100_by_height[index]))
        average_valid = float(np.mean(valid_by_height[index]))
        worst_valid = float(np.min(valid_by_height[index]))
        average_strong = float(np.mean(above_90_by_height[index]))
        worst_strong = float(np.min(above_90_by_height[index]))
        score = (
            0.40 * average_coverage
            + 0.25 * worst_coverage
            + 0.15 * average_valid
            + 0.10 * worst_valid
            + 0.07 * average_strong
            + 0.03 * worst_strong
        )
        row = {
            "phase_x_m": x,
            "phase_y_m": y,
            "phase_z_m": z,
            "model_z_m": z + 0.45,
            "score": score,
            "average_coverage_above_minus_100": average_coverage,
            "worst_height_coverage_above_minus_100": worst_coverage,
            "average_valid_fraction": average_valid,
            "worst_height_valid_fraction": worst_valid,
            "average_coverage_above_minus_90": average_strong,
            "worst_height_coverage_above_minus_90": worst_strong,
            "average_valid_rx_power_dbm": float(
                np.mean(valid_mean_by_height[index])
            ),
        }
        for height_index, height in enumerate(UAV_HEIGHTS):
            label = str(height).replace(".", "p")
            row[f"coverage_above_minus_100_z{label}"] = float(
                above_100_by_height[index, height_index]
            )
            row[f"valid_fraction_z{label}"] = float(
                valid_by_height[index, height_index]
            )
        rows.append(row)
    return sorted(rows, key=lambda row: row["score"], reverse=True)


def write_csv(path: Path, rows: list[dict[str, float]]) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    args = parse_args()
    if not args.scene.is_file():
        raise FileNotFoundError(args.scene)
    args.output.mkdir(parents=True, exist_ok=True)

    coarse = unique_positions(
        (x, y, z)
        for z in (5.55, 6.55, 7.25, 8.10)
        for y in (-10.0, -6.0, -2.0, 2.0, 6.0, 10.0, 14.0)
        for x in (-8.0, -5.0, -2.0, 1.0, 4.0, 7.0)
    )
    coarse_rows = evaluate(args.scene, coarse, args.coarse_samples, args.cell_size)
    write_csv(args.output / "coarse_scores.csv", coarse_rows)
    print("coarse top 5:", flush=True)
    for row in coarse_rows[:5]:
        print(json.dumps(row, sort_keys=True), flush=True)

    refinement_seeds = coarse_rows[:3]
    refined = unique_positions(
        (
            seed["phase_x_m"] + dx,
            seed["phase_y_m"] + dy,
            min(8.10, max(4.50, seed["phase_z_m"] + dz)),
        )
        for seed in refinement_seeds
        for dx in (-1.0, -0.5, 0.0, 0.5, 1.0)
        for dy in (-1.0, -0.5, 0.0, 0.5, 1.0)
        for dz in (-0.4, 0.0, 0.4)
    )
    refined_rows = evaluate(
        args.scene, refined, args.coarse_samples, args.cell_size
    )
    write_csv(args.output / "refined_scores.csv", refined_rows)
    print("refined top 10:", flush=True)
    for row in refined_rows[:10]:
        print(json.dumps(row, sort_keys=True), flush=True)

    verification_positions = unique_positions(
        [
            (row["phase_x_m"], row["phase_y_m"], row["phase_z_m"])
            for row in refined_rows[:10]
        ]
        + [(-0.5, 2.85, 5.55), (-0.5, 2.85, 8.10)]
    )
    verified_rows = evaluate(
        args.scene, verification_positions, args.verify_samples, args.cell_size
    )
    write_csv(args.output / "verified_scores.csv", verified_rows)
    recommended = verified_rows[0]
    report = {
        "method": {
            "frequency_hz": 28.0e9,
            "tx_power_dbm": 33.0,
            "max_depth": 2,
            "diffraction": False,
            "diffuse_reflection": False,
            "refraction": True,
            "coarse_samples_per_tx": args.coarse_samples,
            "verification_samples_per_tx": args.verify_samples,
            "cell_size_m": args.cell_size,
            "map_bounds_m": MAP_BOUNDS,
            "uav_heights_m": UAV_HEIGHTS,
        },
        "objective": (
            "Multi-height average and worst-height coverage, emphasizing "
            "received power above -100 dBm and valid ray paths."
        ),
        "recommended": recommended,
        "verified_candidates": verified_rows,
    }
    (args.output / "placement_report.json").write_text(
        json.dumps(report, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps(report, indent=2), flush=True)


if __name__ == "__main__":
    main()
