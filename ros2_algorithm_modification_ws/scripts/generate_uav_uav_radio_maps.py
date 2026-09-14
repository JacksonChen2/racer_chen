#!/usr/bin/env python3
"""Generate same-height UAV-to-UAV radio maps from sampled transmitter poses."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scene", type=Path, required=True)
    parser.add_argument("--points", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-label", default="Warehouse Full")
    parser.add_argument("--cell-size", type=float, default=0.5)
    parser.add_argument("--samples", type=int, default=100_000)
    parser.add_argument("--max-depth", type=int, default=2)
    parser.add_argument("--frequency-hz", type=float, default=28.0e9)
    parser.add_argument("--bandwidth-hz", type=float, default=100.0e6)
    parser.add_argument("--tx-power-dbm", type=float, default=23.0)
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def as_numpy(value) -> np.ndarray:
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def height_label(height: float) -> str:
    return f"{height:g}".replace(".", "p")


def main() -> None:
    args = parse_args()
    selection = json.loads(args.points.read_text(encoding="utf-8"))
    x_min, x_max, y_min, y_max = map(float, selection["bounds_m"])
    rows, cols = map(int, selection["grid_shape"])
    expected = rows * cols
    if expected < 1:
        raise ValueError(f"expected at least one point per height, got {expected}")

    from sionna.rt import PlanarArray, RadioMapSolver, Transmitter, load_scene

    scene = load_scene(str(args.scene.resolve()), merge_shapes=True)
    scene.frequency = args.frequency_hz
    scene.bandwidth = args.bandwidth_hz
    scene.tx_array = PlanarArray(
        num_rows=1,
        num_cols=1,
        vertical_spacing=0.5,
        horizontal_spacing=0.5,
        pattern="iso",
        polarization="V",
    )
    transmitter = Transmitter(
        name="uav_tx",
        position=[0.0, 0.0, 1.0],
        power_dbm=args.tx_power_dbm,
    )
    scene.add(transmitter)
    solver = RadioMapSolver()
    center = [0.5 * (x_min + x_max), 0.5 * (y_min + y_max)]
    size = [x_max - x_min, y_max - y_min]

    all_gains = []
    all_points = []
    all_heights = []
    rx_x = None
    rx_y = None
    point_statistics = []
    args.output.mkdir(parents=True, exist_ok=True)

    for height_index, height_entry in enumerate(selection["heights"]):
        height = float(height_entry["height_m"])
        points = np.asarray(
            [item["position"] for item in height_entry["points"]], dtype=np.float64
        )
        if points.shape != (expected, 3):
            raise ValueError(f"height {height:g} has point array {points.shape}")
        height_gains = []
        for point_index, point in enumerate(points):
            transmitter.position = point.tolist()
            radio_map = solver(
                scene=scene,
                center=[center[0], center[1], height],
                orientation=[0.0, 0.0, 0.0],
                size=size,
                cell_size=[args.cell_size, args.cell_size],
                samples_per_tx=args.samples,
                max_depth=args.max_depth,
                los=True,
                specular_reflection=True,
                diffuse_reflection=False,
                refraction=True,
                diffraction=False,
                seed=args.seed,
            )
            gain = np.asarray(as_numpy(radio_map.path_gain), dtype=np.float64)[0]
            gain_db = 10.0 * np.log10(np.maximum(gain, 1.0e-30))
            height_gains.append(gain_db.astype(np.float32))
            centers = np.asarray(as_numpy(radio_map.cell_centers), dtype=np.float64)
            if rx_x is None:
                rx_x = centers[0, :, 0]
                rx_y = centers[:, 0, 1]
            power = gain_db + args.tx_power_dbm
            point_statistics.append(
                {
                    "height_m": height,
                    "point_id": point_index + 1,
                    "tx_position_m": point.tolist(),
                    "rx_power_mean_dbm": float(np.mean(power)),
                    "fraction_rx_power_above_minus_100_dbm": float(
                        np.mean(power > -100.0)
                    ),
                    "fraction_rx_power_above_minus_120_dbm": float(
                        np.mean(power > -120.0)
                    ),
                }
            )
            print(
                f"height {height_index + 1}/{len(selection['heights'])} "
                f"z={height:g}, point {point_index + 1}/{expected}",
                flush=True,
            )

        gains = np.stack(height_gains)
        powers = gains + args.tx_power_dbm
        all_gains.append(gains)
        all_points.append(points)
        all_heights.append(height)

        figure, axes = plt.subplots(
            rows,
            cols,
            figsize=(22, 22),
            constrained_layout=True,
            squeeze=False,
        )
        image = None
        extent = [float(rx_x[0]), float(rx_x[-1]), float(rx_y[0]), float(rx_y[-1])]
        for index, (axis, point, values) in enumerate(
            zip(axes.flat, points, powers), start=1
        ):
            image = axis.imshow(
                values,
                origin="lower",
                extent=extent,
                cmap="turbo",
                vmin=-180.0,
                vmax=-40.0,
                interpolation="nearest",
                aspect="equal",
            )
            axis.scatter(
                [point[0]], [point[1]], marker="*", s=65,
                c="white", edgecolors="black", linewidths=0.7,
            )
            axis.set_title(
                f"P{index:02d}: ({point[0]:.1f}, {point[1]:.1f})",
                fontsize=9,
            )
            axis.tick_params(labelsize=7)
            if index > expected - cols:
                axis.set_xlabel("x (m)", fontsize=8)
            if (index - 1) % cols == 0:
                axis.set_ylabel("y (m)", fontsize=8)
        colorbar = figure.colorbar(image, ax=list(axes.flat), shrink=0.82)
        colorbar.set_label("UAV receiver power (dBm, isotropic path gain)")
        figure.suptitle(
            f"{args.scene_label} UAV-to-UAV radio maps | z={height:g} m | "
            f"{expected} transmitter points | "
            f"{args.frequency_hz / 1.0e9:g} GHz, {args.tx_power_dbm:g} dBm",
            fontsize=15,
        )
        figure.savefig(
            args.output
            / f"uav_uav_{expected}points_z_{height_label(height)}m.png",
            dpi=160,
        )
        plt.close(figure)

    path_gain_db = np.stack(all_gains)
    tx_points = np.stack(all_points)
    heights = np.asarray(all_heights, dtype=np.float64)
    rx_power_dbm = path_gain_db + args.tx_power_dbm
    metadata = {
        "scene": str(args.scene.resolve()),
        "scene_label": args.scene_label,
        "point_selection": str(args.points.resolve()),
        "points_per_height": expected,
        "uav_heights_m": heights.tolist(),
        "bounds_m": [x_min, x_max, y_min, y_max],
        "cell_size_m": args.cell_size,
        "frequency_hz": args.frequency_hz,
        "bandwidth_hz": args.bandwidth_hz,
        "uav_tx_power_dbm": args.tx_power_dbm,
        "samples_per_map": args.samples,
        "max_depth": args.max_depth,
        "diffraction": False,
        "seed": args.seed,
        "receiver_plane": "same height as each transmitter",
        "radio_map_quantity": "isotropic path gain and received power",
        "point_statistics": point_statistics,
    }
    (args.output / "uav_uav_radio_map_summary.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    np.savez_compressed(
        args.output / "uav_uav_radio_maps.npz",
        rx_x=rx_x,
        rx_y=rx_y,
        uav_heights=heights,
        tx_points=tx_points,
        path_gain_db=path_gain_db,
        rx_power_dbm=rx_power_dbm,
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    print(f"saved UAV-UAV maps to {args.output}", flush=True)


if __name__ == "__main__":
    main()
