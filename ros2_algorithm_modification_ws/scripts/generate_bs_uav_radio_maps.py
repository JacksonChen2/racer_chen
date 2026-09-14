#!/usr/bin/env python3
"""Generate fixed-BS radio maps for several UAV flight heights."""

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
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-label", default="Warehouse")
    parser.add_argument("--bs-position", type=float, nargs=3, required=True)
    parser.add_argument(
        "--uav-heights", type=float, nargs="+", default=[0.75, 1.5, 3.0, 5.0, 7.5]
    )
    parser.add_argument("--x-min", type=float, default=-10.0)
    parser.add_argument("--x-max", type=float, default=9.0)
    parser.add_argument("--y-min", type=float, default=-11.9)
    parser.add_argument("--y-max", type=float, default=17.6)
    parser.add_argument("--cell-size", type=float, default=0.5)
    parser.add_argument("--samples", type=int, default=200_000)
    parser.add_argument("--max-depth", type=int, default=2)
    parser.add_argument("--diffraction", action="store_true")
    parser.add_argument("--frequency-hz", type=float, default=28.0e9)
    parser.add_argument("--bandwidth-hz", type=float, default=100.0e6)
    parser.add_argument("--tx-power-dbm", type=float, default=33.0)
    parser.add_argument(
        "--uplink-tx-power-dbm",
        type=float,
        help=(
            "UAV transmit power for reciprocal UAV-to-BS maps. When set, "
            "uplink received-power arrays and figures are emitted alongside "
            "the BS-to-UAV downlink outputs."
        ),
    )
    parser.add_argument("--seed", type=int, default=42)
    return parser.parse_args()


def as_numpy(value) -> np.ndarray:
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def height_label(height: float) -> str:
    return f"{height:g}".replace(".", "p")


def frequency_label(frequency_hz: float) -> str:
    if frequency_hz >= 1.0e9:
        return f"{frequency_hz / 1.0e9:g} GHz"
    if frequency_hz >= 1.0e6:
        return f"{frequency_hz / 1.0e6:g} MHz"
    return f"{frequency_hz:g} Hz"


def main() -> None:
    args = parse_args()
    if not args.scene.is_file():
        raise FileNotFoundError(args.scene)
    if args.cell_size <= 0.0 or args.samples <= 0:
        raise ValueError("cell size and sample count must be positive")
    if args.x_max <= args.x_min or args.y_max <= args.y_min:
        raise ValueError("radio-map bounds must have positive size")

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
        name="warehouse_bs",
        position=args.bs_position,
        power_dbm=args.tx_power_dbm,
    )
    scene.add(transmitter)
    solver = RadioMapSolver()

    center_xy = [
        0.5 * (args.x_min + args.x_max),
        0.5 * (args.y_min + args.y_max),
    ]
    size_xy = [args.x_max - args.x_min, args.y_max - args.y_min]
    heights = np.asarray(sorted(set(args.uav_heights)), dtype=np.float64)
    path_gain_maps = []
    rx_x = None
    rx_y = None

    for index, height in enumerate(heights):
        radio_map = solver(
            scene=scene,
            center=[center_xy[0], center_xy[1], float(height)],
            orientation=[0.0, 0.0, 0.0],
            size=size_xy,
            cell_size=[args.cell_size, args.cell_size],
            samples_per_tx=args.samples,
            max_depth=args.max_depth,
            los=True,
            specular_reflection=True,
            diffuse_reflection=False,
            refraction=True,
            diffraction=args.diffraction,
            seed=args.seed,
        )
        gain = np.asarray(as_numpy(radio_map.path_gain), dtype=np.float64)[0]
        gain_db = 10.0 * np.log10(np.maximum(gain, 1.0e-30))
        path_gain_maps.append(gain_db.astype(np.float32))
        centers = np.asarray(as_numpy(radio_map.cell_centers), dtype=np.float64)
        if rx_x is None:
            rx_x = centers[0, :, 0]
            rx_y = centers[:, 0, 1]
        print(
            f"height {index + 1}/{len(heights)}: z={height:g} m, "
            f"grid={gain.shape[1]}x{gain.shape[0]}",
            flush=True,
        )

    path_gain_db = np.stack(path_gain_maps)
    downlink_rx_power_dbm = path_gain_db + args.tx_power_dbm
    uplink_rx_power_dbm = (
        path_gain_db + args.uplink_tx_power_dbm
        if args.uplink_tx_power_dbm is not None
        else None
    )
    # Keep the historical name as a downlink alias for existing consumers.
    rx_power_dbm = downlink_rx_power_dbm
    finite_maps = [downlink_rx_power_dbm]
    if uplink_rx_power_dbm is not None:
        finite_maps.append(uplink_rx_power_dbm)
    finite_values = np.concatenate([values.ravel() for values in finite_maps])
    finite = finite_values[np.isfinite(finite_values)]
    color_min = float(np.percentile(finite, 2.0))
    color_max = float(np.percentile(finite, 98.0))
    color_min = max(-180.0, np.floor(color_min / 5.0) * 5.0)
    color_max = min(-20.0, np.ceil(color_max / 5.0) * 5.0)
    if color_max <= color_min:
        color_max = color_min + 10.0

    args.output.mkdir(parents=True, exist_ok=True)
    extent = [float(rx_x[0]), float(rx_x[-1]), float(rx_y[0]), float(rx_y[-1])]
    bs_x, bs_y, bs_z = args.bs_position
    carrier_label = frequency_label(args.frequency_hz)

    def draw(axis, values: np.ndarray, height: float) -> None:
        image = axis.imshow(
            values,
            origin="lower",
            extent=extent,
            cmap="turbo",
            vmin=color_min,
            vmax=color_max,
            interpolation="nearest",
            aspect="equal",
        )
        axis.scatter([bs_x], [bs_y], marker="*", s=110, c="white", edgecolors="black")
        axis.set_title(f"UAV height z = {height:g} m")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        return image

    def save_height_maps(
        values_by_height: np.ndarray,
        direction: str,
        tx_power_dbm: float,
    ) -> None:
        for height, values in zip(heights, values_by_height):
            figure, axis = plt.subplots(figsize=(7.4, 8.2), constrained_layout=True)
            image = draw(axis, values, float(height))
            colorbar = figure.colorbar(image, ax=axis, shrink=0.82)
            colorbar.set_label("Received power (dBm, isotropic path gain)")
            figure.suptitle(
                f"{args.scene_label} {direction} radio map\n"
                f"BS=({bs_x:g}, {bs_y:g}, {bs_z:g}) m, "
                f"{carrier_label}, transmitter={tx_power_dbm:g} dBm"
            )
            figure.savefig(
                args.output
                / f"{direction}_radio_map_z_{height_label(float(height))}m.png",
                dpi=180,
            )
            plt.close(figure)

    save_height_maps(downlink_rx_power_dbm, "downlink_bs_to_uav", args.tx_power_dbm)
    if uplink_rx_power_dbm is not None:
        save_height_maps(
            uplink_rx_power_dbm,
            "uplink_uav_to_bs",
            args.uplink_tx_power_dbm,
        )

    # Preserve the historical per-height downlink filenames.
    for height, values in zip(heights, downlink_rx_power_dbm):
        figure, axis = plt.subplots(figsize=(7.4, 8.2), constrained_layout=True)
        image = draw(axis, values, float(height))
        colorbar = figure.colorbar(image, ax=axis, shrink=0.82)
        colorbar.set_label("Received power (dBm, isotropic path gain)")
        figure.suptitle(
            f"{args.scene_label} BS-UAV radio map\n"
            f"BS=({bs_x:g}, {bs_y:g}, {bs_z:g}) m, "
            f"{carrier_label}, {args.tx_power_dbm:g} dBm"
        )
        figure.savefig(
            args.output / f"bs_uav_radio_map_z_{height_label(float(height))}m.png",
            dpi=180,
        )
        plt.close(figure)

    column_count = 3
    row_count = int(np.ceil(len(heights) / column_count))
    figure, axes = plt.subplots(
        row_count,
        column_count,
        figsize=(5.2 * column_count, 5.6 * row_count),
        constrained_layout=True,
        squeeze=False,
    )
    image = None
    for axis, height, values in zip(axes.flat, heights, rx_power_dbm):
        image = draw(axis, values, float(height))
    for axis in axes.flat[len(heights) :]:
        axis.set_visible(False)
    colorbar = figure.colorbar(image, ax=list(axes.flat), shrink=0.80)
    colorbar.set_label("Received power (dBm, isotropic path gain)")
    figure.suptitle(
        f"{args.scene_label} fixed BS to UAV | "
        f"BS=({bs_x:g}, {bs_y:g}, {bs_z:g}) m | "
        f"{carrier_label}, {args.tx_power_dbm:g} dBm",
        fontsize=14,
    )
    figure.savefig(args.output / "bs_uav_radio_maps_all_heights.png", dpi=180)
    plt.close(figure)

    if uplink_rx_power_dbm is not None:
        figure, axes = plt.subplots(
            row_count,
            column_count,
            figsize=(5.2 * column_count, 5.6 * row_count),
            constrained_layout=True,
            squeeze=False,
        )
        image = None
        for axis, height, values in zip(
            axes.flat, heights, uplink_rx_power_dbm
        ):
            image = draw(axis, values, float(height))
        for axis in axes.flat[len(heights) :]:
            axis.set_visible(False)
        colorbar = figure.colorbar(image, ax=list(axes.flat), shrink=0.80)
        colorbar.set_label("Received power (dBm, isotropic path gain)")
        figure.suptitle(
            f"{args.scene_label} UAV-to-BS uplink | "
            f"BS=({bs_x:g}, {bs_y:g}, {bs_z:g}) m | "
            f"{carrier_label}, UAV transmitter={args.uplink_tx_power_dbm:g} dBm",
            fontsize=14,
        )
        figure.savefig(
            args.output / "uplink_uav_to_bs_radio_maps_all_heights.png",
            dpi=180,
        )
        plt.close(figure)

    statistics = []
    for index, (height, gain_db, power_dbm) in enumerate(
        zip(heights, path_gain_db, downlink_rx_power_dbm)
    ):
        row = {
                "uav_height_m": float(height),
                "path_gain_mean_db": float(np.mean(gain_db)),
                "path_gain_p10_db": float(np.percentile(gain_db, 10.0)),
                "rx_power_mean_dbm": float(np.mean(power_dbm)),
                "rx_power_p10_dbm": float(np.percentile(power_dbm, 10.0)),
                "fraction_rx_power_above_minus_100_dbm": float(
                    np.mean(power_dbm > -100.0)
                ),
                "fraction_rx_power_above_minus_120_dbm": float(
                    np.mean(power_dbm > -120.0)
                ),
                "downlink_rx_power_mean_dbm": float(np.mean(power_dbm)),
                "downlink_rx_power_p10_dbm": float(
                    np.percentile(power_dbm, 10.0)
                ),
            }
        if uplink_rx_power_dbm is not None:
            uplink_power = uplink_rx_power_dbm[index]
            row.update(
                {
                    "uplink_rx_power_mean_dbm": float(np.mean(uplink_power)),
                    "uplink_rx_power_p10_dbm": float(
                        np.percentile(uplink_power, 10.0)
                    ),
                    "uplink_fraction_rx_power_above_minus_100_dbm": float(
                        np.mean(uplink_power > -100.0)
                    ),
                    "uplink_fraction_rx_power_above_minus_120_dbm": float(
                        np.mean(uplink_power > -120.0)
                    ),
                }
            )
        statistics.append(row)
    metadata = {
        "scene": str(args.scene.resolve()),
        "scene_label": args.scene_label,
        "bs_position_m": [float(value) for value in args.bs_position],
        "uav_heights_m": heights.tolist(),
        "bounds_m": [args.x_min, args.x_max, args.y_min, args.y_max],
        "cell_size_m": args.cell_size,
        "frequency_hz": args.frequency_hz,
        "bandwidth_hz": args.bandwidth_hz,
        "tx_power_dbm": args.tx_power_dbm,
        "downlink_tx_power_dbm": args.tx_power_dbm,
        "uplink_tx_power_dbm": args.uplink_tx_power_dbm,
        "samples_per_map": args.samples,
        "max_depth": args.max_depth,
        "diffraction": args.diffraction,
        "seed": args.seed,
        "radio_map_quantity": "isotropic path gain and received power; no UPA beamforming gain",
        "statistics": statistics,
    }
    (args.output / "radio_map_summary.json").write_text(
        json.dumps(metadata, indent=2) + "\n", encoding="utf-8"
    )
    np.savez_compressed(
        args.output / "bs_uav_radio_maps.npz",
        rx_x=rx_x,
        rx_y=rx_y,
        uav_heights=heights,
        path_gain_db=path_gain_db,
        rx_power_dbm=rx_power_dbm,
        downlink_rx_power_dbm=downlink_rx_power_dbm,
        uplink_rx_power_dbm=(
            uplink_rx_power_dbm
            if uplink_rx_power_dbm is not None
            else np.asarray([], dtype=np.float32)
        ),
        metadata_json=np.asarray(json.dumps(metadata, sort_keys=True)),
    )
    print(f"saved radio maps to {args.output}", flush=True)


if __name__ == "__main__":
    main()
