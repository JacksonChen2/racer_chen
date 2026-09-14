#!/usr/bin/env python3
"""Plot recorded multi-UAV trajectories and mapping-coverage history."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path, help="Experiment result JSON")
    parser.add_argument("output", type=Path, help="Output PNG")
    parser.add_argument("--xmin", type=float, default=-27.0)
    parser.add_argument("--xmax", type=float, default=6.0)
    parser.add_argument("--ymin", type=float, default=0.6)
    parser.add_argument("--ymax", type=float, default=30.6)
    parser.add_argument("--zmin", type=float, default=0.4)
    parser.add_argument("--zmax", type=float, default=8.4)
    return parser.parse_args()


def coverage_series(history: list[dict], drone_count: int):
    per_drone = []
    for drone_id in range(drone_count):
        samples = [sample for sample in history if sample["drone_id"] == drone_id]
        samples.sort(key=lambda sample: sample["time_s"])
        per_drone.append(
            (
                np.asarray([sample["time_s"] for sample in samples]),
                np.asarray([sample["ratio"] for sample in samples]),
            )
        )

    common_length = min((len(times) for times, _ in per_drone), default=0)
    if common_length:
        joint_time = np.mean(
            np.vstack([times[:common_length] for times, _ in per_drone]), axis=0
        )
        joint_ratio = np.max(
            np.vstack([ratios[:common_length] for _, ratios in per_drone]), axis=0
        )
    else:
        joint_time = np.empty(0)
        joint_ratio = np.empty(0)
    return per_drone, joint_time, joint_ratio


def main() -> None:
    args = parse_args()
    with args.result.open("r", encoding="utf-8") as stream:
        result = json.load(stream)

    metrics = result["metrics"]
    drone_count = int(result["drone_count"])
    history = metrics["trajectory_history"]
    positions = np.asarray([sample["positions"] for sample in history], dtype=float)
    times = np.asarray([sample["time_s"] for sample in history], dtype=float)
    starts = np.asarray(metrics["start_positions"], dtype=float)
    ends = np.asarray(metrics["positions"], dtype=float)
    path_lengths = np.asarray(metrics["path_lengths"], dtype=float)
    final_coverage = float(metrics["mapping_coverage_joint"])
    elapsed = float(metrics["elapsed"])

    colors = plt.get_cmap("tab10")(np.arange(drone_count))
    fig = plt.figure(figsize=(19, 7.3), constrained_layout=True)
    grid = fig.add_gridspec(1, 3, width_ratios=(1.12, 1.12, 1.0))
    ax_top = fig.add_subplot(grid[0, 0])
    ax_3d = fig.add_subplot(grid[0, 1], projection="3d")
    ax_cov = fig.add_subplot(grid[0, 2])

    # Top-down trajectory view in the configured mapping coverage box.
    ax_top.set_facecolor("#f4f5f7")
    ax_top.fill_between(
        [args.xmin, args.xmax], args.ymin, args.ymax, color="#e8edf3", zorder=0
    )
    for drone_id in range(drone_count):
        trajectory = positions[:, drone_id, :]
        ax_top.plot(
            trajectory[:, 0], trajectory[:, 1], color=colors[drone_id], lw=1.8,
            alpha=0.9, label=f"UAV {drone_id + 1} ({path_lengths[drone_id]:.1f} m)"
        )
        ax_top.scatter(
            starts[drone_id, 0], starts[drone_id, 1], marker="*", s=105,
            color=colors[drone_id], edgecolor="black", linewidth=0.45, zorder=5
        )
        ax_top.scatter(
            ends[drone_id, 0], ends[drone_id, 1], marker="X", s=55,
            color=colors[drone_id], edgecolor="white", linewidth=0.55, zorder=5
        )
        ax_top.annotate(
            str(drone_id + 1), (ends[drone_id, 0], ends[drone_id, 1]),
            xytext=(4, 4), textcoords="offset points", fontsize=8,
            color=colors[drone_id], weight="bold"
        )
    ax_top.set_xlim(args.xmin, args.xmax)
    ax_top.set_ylim(args.ymin, args.ymax)
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.set_xlabel("World X (m)")
    ax_top.set_ylabel("World Y (m)")
    ax_top.set_title("Top-down exploration trajectories")
    ax_top.grid(True, color="white", linewidth=1.1)

    # 3D trajectory view; a faint box indicates the coverage-measurement volume.
    for drone_id in range(drone_count):
        trajectory = positions[:, drone_id, :]
        ax_3d.plot(
            trajectory[:, 0], trajectory[:, 1], trajectory[:, 2],
            color=colors[drone_id], lw=1.5, alpha=0.92
        )
        ax_3d.scatter(*starts[drone_id], marker="*", s=65, color=colors[drone_id])
        ax_3d.scatter(*ends[drone_id], marker="X", s=35, color=colors[drone_id])
    ax_3d.set_xlim(args.xmin, args.xmax)
    ax_3d.set_ylim(args.ymin, args.ymax)
    ax_3d.set_zlim(args.zmin, args.zmax)
    ax_3d.set_xlabel("X (m)")
    ax_3d.set_ylabel("Y (m)")
    ax_3d.set_zlabel("Z (m)")
    ax_3d.set_title("3D flown trajectories")
    ax_3d.view_init(elev=30, azim=-62)
    ax_3d.set_box_aspect((args.xmax - args.xmin, args.ymax - args.ymin, 14.0))

    # Per-agent and fused-map mapping coverage.
    per_drone, joint_time, joint_ratio = coverage_series(
        metrics["mapping_coverage_history"], drone_count
    )
    for drone_id, (coverage_time, coverage_ratio) in enumerate(per_drone):
        ax_cov.plot(
            coverage_time, 100.0 * coverage_ratio, color=colors[drone_id],
            lw=1.0, alpha=0.48
        )
    if len(joint_time):
        ax_cov.plot(
            joint_time, 100.0 * joint_ratio, color="#161616", lw=2.7,
            label="Joint (max peer-fused map)"
        )
    ax_cov.scatter(
        [elapsed], [100.0 * final_coverage], marker="o", s=58,
        color="#d62728", edgecolor="white", linewidth=0.7, zorder=5
    )
    ax_cov.annotate(
        f"Final: {100.0 * final_coverage:.2f}%",
        xy=(elapsed, 100.0 * final_coverage), xytext=(-92, 18),
        textcoords="offset points", fontsize=11, weight="bold",
        arrowprops={"arrowstyle": "->", "color": "#d62728"}
    )
    ax_cov.set_xlim(0.0, elapsed * 1.02)
    ax_cov.set_ylim(0.0, max(50.0, 100.0 * final_coverage + 6.0))
    ax_cov.set_xlabel("Simulation time (s)")
    ax_cov.set_ylabel("Known voxels / planning-box voxels (%)")
    ax_cov.set_title("Mapping coverage over time")
    ax_cov.grid(True, alpha=0.28)
    ax_cov.legend(loc="lower right", frameon=True)

    trajectory_handles = [
        Line2D([0], [0], color=colors[index], lw=2,
               label=f"UAV {index + 1}: {path_lengths[index]:.1f} m")
        for index in range(drone_count)
    ]
    marker_handles = [
        Line2D([0], [0], marker="*", color="none", markerfacecolor="#777777",
               markeredgecolor="black", markersize=10, label="Start"),
        Line2D([0], [0], marker="X", color="none", markerfacecolor="#777777",
               markeredgecolor="white", markersize=8, label="Final position"),
    ]
    ax_top.legend(
        handles=trajectory_handles + marker_handles, loc="upper left", ncol=2,
        fontsize=7.7, framealpha=0.92
    )

    total_path = float(np.sum(path_lengths))
    fig.suptitle(
        "10-UAV RACER — Ideal Communication, Central Takeoff\n"
        f"300 s | joint coverage {100.0 * final_coverage:.2f}% | "
        f"total path {total_path:.1f} m | 0 collisions",
        fontsize=16, weight="bold"
    )
    fig.text(
        0.5, 0.006,
        "Visualization reconstructed from the saved trajectory and coverage histories. "
        "The occupied/free/unknown voxel field was not persisted by this run.",
        ha="center", fontsize=9, color="#555555"
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=220, bbox_inches="tight", facecolor="white")
    plt.close(fig)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
