#!/usr/bin/env python3
"""Compare coverage and trajectories for two Sionna bandwidth experiments."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from generate_scene_top_view import draw_geometry, load_projected_geometry


BOUNDS = (-27.0, 6.0, 0.6, 30.6)
TEXT = "#172b4d"
MUTED = "#52667a"
BLUE = "#1677ff"
ORANGE = "#e67700"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--reference", type=Path, required=True)
    parser.add_argument("--experiment", type=Path, required=True)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def choose_font() -> str:
    candidates = (
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans CN",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    )
    installed = {item.name for item in font_manager.fontManager.ttflist}
    return next((name for name in candidates if name in installed), "DejaVu Sans")


def joint_coverage(result: dict) -> tuple[np.ndarray, np.ndarray]:
    metrics = result["metrics"]
    elapsed = float(metrics["elapsed"])
    times = np.arange(0.0, elapsed + 1.0, 1.0)
    curves = []
    for drone_id in range(int(result["drone_count"])):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in metrics["mapping_coverage_history"]
                if int(item["drone_id"]) == drone_id
            ),
            key=lambda item: item[0],
        )
        sample_times = np.asarray(
            [0.0] + [item[0] for item in records] + [elapsed], dtype=float
        )
        sample_values = np.asarray(
            [0.0]
            + [item[1] for item in records]
            + [float(metrics["mapping_coverage_per_agent"][drone_id])],
            dtype=float,
        )
        unique_times, inverse = np.unique(sample_times, return_inverse=True)
        unique_values = np.zeros_like(unique_times)
        np.maximum.at(unique_values, inverse, sample_values)
        curves.append(
            np.interp(times, unique_times, np.maximum.accumulate(unique_values))
        )
    coverage = np.maximum.accumulate(np.max(np.vstack(curves), axis=0))
    coverage[-1] = float(metrics["mapping_coverage_joint"])
    return times, 100.0 * coverage


def draw_trajectories(
    axis: plt.Axes,
    result: dict,
    geometry,
    title: str,
    panel: str,
    colors: np.ndarray,
) -> None:
    metrics = result["metrics"]
    history = np.asarray(
        [item["positions"] for item in metrics["trajectory_history"]], dtype=float
    )
    starts = np.asarray(metrics["start_positions"], dtype=float)
    ends = np.asarray(metrics["positions"], dtype=float)
    for index in range(int(result["drone_count"])):
        axis.plot(
            history[:, index, 0],
            history[:, index, 1],
            color=colors[index],
            linewidth=1.8,
            alpha=0.95,
            zorder=5,
        )
        axis.scatter(
            starts[index, 0], starts[index, 1], marker="*", s=100,
            color=colors[index], edgecolor="black", linewidth=0.45, zorder=8,
        )
        axis.scatter(
            ends[index, 0], ends[index, 1], marker="X", s=45,
            color=colors[index], edgecolor="white", linewidth=0.5, zorder=8,
        )
        axis.annotate(
            f"U{index + 1}", ends[index, :2], xytext=(3, 3),
            textcoords="offset points", fontsize=7.2, color=colors[index],
            fontweight="bold", zorder=9,
        )
    draw_geometry(axis, geometry, BOUNDS)
    stats = result["communication"]["statistics"]
    axis.set_title(
        f"{panel}  {title}\n"
        f"覆盖 {100.0 * metrics['mapping_coverage_joint']:.2f}%  ·  "
        f"总航程 {sum(metrics['path_lengths']):.1f} m  ·  "
        f"逻辑投递率 {100.0 * stats['logical_delivery_ratio']:.2f}%",
        loc="left", fontsize=12, fontweight="bold", color=TEXT,
    )
    axis.set_xlabel("世界坐标 X / m")
    axis.set_ylabel("世界坐标 Y / m")


def main() -> None:
    args = parse_args()
    reference = load(args.reference)
    experiment = load(args.experiment)
    if reference["metrics"]["start_positions"] != experiment["metrics"]["start_positions"]:
        raise ValueError("the two runs do not use identical start positions")
    geometry = load_projected_geometry(args.mesh_dir, 0.25, 8.4)
    n_uavs = int(reference["drone_count"])
    colors = plt.get_cmap("tab10")(np.arange(n_uavs))

    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "axes.edgecolor": "#9fb3c8",
            "axes.labelcolor": TEXT,
            "xtick.color": MUTED,
            "ytick.color": MUTED,
        }
    )
    figure = plt.figure(figsize=(17, 11.5), facecolor="#f7f9fc")
    grid = figure.add_gridspec(
        2, 2, left=0.06, right=0.975, bottom=0.11, top=0.86,
        height_ratios=(0.72, 1.45), hspace=0.36, wspace=0.2,
    )
    coverage_axis = figure.add_subplot(grid[0, :])
    reference_axis = figure.add_subplot(grid[1, 0])
    experiment_axis = figure.add_subplot(grid[1, 1])

    figure.suptitle(
        "10-UAV 无 BS Sionna 分布式通信：带宽效果对比",
        x=0.06, y=0.965, ha="left", fontsize=22, fontweight="bold", color=TEXT,
    )
    figure.text(
        0.06, 0.92,
        "seed 42 · 相同 5 起飞点 · 300 s · 20 dBm · 固定 MCS14 · 0 次重传 · 共享 UAV OFDMA",
        fontsize=11.5, color=MUTED,
    )

    for result, color, label in (
        (reference, BLUE, "100 MHz / 66 RB（参考）"),
        (experiment, ORANGE, "50 MHz / 32 RB"),
    ):
        times, coverage = joint_coverage(result)
        coverage_axis.plot(times, coverage, color=color, linewidth=2.7, label=label)
        coverage_axis.scatter(times[-1], coverage[-1], color=color, s=42, zorder=5)
        coverage_axis.annotate(
            f"{coverage[-1]:.2f}%", (times[-1], coverage[-1]),
            xytext=(-8, 8), textcoords="offset points", ha="right",
            color=color, fontsize=10.5, fontweight="bold",
        )
    coverage_axis.set_title(
        "A  联合地图覆盖率变化", loc="left", fontsize=13,
        fontweight="bold", color=TEXT,
    )
    coverage_axis.set_xlim(0, 300)
    coverage_axis.set_ylim(0, 75)
    coverage_axis.set_xlabel("仿真时间 / s")
    coverage_axis.set_ylabel("已知体素覆盖率 / %")
    coverage_axis.grid(color="#d9e2ec", linewidth=0.8, alpha=0.9)
    coverage_axis.legend(frameon=False, loc="upper left", ncol=2)
    coverage_axis.set_facecolor("white")

    draw_trajectories(
        reference_axis, reference, geometry, "100 MHz / 66 RB 俯视轨迹",
        "B", colors,
    )
    draw_trajectories(
        experiment_axis, experiment, geometry, "50 MHz / 32 RB 俯视轨迹",
        "C", colors,
    )

    handles = [
        Line2D([0], [0], color=colors[index], lw=2, label=f"UAV {index + 1}")
        for index in range(n_uavs)
    ]
    handles.extend(
        [
            Line2D([0], [0], marker="*", linestyle="", markerfacecolor="#777",
                   markeredgecolor="black", markersize=10, label="起点"),
            Line2D([0], [0], marker="X", linestyle="", markerfacecolor="#777",
                   markeredgecolor="white", markersize=8, label="终点"),
        ]
    )
    figure.legend(
        handles=handles, loc="lower center", bbox_to_anchor=(0.5, 0.035),
        ncol=12, frameon=False, fontsize=8.5,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=200, facecolor=figure.get_facecolor())
    figure.savefig(args.output.with_suffix(".svg"), facecolor=figure.get_facecolor())
    plt.close(figure)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
