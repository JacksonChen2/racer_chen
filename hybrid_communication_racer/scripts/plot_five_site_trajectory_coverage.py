#!/usr/bin/env python3
"""Plot coverage history and top-view trajectories for a paired 10-UAV run."""

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
IDEAL_COLOR = "#1677ff"
SIONNA_COLOR = "#f59e0b"
TEXT_COLOR = "#172b4d"
MUTED_COLOR = "#52667a"
GRID_COLOR = "#d9e2ec"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ideal", type=Path, required=True)
    parser.add_argument("--sionna", type=Path, required=True)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--seed", type=int, required=True)
    return parser.parse_args()


def load_result(path: Path) -> dict:
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


def joint_coverage_series(result: dict) -> tuple[np.ndarray, np.ndarray]:
    """Synchronize asynchronous per-UAV map snapshots at one-second intervals."""
    metrics = result["metrics"]
    elapsed = float(metrics["elapsed"])
    times = np.arange(0.0, elapsed + 1.0e-6, 1.0)
    if times[-1] < elapsed:
        times = np.append(times, elapsed)

    curves = []
    history = metrics["mapping_coverage_history"]
    for drone_id in range(int(result["drone_count"])):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in history
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

    joint = np.maximum.accumulate(np.max(np.vstack(curves), axis=0))
    joint[-1] = float(metrics["mapping_coverage_joint"])
    return times, joint * 100.0


def draw_trajectory_panel(
    axis: plt.Axes,
    result: dict,
    geometry,
    title: str,
    panel: str,
) -> None:
    metrics = result["metrics"]
    samples = metrics["trajectory_history"]
    positions = np.asarray([sample["positions"] for sample in samples], dtype=float)
    starts = np.asarray(metrics["start_positions"], dtype=float)
    ends = np.asarray(metrics["positions"], dtype=float)
    paths = np.asarray(metrics["path_lengths"], dtype=float)
    executed = set(int(value) for value in result["executed_drone_ids"])
    colors = plt.get_cmap("tab10")(np.arange(10))

    draw_geometry(axis, geometry, BOUNDS)
    for index in range(10):
        drone_id = index + 1
        trajectory = positions[:, index, :]
        active = drone_id in executed
        axis.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            color=colors[index],
            linewidth=1.9 if active else 1.1,
            linestyle="-" if active else "--",
            alpha=0.95 if active else 0.35,
            zorder=5,
        )
        axis.scatter(
            starts[index, 0],
            starts[index, 1],
            marker="*",
            s=105,
            color=colors[index],
            edgecolor="black",
            linewidth=0.45,
            zorder=8,
        )
        axis.scatter(
            ends[index, 0],
            ends[index, 1],
            marker="X",
            s=48,
            color=colors[index],
            edgecolor="white",
            linewidth=0.55,
            alpha=1.0 if active else 0.55,
            zorder=8,
        )
        axis.annotate(
            f"U{drone_id}",
            ends[index, :2],
            xytext=(3, 3),
            textcoords="offset points",
            fontsize=7.5,
            color=colors[index],
            fontweight="bold",
            alpha=1.0 if active else 0.65,
            zorder=9,
        )

    coverage = 100.0 * float(metrics["mapping_coverage_joint"])
    axis.set_title(
        f"{panel}  {title}\n"
        f"覆盖 {coverage:.2f}% · 航程 {paths.sum():.1f} m · 执行 {len(executed)}/10",
        loc="left",
        fontsize=12.5,
        fontweight="bold",
        color=TEXT_COLOR,
    )
    axis.set_xlabel("世界坐标 X / m")
    axis.set_ylabel("世界坐标 Y / m")


def main() -> None:
    args = parse_args()
    ideal = load_result(args.ideal)
    sionna = load_result(args.sionna)
    geometry = load_projected_geometry(args.mesh_dir, 0.25, 8.4)
    ideal_executed = [int(value) for value in ideal["executed_drone_ids"]]
    sionna_executed = [int(value) for value in sionna["executed_drone_ids"]]

    def execution_summary(label: str, executed: list[int]) -> str:
        if len(executed) == 10:
            return f"{label}：10/10 架执行，完整性验证通过"
        ids = "、".join(f"U{value}" for value in executed)
        return f"{label}：仅 {ids} 执行，完整性验证失败"

    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "axes.edgecolor": "#9fb3c8",
            "axes.labelcolor": TEXT_COLOR,
            "xtick.color": MUTED_COLOR,
            "ytick.color": MUTED_COLOR,
        }
    )
    figure = plt.figure(figsize=(17, 12), facecolor="#f7f9fc")
    grid = figure.add_gridspec(
        2,
        2,
        left=0.06,
        right=0.975,
        bottom=0.105,
        top=0.84,
        height_ratios=(0.72, 1.45),
        hspace=0.37,
        wspace=0.20,
    )
    coverage_axis = figure.add_subplot(grid[0, :])
    ideal_axis = figure.add_subplot(grid[1, 0])
    sionna_axis = figure.add_subplot(grid[1, 1])

    figure.suptitle(
        "五起飞点 10-UAV：轨迹与覆盖率变化",
        x=0.06,
        y=0.965,
        ha="left",
        fontsize=22,
        fontweight="bold",
        color=TEXT_COLOR,
    )
    figure.text(
        0.06,
        0.923,
        f"seed={args.seed} · 300 s · 100 Hz 物理 · 10 Hz 传感器 · 76,800 rays · 同一起飞位置",
        fontsize=11.5,
        color=MUTED_COLOR,
    )
    figure.text(
        0.06,
        0.884,
        execution_summary("完美通信", ideal_executed),
        fontsize=11.5,
        color=IDEAL_COLOR,
        fontweight="bold",
    )
    figure.text(
        0.34,
        0.884,
        execution_summary("Sionna", sionna_executed),
        fontsize=11.5,
        color="#b54708",
        fontweight="bold",
    )

    curve_specs = (
        (ideal, IDEAL_COLOR, "完美通信", -17),
        (sionna, SIONNA_COLOR, "Sionna 分布式", 8),
    )
    final_values = []
    for result, color, label, annotation_y in curve_specs:
        times, coverage = joint_coverage_series(result)
        final_values.append(float(coverage[-1]))
        coverage_axis.plot(times, coverage, color=color, linewidth=2.7, label=label)
        coverage_axis.scatter(times[-1], coverage[-1], s=38, color=color, zorder=5)
        coverage_axis.annotate(
            f"{coverage[-1]:.2f}%",
            (times[-1], coverage[-1]),
            xytext=(-8, annotation_y),
            textcoords="offset points",
            ha="right",
            color=color,
            fontweight="bold",
        )
    coverage_axis.set_title(
        "A  联合地图覆盖率随仿真时间变化",
        loc="left",
        fontsize=13,
        fontweight="bold",
        color=TEXT_COLOR,
    )
    coverage_axis.set_xlabel("仿真时间 / s")
    coverage_axis.set_ylabel("已知体素覆盖率 / %")
    coverage_axis.set_xlim(0, 300)
    coverage_axis.set_ylim(0, max(45.0, np.ceil(max(final_values) / 5.0) * 5.0 + 5.0))
    coverage_axis.grid(color=GRID_COLOR, linewidth=0.8, alpha=0.85)
    coverage_axis.set_axisbelow(True)
    coverage_axis.legend(frameon=False, loc="upper left", ncol=2)
    coverage_axis.set_facecolor("white")

    draw_trajectory_panel(ideal_axis, ideal, geometry, "完美通信俯视轨迹", "B")
    draw_trajectory_panel(sionna_axis, sionna, geometry, "Sionna 分布式俯视轨迹", "C")

    colors = plt.get_cmap("tab10")(np.arange(10))
    legend_handles = [
        Line2D([0], [0], color=colors[index], lw=2, label=f"UAV {index + 1}")
        for index in range(10)
    ]
    legend_handles.extend(
        [
            Line2D(
                [0], [0], marker="*", linestyle="", markerfacecolor="#777777",
                markeredgecolor="black", markersize=10, label="起点",
            ),
            Line2D(
                [0], [0], marker="X", linestyle="", markerfacecolor="#777777",
                markeredgecolor="white", markersize=8, label="终点",
            ),
            Line2D(
                [0], [0], color="#777777", lw=1.2, linestyle="--",
                alpha=0.5, label="未执行 FSM 轨迹",
            ),
        ]
    )
    figure.legend(
        handles=legend_handles,
        loc="lower center",
        bbox_to_anchor=(0.5, 0.048),
        ncol=13,
        frameon=False,
        fontsize=8.8,
    )
    figure.text(
        0.06,
        0.018,
        "注：轨迹由约 0.5 s 间隔的位置历史重建；灰色/蓝灰色区域为仓库障碍物俯视投影。"
        "覆盖率曲线由各 UAV 异步地图快照同步后取联合累计包络。",
        fontsize=9.7,
        color=MUTED_COLOR,
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=200, facecolor=figure.get_facecolor())
    figure.savefig(args.output.with_suffix(".svg"), facecolor=figure.get_facecolor())
    plt.close(figure)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
