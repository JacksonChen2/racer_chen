#!/usr/bin/env python3
"""Plot UAV trajectories and coverage history from a RACER result."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


BS_POSITION = np.asarray((-10.028926, 14.888599, 7.55), dtype=float)
WORLD_LIMITS = (-27.0, 6.0, 0.6, 30.6)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def choose_font() -> str:
    candidates = (
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans CN",
        "Droid Sans Fallback",
        "DejaVu Sans",
    )
    installed = {item.name for item in font_manager.fontManager.ttflist}
    return next((font for font in candidates if font in installed), "DejaVu Sans")


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "axes.edgecolor": "#9aa9b8",
            "axes.labelcolor": "#243447",
            "xtick.color": "#52667a",
            "ytick.color": "#52667a",
        }
    )


def aligned_coverage(
    result: dict, interval_s: float = 1.0
) -> tuple[np.ndarray, np.ndarray]:
    """Align asynchronous per-UAV coverage samples by hold-last interpolation."""

    metrics = result["metrics"]
    drone_count = int(result["drone_count"])
    end_time = float(metrics["elapsed"])
    times = np.arange(0.0, end_time, interval_s, dtype=float)
    times = np.append(times, end_time)
    history = metrics["mapping_coverage_history"]
    final_ratios = metrics["mapping_coverage_per_agent"]

    per_agent: list[np.ndarray] = []
    for drone_id in range(drone_count):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in history
                if int(item["drone_id"]) == drone_id
            ),
            key=lambda item: item[0],
        )
        sample_t = np.asarray(
            [0.0] + [item[0] for item in records] + [end_time], dtype=float
        )
        sample_y = np.asarray(
            [0.0] + [item[1] for item in records] + [final_ratios[drone_id]],
            dtype=float,
        )
        unique_t, inverse = np.unique(sample_t, return_inverse=True)
        unique_y = np.zeros_like(unique_t)
        np.maximum.at(unique_y, inverse, sample_y)
        unique_y = np.maximum.accumulate(unique_y)
        # Mapping reports arrive asynchronously. Use the most recent observed
        # value instead of inventing intermediate coverage by linear blending.
        indices = np.searchsorted(unique_t, times, side="right") - 1
        indices = np.clip(indices, 0, len(unique_y) - 1)
        per_agent.append(unique_y[indices])

    values = 100.0 * np.vstack(per_agent)
    values[:, -1] = 100.0 * np.asarray(final_ratios, dtype=float)
    return times, values


def plot(result: dict, output: Path) -> None:
    configure_style()
    metrics = result["metrics"]
    history = metrics["trajectory_history"]
    positions = np.asarray([item["positions"] for item in history], dtype=float)
    trajectory_times = np.asarray([item["time_s"] for item in history], dtype=float)
    starts = np.asarray(metrics["start_positions"], dtype=float)
    ends = np.asarray(metrics["positions"], dtype=float)
    path_lengths = np.asarray(metrics["path_lengths"], dtype=float)
    drone_count = int(result["drone_count"])
    final_coverage = 100.0 * float(metrics["mapping_coverage_joint"])
    end_time = float(metrics["elapsed"])
    statistics = result.get("communication", {}).get("statistics", {})
    has_bs = bool(statistics.get("ap_enabled", False))
    colors = plt.get_cmap("tab10")(np.arange(drone_count))

    figure = plt.figure(figsize=(16.0, 7.7), facecolor="#f5f7fb")
    grid = figure.add_gridspec(
        1,
        2,
        left=0.055,
        right=0.965,
        bottom=0.12,
        top=0.75,
        width_ratios=(1.02, 1.18),
        wspace=0.17,
    )
    trajectory_axis = figure.add_subplot(grid[0, 0])
    coverage_axis = figure.add_subplot(grid[0, 1])

    minute_indices = [
        int(np.argmin(np.abs(trajectory_times - second)))
        for second in np.arange(60.0, end_time, 60.0)
    ]
    for drone_id in range(drone_count):
        trajectory = positions[:, drone_id, :]
        color = colors[drone_id]
        trajectory_axis.plot(
            trajectory[:, 0], trajectory[:, 1], color=color, linewidth=1.75
        )
        trajectory_axis.scatter(
            trajectory[minute_indices, 0],
            trajectory[minute_indices, 1],
            s=14,
            color=color,
            edgecolor="white",
            linewidth=0.35,
            zorder=4,
        )
        trajectory_axis.scatter(
            starts[drone_id, 0],
            starts[drone_id, 1],
            marker="*",
            s=95,
            color=color,
            edgecolor="#111827",
            linewidth=0.45,
            zorder=5,
        )
        trajectory_axis.scatter(
            ends[drone_id, 0],
            ends[drone_id, 1],
            marker="X",
            s=52,
            color=color,
            edgecolor="white",
            linewidth=0.6,
            zorder=5,
        )
        trajectory_axis.annotate(
            str(drone_id + 1),
            ends[drone_id, :2],
            xytext=(4, 4),
            textcoords="offset points",
            color=color,
            fontsize=8,
            fontweight="bold",
        )

    if has_bs:
        trajectory_axis.scatter(
            BS_POSITION[0],
            BS_POSITION[1],
            marker="D",
            s=90,
            color="#111827",
            edgecolor="white",
            linewidth=0.8,
            zorder=6,
        )
        trajectory_axis.annotate(
            "BS",
            BS_POSITION[:2],
            xytext=(7, 6),
            textcoords="offset points",
            color="#111827",
            fontweight="bold",
        )
    xmin, xmax, ymin, ymax = WORLD_LIMITS
    trajectory_axis.set(
        xlim=(xmin, xmax),
        ylim=(ymin, ymax),
        xlabel="世界 X / m",
        ylabel="世界 Y / m",
    )
    trajectory_axis.set_aspect("equal", adjustable="box")
    trajectory_axis.set_title(
        "10-UAV 俯视轨迹",
        loc="left",
        y=1.085,
        fontsize=16,
        fontweight="bold",
    )
    trajectory_axis.text(
        0.0,
        1.015,
        "星形：起点　X：终点　圆点：每 60 s 位置",
        transform=trajectory_axis.transAxes,
        color="#60758a",
        fontsize=9.5,
    )
    trajectory_axis.set_facecolor("#eaf0f6")
    trajectory_axis.grid(color="white", linewidth=1.1)

    coverage_times, per_agent_coverage = aligned_coverage(result)
    lower = np.min(per_agent_coverage, axis=0)
    upper = np.max(per_agent_coverage, axis=0)
    joint = np.maximum.accumulate(upper)
    joint[-1] = final_coverage
    coverage_axis.fill_between(
        coverage_times,
        lower,
        upper,
        color="#7db5e8",
        alpha=0.24,
        linewidth=0.0,
        label="各 UAV 覆盖率范围",
    )
    for drone_id in range(drone_count):
        coverage_axis.plot(
            coverage_times,
            per_agent_coverage[drone_id],
            color="#82a6c8",
            linewidth=0.55,
            alpha=0.33,
        )
    coverage_axis.plot(
        coverage_times,
        joint,
        color="#1261a0",
        linewidth=3.0,
        label="联合覆盖率",
        zorder=4,
    )
    coverage_axis.scatter(
        [coverage_times[-1]],
        [joint[-1]],
        s=55,
        color="#1261a0",
        edgecolor="white",
        linewidth=0.7,
        zorder=5,
    )
    coverage_axis.annotate(
        f"{final_coverage:.2f}%",
        (coverage_times[-1], joint[-1]),
        xytext=(-12, 13),
        textcoords="offset points",
        ha="right",
        color="#0b4f86",
        fontsize=11,
        fontweight="bold",
    )
    coverage_axis.axhline(
        final_coverage, color="#1261a0", linewidth=0.8, linestyle="--", alpha=0.35
    )
    coverage_axis.set(
        xlim=(0.0, end_time),
        ylim=(0.0, max(80.0, final_coverage + 5.0)),
        xlabel="仿真时间 / s",
        ylabel="地图覆盖率 / %",
    )
    coverage_axis.set_title(
        "覆盖率随时间变化",
        loc="left",
        y=1.085,
        fontsize=16,
        fontweight="bold",
    )
    coverage_axis.text(
        0.0,
        1.015,
        "异步分 UAV 记录按最近值对齐；联合值取 peer-fused 地图最大覆盖率",
        transform=coverage_axis.transAxes,
        color="#60758a",
        fontsize=9.5,
    )
    coverage_axis.set_facecolor("white")
    coverage_axis.grid(color="#d9e2ec", linewidth=0.8, alpha=0.8)
    coverage_axis.set_axisbelow(True)
    coverage_axis.legend(loc="lower right", frameon=False, fontsize=9.5)

    legend_handles = [
        Line2D(
            [0],
            [0],
            color=colors[index],
            linewidth=2,
            label=f"UAV {index + 1}",
        )
        for index in range(drone_count)
    ]
    trajectory_axis.legend(
        handles=legend_handles,
        loc="upper center",
        bbox_to_anchor=(0.5, -0.12),
        ncol=5,
        frameon=False,
        fontsize=8.5,
        handlelength=1.5,
        columnspacing=1.1,
    )

    algorithm = str(result.get("algorithm", "RACER experiment"))
    is_training_episode = "_episode_" in algorithm
    if is_training_episode:
        episode = algorithm.rsplit("_episode_", 1)[-1]
        title = f"最新完成训练 Episode {episode}：轨迹与覆盖率"
        configuration = "Qwen8B-FP8 + CRPO · ideal perfect-direct"
    else:
        title = "Sionna 纯分布式无 BS 实验：轨迹与覆盖率"
        topology = statistics.get("network_topology", "distributed")
        configuration = f"Sionna · {topology} · BS 关闭"
    executed_count = len(result.get("executed_drone_ids", []))
    figure.suptitle(
        title,
        x=0.055,
        y=0.955,
        ha="left",
        fontsize=20,
        fontweight="bold",
        color="#172b4d",
    )
    figure.text(
        0.055,
        0.885,
        f"{configuration} · "
        f"{end_time:.2f} s · 最终联合覆盖率 {final_coverage:.2f}% · "
        f"总航程 {path_lengths.sum():.1f} m · EXEC_TRAJ {executed_count}/{drone_count} · "
        f"碰撞 {int(metrics['collision_events'])}",
        fontsize=10.8,
        color="#52667a",
    )
    figure.text(
        0.965,
        0.025,
        f"数据源：{result['algorithm']}",
        ha="right",
        fontsize=8.5,
        color="#718096",
    )

    output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(
        output, dpi=220, bbox_inches="tight", facecolor=figure.get_facecolor()
    )
    plt.close(figure)


def main() -> None:
    args = parse_args()
    with args.result.open(encoding="utf-8") as stream:
        result = json.load(stream)
    plot(result, args.output)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
