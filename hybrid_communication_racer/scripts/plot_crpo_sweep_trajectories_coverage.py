#!/usr/bin/env python3
"""Plot the best CRPO-sweep trajectory and joint coverage comparisons."""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.lines import Line2D


@dataclass(frozen=True)
class RunSpec:
    relative_path: str
    label: str
    color: str
    linestyle: str = "-"
    linewidth: float = 2.0
    zorder: int = 2


TRIALS = (
    RunSpec(
        "trial_1_lr2e4_b128_s512_relu/sim/warehouse_full_bs_round_robin_result.json",
        "组 1 · ReLU · n_steps=512 · batch=128",
        "#f59e0b",
    ),
    RunSpec(
        "trial_2_lr1p5e4_b64_s256_elu/sim/warehouse_full_bs_round_robin_result.json",
        "组 2 · ELU · n_steps=256 · batch=64",
        "#8b5cf6",
    ),
    RunSpec(
        "trial_3_lr2e4_b64_s384_tanh/sim/warehouse_full_bs_round_robin_result.json",
        "组 3 · Tanh · n_steps=384 · batch=64（最佳）",
        "#16a34a",
        linewidth=3.2,
        zorder=5,
    ),
    RunSpec(
        "trial_4_lr1p5e4_b32_s256_tanh/sim/warehouse_full_bs_round_robin_result.json",
        "组 4 · Tanh · n_steps=256 · batch=32",
        "#dc2626",
    ),
)

BEST_TRIAL_INDEX = 2
BS_POSITION = np.asarray([-10.028926, 14.888599, 7.55], dtype=float)
WORLD_LIMITS = (-27.0, 6.0, 0.6, 30.6, 0.4, 8.4)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sweep-dir", type=Path, required=True)
    parser.add_argument("--baseline", type=Path, required=True)
    parser.add_argument("--ideal", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    return parser.parse_args()


def load_result(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        return json.load(stream)


def choose_font() -> str:
    candidates = (
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans CN",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    )
    installed = {item.name for item in font_manager.fontManager.ttflist}
    return next((font for font in candidates if font in installed), "DejaVu Sans")


def joint_coverage_series(
    result: dict, interval_s: float = 2.0
) -> tuple[np.ndarray, np.ndarray]:
    """Align asynchronous per-UAV snapshots and form the peer-fused map curve."""
    metrics = result["metrics"]
    drone_count = int(result["drone_count"])
    end_time = float(metrics["elapsed"])
    times = np.arange(0.0, end_time + 0.5 * interval_s, interval_s)
    times = times[times <= end_time]
    if not len(times) or times[-1] < end_time:
        times = np.append(times, end_time)

    history = metrics["mapping_coverage_history"]
    final_ratios = metrics["mapping_coverage_per_agent"]
    per_agent = []
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
        # Coverage is cumulative. Coalesce duplicate timestamps and suppress
        # minor asynchronous-snapshot regressions only for visualization.
        unique_t, inverse = np.unique(sample_t, return_inverse=True)
        unique_y = np.zeros_like(unique_t)
        np.maximum.at(unique_y, inverse, sample_y)
        unique_y = np.maximum.accumulate(unique_y)
        per_agent.append(np.interp(times, unique_t, unique_y))

    joint = np.max(np.vstack(per_agent), axis=0)
    joint[-1] = float(metrics["mapping_coverage_joint"])
    return times, 100.0 * np.maximum.accumulate(joint)


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


def plot_trajectories(result: dict, output: Path) -> None:
    metrics = result["metrics"]
    history = metrics["trajectory_history"]
    positions = np.asarray([item["positions"] for item in history], dtype=float)
    sample_times = np.asarray([item["time_s"] for item in history], dtype=float)
    starts = np.asarray(metrics["start_positions"], dtype=float)
    ends = np.asarray(metrics["positions"], dtype=float)
    path_lengths = np.asarray(metrics["path_lengths"], dtype=float)
    drone_count = int(result["drone_count"])
    colors = plt.get_cmap("tab10")(np.arange(drone_count))
    xmin, xmax, ymin, ymax, zmin, zmax = WORLD_LIMITS

    fig = plt.figure(figsize=(16, 7.5), facecolor="#f7f9fc")
    grid = fig.add_gridspec(
        1, 2, left=0.055, right=0.91, bottom=0.10, top=0.84, wspace=0.15
    )
    ax_top = fig.add_subplot(grid[0, 0])
    ax_3d = fig.add_subplot(grid[0, 1], projection="3d")

    minute_indices = [
        int(np.argmin(np.abs(sample_times - second)))
        for second in (60.0, 120.0, 180.0, 240.0)
    ]
    for drone_id in range(drone_count):
        trajectory = positions[:, drone_id, :]
        color = colors[drone_id]
        ax_top.plot(trajectory[:, 0], trajectory[:, 1], color=color, lw=1.8)
        ax_top.scatter(
            trajectory[minute_indices, 0],
            trajectory[minute_indices, 1],
            color=color,
            s=16,
            edgecolor="white",
            linewidth=0.35,
            zorder=4,
        )
        ax_top.scatter(
            starts[drone_id, 0],
            starts[drone_id, 1],
            marker="*",
            s=115,
            color=color,
            edgecolor="black",
            linewidth=0.45,
            zorder=5,
        )
        ax_top.scatter(
            ends[drone_id, 0],
            ends[drone_id, 1],
            marker="X",
            s=60,
            color=color,
            edgecolor="white",
            linewidth=0.6,
            zorder=5,
        )
        ax_top.annotate(
            str(drone_id + 1),
            ends[drone_id, :2],
            xytext=(4, 4),
            textcoords="offset points",
            fontsize=8,
            color=color,
            fontweight="bold",
        )

        ax_3d.plot(
            trajectory[:, 0],
            trajectory[:, 1],
            trajectory[:, 2],
            color=color,
            lw=1.6,
        )
        ax_3d.scatter(*starts[drone_id], marker="*", s=65, color=color)
        ax_3d.scatter(*ends[drone_id], marker="X", s=38, color=color)

    ax_top.scatter(
        BS_POSITION[0],
        BS_POSITION[1],
        marker="D",
        s=100,
        color="#111827",
        edgecolor="white",
        linewidth=0.8,
        zorder=6,
        label="基站",
    )
    ax_top.annotate(
        "BS", BS_POSITION[:2], xytext=(7, 6), textcoords="offset points",
        color="#111827", fontweight="bold"
    )
    ax_top.set(xlim=(xmin, xmax), ylim=(ymin, ymax), xlabel="世界 X / m", ylabel="世界 Y / m")
    ax_top.set_aspect("equal", adjustable="box")
    ax_top.set_title("俯视轨迹（圆点为每 60 s 位置）", loc="left", fontweight="bold")
    ax_top.set_facecolor("#eef2f6")
    ax_top.grid(color="white", linewidth=1.1)

    ax_3d.scatter(*BS_POSITION, marker="D", s=75, color="#111827")
    ax_3d.set(
        xlim=(xmin, xmax), ylim=(ymin, ymax), zlim=(zmin, zmax),
        xlabel="X / m", ylabel="Y / m", zlabel="Z / m"
    )
    ax_3d.set_title("三维飞行轨迹", loc="left", fontweight="bold")
    ax_3d.view_init(elev=29, azim=-61)
    ax_3d.set_box_aspect((xmax - xmin, ymax - ymin, 14.0))

    legend_handles = [
        Line2D(
            [0], [0], color=colors[index], lw=2,
            label=f"UAV {index + 1} · {path_lengths[index]:.1f} m"
        )
        for index in range(drone_count)
    ]
    legend_handles.extend(
        [
            Line2D([0], [0], marker="*", color="none", markerfacecolor="#6b7280",
                   markeredgecolor="black", markersize=10, label="起点"),
            Line2D([0], [0], marker="X", color="none", markerfacecolor="#6b7280",
                   markeredgecolor="white", markersize=8, label="终点"),
            Line2D([0], [0], marker="D", color="none", markerfacecolor="#111827",
                   markersize=8, label="基站"),
        ]
    )
    fig.legend(
        handles=legend_handles, loc="center right", bbox_to_anchor=(0.995, 0.48),
        frameon=False, fontsize=9
    )
    final_coverage = 100.0 * float(metrics["mapping_coverage_joint"])
    fig.suptitle(
        "最佳超参数组（组 3）的 10-UAV 轨迹",
        x=0.055, y=0.965, ha="left", fontsize=19, fontweight="bold", color="#172b4d"
    )
    fig.text(
        0.055, 0.905,
        f"Tanh · learning_rate=2e-4 · n_steps=384 · batch_size=64 · "
        f"300 s · 联合覆盖率 {final_coverage:.2f}% · 总航程 {path_lengths.sum():.1f} m",
        fontsize=11, color="#52667a"
    )
    fig.text(
        0.055, 0.025,
        "轨迹由保存的位置历史重建；图中未叠加仓库障碍物几何。",
        fontsize=9, color="#64748b"
    )
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=220, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def plot_coverage(
    runs: list[tuple[RunSpec, dict]], output: Path
) -> None:
    fig, ax = plt.subplots(figsize=(13.5, 7.6), facecolor="#f7f9fc")
    ax.set_facecolor("white")
    for spec, result in runs:
        time_s, coverage = joint_coverage_series(result)
        final = 100.0 * float(result["metrics"]["mapping_coverage_joint"])
        ax.plot(
            time_s,
            coverage,
            color=spec.color,
            linestyle=spec.linestyle,
            linewidth=spec.linewidth,
            zorder=spec.zorder,
            label=f"{spec.label} · {final:.2f}%",
        )
        ax.scatter(
            [time_s[-1]], [coverage[-1]], color=spec.color, s=36,
            edgecolor="white", linewidth=0.6, zorder=spec.zorder + 1
        )

    ax.set_xlim(0.0, 306.0)
    ax.set_ylim(0.0, 80.0)
    ax.set_xlabel("仿真时间 / s", fontsize=11)
    ax.set_ylabel("联合地图覆盖率 / %", fontsize=11)
    ax.set_title(
        "联合覆盖率随时间变化",
        loc="left", fontsize=18, fontweight="bold", color="#172b4d", pad=18
    )
    ax.text(
        0.0, 1.015,
        "异步记录插值到统一 2 s 时间轴；联合覆盖率取各 UAV 已共享地图的最大覆盖率",
        transform=ax.transAxes, fontsize=10.5, color="#52667a"
    )
    ax.grid(color="#d9e2ec", linewidth=0.8, alpha=0.85)
    ax.set_axisbelow(True)
    ax.legend(loc="upper left", frameon=False, fontsize=9.5, ncol=2)
    fig.tight_layout(pad=2.0)
    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=220, bbox_inches="tight", facecolor=fig.get_facecolor())
    plt.close(fig)


def main() -> None:
    args = parse_args()
    configure_style()

    trial_results = [
        (spec, load_result(args.sweep_dir / spec.relative_path)) for spec in TRIALS
    ]
    best_result = trial_results[BEST_TRIAL_INDEX][1]
    baseline = load_result(args.baseline)
    ideal = load_result(args.ideal)

    trajectory_output = args.output_dir / "best_trial_3_uav_trajectories.png"
    coverage_output = args.output_dir / "coverage_vs_time_all_runs.png"
    plot_trajectories(best_result, trajectory_output)

    coverage_runs = [
        (
            RunSpec("", "完美通信", "#1677ff", linestyle="--", linewidth=3.0, zorder=4),
            ideal,
        ),
        (
            RunSpec("", "原 BS-assisted 基线", "#64748b", linestyle=":", linewidth=2.8, zorder=3),
            baseline,
        ),
        *trial_results,
    ]
    plot_coverage(coverage_runs, coverage_output)
    print(trajectory_output.resolve())
    print(coverage_output.resolve())


if __name__ == "__main__":
    main()
