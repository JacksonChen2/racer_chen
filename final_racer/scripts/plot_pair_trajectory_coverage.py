#!/usr/bin/env python3
"""Plot trajectories and coverage for a final_racer perfect/Sionna pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import numpy as np


PERFECT_COLOR = "#2468B4"
SIONNA_COLOR = "#E07A2F"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("suite", type=Path)
    return parser.parse_args()


def load_result(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def configure_style() -> None:
    noto_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")
    if noto_path.is_file():
        font_manager.fontManager.addfont(str(noto_path))
        family = font_manager.FontProperties(fname=str(noto_path)).get_name()
    else:
        family = "DejaVu Sans"
    plt.rcParams.update(
        {
            "font.family": family,
            "axes.unicode_minus": False,
            "axes.edgecolor": "#8CA0B3",
            "axes.labelcolor": "#243447",
            "xtick.color": "#526477",
            "ytick.color": "#526477",
        }
    )


def coverage_curve(result: dict, step_s: float = 0.5) -> tuple[np.ndarray, np.ndarray]:
    metrics = result["metrics"]
    elapsed = float(metrics["elapsed"])
    union_history = metrics.get("mapping_coverage_joint_history", [])
    if union_history:
        records = sorted(
            (
                (float(row["time_s"]), float(row["ratio"]))
                for row in union_history
            ),
            key=lambda row: row[0],
        )
        record_times = np.asarray(
            [0.0] + [row[0] for row in records] + [elapsed], dtype=float
        )
        record_ratios = np.asarray(
            [0.0]
            + [row[1] for row in records]
            + [float(metrics["mapping_coverage_joint"])],
            dtype=float,
        )
        times = np.arange(0.0, elapsed + step_s, step_s)
        times[-1] = elapsed
        joint = np.interp(
            times, record_times, np.maximum.accumulate(record_ratios)
        )
        return times, 100.0 * np.maximum.accumulate(joint)

    # Compatibility for results produced before the true union bitmap metric.
    drone_count = int(result["drone_count"])
    times = np.arange(0.0, elapsed + step_s, step_s)
    times[-1] = elapsed
    curves = []
    for drone_id in range(drone_count):
        records = sorted(
            (
                (float(row["time_s"]), float(row["ratio"]))
                for row in metrics["mapping_coverage_history"]
                if int(row["drone_id"]) == drone_id
            ),
            key=lambda row: row[0],
        )
        record_times = np.asarray(
            [0.0] + [row[0] for row in records] + [elapsed], dtype=float
        )
        record_ratios = np.asarray(
            [0.0]
            + [row[1] for row in records]
            + [float(metrics["mapping_coverage_per_agent"][drone_id])],
            dtype=float,
        )
        curves.append(
            np.interp(times, record_times, np.maximum.accumulate(record_ratios))
        )
    joint = np.max(np.vstack(curves), axis=0)
    joint[-1] = float(metrics["mapping_coverage_joint"])
    return times, 100.0 * np.maximum.accumulate(joint)


def plot_trajectories(perfect: dict, sionna: dict, output: Path) -> None:
    colors = plt.get_cmap("tab10")(np.arange(10))
    figure, axes = plt.subplots(
        1, 2, figsize=(15.5, 7.5), sharex=True, sharey=True,
        constrained_layout=True,
    )
    cases = (
        (axes[0], perfect, "完美通信"),
        (axes[1], sionna, "Sionna 纯分布式通信（23 dBm）"),
    )
    for axis, result, label in cases:
        metrics = result["metrics"]
        history = np.asarray(
            [sample["positions"] for sample in metrics["trajectory_history"]],
            dtype=float,
        )
        starts = np.asarray(metrics["start_positions"], dtype=float)
        ends = np.asarray(metrics["positions"], dtype=float)
        path_lengths = np.asarray(metrics["path_lengths"], dtype=float)
        executed = set(int(value) for value in result["executed_drone_ids"])
        for drone_id in range(10):
            trajectory = history[:, drone_id, :]
            is_executed = drone_id + 1 in executed
            axis.plot(
                trajectory[:, 0], trajectory[:, 1],
                color=colors[drone_id], linewidth=1.8 if is_executed else 1.0,
                alpha=0.92 if is_executed else 0.38,
            )
            axis.scatter(
                starts[drone_id, 0], starts[drone_id, 1], marker="*", s=105,
                color=colors[drone_id], edgecolor="#202020", linewidth=0.45,
                zorder=4,
            )
            axis.scatter(
                ends[drone_id, 0], ends[drone_id, 1], marker="X", s=48,
                color=colors[drone_id], edgecolor="white", linewidth=0.55,
                zorder=4,
            )
        coverage = 100.0 * float(metrics["mapping_coverage_joint"])
        status = (
            f"{len(executed)}/10 UAV 执行｜碰撞 {int(metrics['collision_events'])}"
        )
        axis.set_title(
            f"{label}\n覆盖率 {coverage:.2f}%｜总航程 {path_lengths.sum():.1f} m\n{status}",
            fontsize=13, fontweight="bold",
            color="#243447",
        )
        axis.set_xlim(-27.0, 6.0)
        axis.set_ylim(0.6, 30.6)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("世界坐标 X（m）")
        axis.set_facecolor("#F1F4F7")
        axis.grid(color="white", linewidth=1.0)
    axes[0].set_ylabel("世界坐标 Y（m）")
    handles = [
        Line2D([0], [0], color=colors[index], linewidth=2, label=f"UAV {index + 1}")
        for index in range(10)
    ]
    handles.extend(
        [
            Line2D([0], [0], marker="*", color="none", markerfacecolor="#777777",
                   markeredgecolor="#202020", markersize=10, label="起点"),
            Line2D([0], [0], marker="X", color="none", markerfacecolor="#777777",
                   markeredgecolor="white", markersize=8, label="终点"),
        ]
    )
    figure.legend(
        handles=handles, loc="lower center", bbox_to_anchor=(0.5, -0.075),
        ncol=12, fontsize=8.5,
    )
    figure.suptitle(
        "final_racer｜10 UAV、5 个起飞点、300 秒｜俯视轨迹对比",
        fontsize=17, fontweight="bold",
    )
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    figure.savefig(output.with_suffix(".svg"), bbox_inches="tight", facecolor="white")
    plt.close(figure)


def plot_coverage(perfect: dict, sionna: dict, output: Path) -> None:
    perfect_t, perfect_y = coverage_curve(perfect)
    sionna_t, sionna_y = coverage_curve(sionna)
    figure, axis = plt.subplots(figsize=(12.5, 7.2), constrained_layout=True)
    axis.plot(
        perfect_t, perfect_y, color=PERFECT_COLOR, linewidth=2.8,
        label="完美通信",
    )
    axis.plot(
        sionna_t, sionna_y, color=SIONNA_COLOR, linewidth=2.8,
        label="Sionna 纯分布式 23 dBm",
    )
    for times, values, color, offset in (
        (perfect_t, perfect_y, PERFECT_COLOR, (8, -5)),
        (sionna_t, sionna_y, SIONNA_COLOR, (8, 8)),
    ):
        axis.scatter(times[-1], values[-1], s=60, color=color, zorder=4)
        axis.annotate(
            f"{values[-1]:.2f}%", (times[-1], values[-1]),
            xytext=offset, textcoords="offset points", color=color,
            fontsize=12, fontweight="bold",
        )
    axis.set_xlim(0.0, 306.0)
    axis.set_ylim(0.0, max(82.0, perfect_y[-1] + 5.0))
    axis.set_xlabel("仿真时间（s）", fontsize=11)
    axis.set_ylabel("所有 UAV 已知地图并集覆盖率（%）", fontsize=11)
    axis.set_title(
        "final_racer｜所有 UAV 已知地图并集覆盖率随时间变化\n"
        "10 UAV、5 个起飞点、300 秒、seed 42",
        fontsize=17, fontweight="bold",
    )
    axis.grid(True, alpha=0.24)
    axis.legend(loc="upper left", frameon=True)
    axis.text(
        0.5, -0.14,
        "定义：至少被一架 UAV 标记为 FREE 或 OCCUPIED 的规划区体素数 / 规划区总数；"
        "同一体素在多架 UAV 地图中只计一次。",
        transform=axis.transAxes, ha="center", va="top", fontsize=10.5,
        color="#46596B",
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#F2F6FA", "edgecolor": "#A9BAC9"},
    )
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    figure.savefig(output.with_suffix(".svg"), bbox_inches="tight", facecolor="white")
    plt.close(figure)


def main() -> None:
    args = parse_args()
    perfect_path = args.suite / "perfect/formal_300s/warehouse_full_distributed_result.json"
    sionna_path = args.suite / "sionna_distributed/formal_300s/warehouse_full_distributed_result.json"
    perfect = load_result(perfect_path)
    sionna = load_result(sionna_path)
    output_dir = args.suite / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    trajectory_output = output_dir / "trajectory_comparison.png"
    coverage_output = output_dir / "coverage_over_time.png"
    plot_trajectories(perfect, sionna, trajectory_output)
    plot_coverage(perfect, sionna, coverage_output)
    print(trajectory_output.resolve())
    print(coverage_output.resolve())


if __name__ == "__main__":
    main()
