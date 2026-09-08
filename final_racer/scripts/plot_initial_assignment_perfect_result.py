#!/usr/bin/env python3
"""Plot the initial-perfect-assignment Sionna result against the prior pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
from matplotlib.lines import Line2D
import numpy as np


COLORS = {
    "perfect": "#2864A8",
    "plain_sionna": "#C9544D",
    "initial_perfect": "#16876B",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("result_suite", type=Path)
    parser.add_argument("comparison_suite", type=Path)
    return parser.parse_args()


def load_json(path: Path) -> dict:
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def result_path(suite: Path, case: str | None = None) -> Path:
    prefix = suite / case if case else suite
    return prefix / "formal_300s/warehouse_full_distributed_result.json"


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


def plot_trajectories(cases: list[dict], output: Path) -> None:
    drone_colors = plt.get_cmap("tab10")(np.arange(10))
    figure, axes = plt.subplots(
        1,
        3,
        figsize=(20.5, 7.3),
        sharex=True,
        sharey=True,
        constrained_layout=True,
    )
    for axis, case in zip(axes, cases):
        result = case["result"]
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
            active = drone_id + 1 in executed
            axis.plot(
                trajectory[:, 0],
                trajectory[:, 1],
                color=drone_colors[drone_id],
                linewidth=1.65 if active else 0.9,
                alpha=0.9 if active else 0.25,
            )
            axis.scatter(
                starts[drone_id, 0],
                starts[drone_id, 1],
                marker="*",
                s=90,
                color=drone_colors[drone_id],
                edgecolor="#202020",
                linewidth=0.4,
                zorder=4,
            )
            axis.scatter(
                ends[drone_id, 0],
                ends[drone_id, 1],
                marker="X",
                s=42,
                color=drone_colors[drone_id],
                edgecolor="white",
                linewidth=0.5,
                zorder=4,
            )
        coverage = 100.0 * float(metrics["mapping_coverage_joint"])
        axis.set_title(
            f"{case['title']}\n覆盖率 {coverage:.2f}%｜执行 {len(executed)}/10"
            f"｜总航程 {path_lengths.sum():.1f} m",
            fontsize=12.2,
            fontweight="bold",
            color=case["color"],
        )
        axis.set_xlim(-27.0, 6.0)
        axis.set_ylim(0.6, 30.6)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel("世界坐标 X（m）")
        axis.set_facecolor("#F1F4F7")
        axis.grid(color="white", linewidth=1.0)
    axes[0].set_ylabel("世界坐标 Y（m）")
    handles = [
        Line2D(
            [0], [0], color=drone_colors[index], linewidth=2,
            label=f"UAV {index + 1}",
        )
        for index in range(10)
    ]
    figure.legend(
        handles=handles,
        loc="lower center",
        bbox_to_anchor=(0.5, -0.015),
        ncol=10,
        fontsize=8.5,
    )
    figure.suptitle(
        "final_racer｜10 UAV、5 个起飞点、300 秒｜俯视轨迹对比",
        fontsize=17,
        fontweight="bold",
    )
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    figure.savefig(output.with_suffix(".svg"), bbox_inches="tight", facecolor="white")
    plt.close(figure)


def plot_coverage(cases: list[dict], switch_time_s: float, output: Path) -> None:
    figure, axis = plt.subplots(figsize=(13.5, 7.5), constrained_layout=True)
    end_points = []
    for case in cases:
        times, values = coverage_curve(case["result"])
        axis.plot(
            times,
            values,
            color=case["color"],
            linewidth=2.7,
            label=case["curve_label"],
        )
        axis.scatter(times[-1], values[-1], s=58, color=case["color"], zorder=5)
        end_points.append((times[-1], values[-1], case["color"]))

    axis.axvspan(0.0, switch_time_s, color="#F4C95D", alpha=0.18, zorder=0)
    axis.axvline(switch_time_s, color="#9A6B00", linestyle="--", linewidth=1.5)
    axis.annotate(
        f"{switch_time_s:.2f} s：epoch 1 获得 10/10 ACK\n结束初始完美通信，切换到正常 Sionna",
        xy=(switch_time_s, 5.0),
        xytext=(24.0, 14.0),
        arrowprops={"arrowstyle": "->", "color": "#8A650D"},
        color="#735407",
        fontsize=10.5,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "#FFF8DF", "edgecolor": "#D2AE4E"},
    )
    label_offsets = ((7, 7), (7, -13), (7, -2))
    for (x_value, y_value, color), offset in zip(end_points, label_offsets):
        axis.annotate(
            f"{y_value:.2f}%",
            (x_value, y_value),
            xytext=offset,
            textcoords="offset points",
            color=color,
            fontsize=11.5,
            fontweight="bold",
        )
    axis.set_xlim(0.0, 312.0)
    axis.set_ylim(0.0, 84.0)
    axis.set_xlabel("仿真时间（s）", fontsize=11)
    axis.set_ylabel("联合地图覆盖率（%）", fontsize=11)
    axis.set_title(
        "初始完美任务分配消除了启动失败\n"
        "10 UAV、5 个起飞点、300 秒、23 dBm、seed 42",
        fontsize=17,
        fontweight="bold",
    )
    axis.grid(True, alpha=0.23)
    axis.legend(loc="lower right", frameon=True)
    axis.text(
        0.5,
        -0.13,
        "新方案：仅初始任务分配使用完美通信；5.23 秒后所有数据均经过正常 Sionna 分布式通信。",
        transform=axis.transAxes,
        ha="center",
        va="top",
        fontsize=10.5,
        color="#315A50",
    )
    figure.savefig(output, dpi=220, bbox_inches="tight", facecolor="white")
    figure.savefig(output.with_suffix(".svg"), bbox_inches="tight", facecolor="white")
    plt.close(figure)


def main() -> None:
    args = parse_args()
    perfect = load_json(result_path(args.comparison_suite, "perfect"))
    plain_sionna = load_json(result_path(args.comparison_suite, "sionna_distributed"))
    initial_perfect = load_json(result_path(args.result_suite))
    switch_time_s = float(
        initial_perfect["communication"]["statistics"]
        ["initial_assignment_perfect_completed_at_s"]
    )
    cases = [
        {
            "result": perfect,
            "title": "完美通信",
            "curve_label": "完美通信（77.61%，10/10 执行）",
            "color": COLORS["perfect"],
        },
        {
            "result": plain_sionna,
            "title": "普通 Sionna 分布式（23 dBm）",
            "curve_label": "普通 Sionna（23.38%，3/10 执行）",
            "color": COLORS["plain_sionna"],
        },
        {
            "result": initial_perfect,
            "title": "初始完美分配 → Sionna",
            "curve_label": "初始完美分配 → Sionna（75.10%，10/10 执行）",
            "color": COLORS["initial_perfect"],
        },
    ]
    output_dir = args.result_suite / "plots"
    output_dir.mkdir(parents=True, exist_ok=True)
    configure_style()
    trajectory_output = output_dir / "trajectory_three_way_comparison.png"
    coverage_output = output_dir / "coverage_three_way_comparison.png"
    plot_trajectories(cases, trajectory_output)
    plot_coverage(cases, switch_time_s, coverage_output)
    print(trajectory_output.resolve())
    print(coverage_output.resolve())


if __name__ == "__main__":
    main()
