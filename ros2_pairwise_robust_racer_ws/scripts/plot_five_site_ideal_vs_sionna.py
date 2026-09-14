#!/usr/bin/env python3
"""Plot the matched five-site 10-UAV ideal/Sionna experiment results."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import font_manager
import numpy as np


IDEAL_COLOR = "#2878B5"
SIONNA_COLOR = "#E07B39"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("ideal_result", type=Path)
    parser.add_argument("sionna_result", type=Path)
    parser.add_argument("output", type=Path)
    return parser.parse_args()


def load(path: Path) -> dict:
    return json.loads(path.read_text())


def joint_coverage_curve(result: dict, duration: int = 300) -> tuple[np.ndarray, np.ndarray]:
    history = sorted(
        result["metrics"]["mapping_coverage_history"],
        key=lambda row: (float(row["time_s"]), int(row["drone_id"])),
    )
    times = np.arange(duration + 1, dtype=float)
    latest: dict[int, float] = {}
    curve = np.zeros_like(times)
    event_index = 0
    for index, stamp in enumerate(times):
        while event_index < len(history) and float(history[event_index]["time_s"]) <= stamp:
            event = history[event_index]
            latest[int(event["drone_id"])] = float(event["ratio"])
            event_index += 1
        curve[index] = max(latest.values(), default=0.0)
    curve = np.maximum.accumulate(curve)
    curve[-1] = max(curve[-1], float(result["metrics"]["mapping_coverage_joint"]))
    return times, curve * 100.0


def annotate_bars(axis, bars, fmt="{:.1f}", suffix="") -> None:
    for bar in bars:
        height = bar.get_height()
        axis.annotate(
            fmt.format(height) + suffix,
            (bar.get_x() + bar.get_width() / 2.0, height),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=9,
        )


def main() -> None:
    args = parse_args()
    ideal = load(args.ideal_result)
    sionna = load(args.sionna_result)
    ideal_metrics = ideal["metrics"]
    sionna_metrics = sionna["metrics"]
    ideal_stats = ideal["communication"]["statistics"]
    sionna_stats = sionna["communication"]["statistics"]

    noto_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font_manager.fontManager.addfont(noto_path)
    noto_family = font_manager.FontProperties(fname=noto_path).get_name()
    plt.rcParams.update(
        {
            "font.family": noto_family,
            "axes.unicode_minus": False,
            "axes.titlesize": 13,
            "axes.labelsize": 11,
            "legend.fontsize": 10,
        }
    )
    figure, axes = plt.subplots(2, 2, figsize=(16, 10), constrained_layout=False)
    figure.subplots_adjust(left=0.07, right=0.98, top=0.86, bottom=0.12, hspace=0.34, wspace=0.23)

    ideal_times, ideal_curve = joint_coverage_curve(ideal)
    sionna_times, sionna_curve = joint_coverage_curve(sionna)
    axis = axes[0, 0]
    axis.plot(ideal_times, ideal_curve, color=IDEAL_COLOR, linewidth=2.6, label="完美通信")
    axis.plot(sionna_times, sionna_curve, color=SIONNA_COLOR, linewidth=2.6, label="Sionna（异常）")
    axis.scatter([300, 300], [ideal_curve[-1], sionna_curve[-1]], color=[IDEAL_COLOR, SIONNA_COLOR], zorder=4)
    axis.annotate(f"{ideal_curve[-1]:.2f}%", (300, ideal_curve[-1]), xytext=(-8, 8), textcoords="offset points", ha="right", color=IDEAL_COLOR, weight="bold")
    axis.annotate(f"{sionna_curve[-1]:.2f}%", (300, sionna_curve[-1]), xytext=(-8, 8), textcoords="offset points", ha="right", color=SIONNA_COLOR, weight="bold")
    axis.set_title("A  总体覆盖率随仿真时间变化")
    axis.set_xlabel("仿真时间（s）")
    axis.set_ylabel("总体覆盖率（%）")
    axis.set_xlim(0, 300)
    axis.set_ylim(0, 72)
    axis.grid(True, alpha=0.25)
    axis.legend(loc="upper left")

    uavs = np.arange(1, 11)
    width = 0.38
    axis = axes[0, 1]
    ideal_coverage = np.asarray(ideal_metrics["mapping_coverage_per_agent"]) * 100.0
    sionna_coverage = np.asarray(sionna_metrics["mapping_coverage_per_agent"]) * 100.0
    axis.bar(uavs - width / 2, ideal_coverage, width, color=IDEAL_COLOR, label="完美通信")
    axis.bar(uavs + width / 2, sionna_coverage, width, color=SIONNA_COLOR, label="Sionna（异常）")
    axis.set_title("B  各 UAV 最终地图覆盖率")
    axis.set_xlabel("UAV 编号")
    axis.set_ylabel("覆盖率（%）")
    axis.set_xticks(uavs)
    axis.set_ylim(0, 72)
    axis.grid(axis="y", alpha=0.25)
    axis.legend(loc="upper right")

    axis = axes[1, 0]
    ideal_paths = np.asarray(ideal_metrics["path_lengths"])
    sionna_paths = np.asarray(sionna_metrics["path_lengths"])
    axis.bar(uavs - width / 2, ideal_paths, width, color=IDEAL_COLOR, label="完美通信")
    axis.bar(uavs + width / 2, sionna_paths, width, color=SIONNA_COLOR, label="Sionna（异常）")
    axis.set_title(
        "C  各 UAV 累计航程"
        f"（总计 {ideal_paths.sum():.1f} m vs {sionna_paths.sum():.1f} m）"
    )
    axis.set_xlabel("UAV 编号")
    axis.set_ylabel("累计航程（m）")
    axis.set_xticks(uavs)
    axis.grid(axis="y", alpha=0.25)
    axis.legend(loc="upper left")

    axis = axes[1, 1]
    categories = ["总体覆盖率", "物理包投递率", "执行算法的 UAV"]
    ideal_values = [
        float(ideal_metrics["mapping_coverage_joint"]) * 100.0,
        100.0 * ideal_stats["delivered_packets"] / ideal_stats["attempted_packets"],
        10.0 * len(ideal["executed_drone_ids"]),
    ]
    sionna_values = [
        float(sionna_metrics["mapping_coverage_joint"]) * 100.0,
        100.0 * sionna_stats["delivered_packets"] / sionna_stats["attempted_packets"],
        10.0 * len(sionna["executed_drone_ids"]),
    ]
    positions = np.arange(len(categories))
    ideal_bars = axis.bar(positions - width / 2, ideal_values, width, color=IDEAL_COLOR, label="完美通信")
    sionna_bars = axis.bar(positions + width / 2, sionna_values, width, color=SIONNA_COLOR, label="Sionna（异常）")
    annotate_bars(axis, ideal_bars, suffix="%")
    annotate_bars(axis, sionna_bars, suffix="%")
    axis.set_title("D  结果摘要（数值越高越好）")
    axis.set_ylabel("比例（%）")
    axis.set_xticks(positions, categories)
    axis.set_ylim(0, 112)
    axis.grid(axis="y", alpha=0.25)
    axis.legend(loc="upper center", ncol=2)

    figure.suptitle(
        "Pairwise-Robust RACER：五个起飞区域的 10-UAV 通信对照",
        fontsize=19,
        weight="bold",
        y=0.965,
    )
    figure.text(
        0.5,
        0.915,
        "同一起飞点、300 s、100 Hz 物理、10 Hz 传感器、76,800 rays、seed 42",
        ha="center",
        fontsize=11,
        color="#444444",
    )
    figure.text(
        0.5,
        0.045,
        "注意：Sionna 组记录 14 次进程崩溃，只有 2/10 架 UAV 执行算法；该组属于异常实验，差异不能归因于通信模型本身。",
        ha="center",
        va="center",
        fontsize=11,
        color="#A33A2B",
        weight="bold",
        bbox={"boxstyle": "round,pad=0.5", "facecolor": "#FFF1ED", "edgecolor": "#D98C7D"},
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=200, bbox_inches="tight", facecolor="white")
    figure.savefig(args.output.with_suffix(".svg"), bbox_inches="tight", facecolor="white")
    print(args.output)


if __name__ == "__main__":
    main()
