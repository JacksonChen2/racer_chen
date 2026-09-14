#!/usr/bin/env python3
"""Plot the matched 5-UAV central-start ideal and Sionna experiments."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "ideal": "#2878B5",
    "sionna": "#E07B39",
    "green": "#22A06B",
    "red": "#D64545",
    "purple": "#7C5CFC",
    "gray": "#64748B",
    "grid": "#D9E2EC",
    "text": "#172B4D",
    "muted": "#52667A",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ideal", type=Path, required=True)
    parser.add_argument("--sionna", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
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
    installed = {font.name for font in font_manager.fontManager.ttflist}
    return next((name for name in candidates if name in installed), "DejaVu Sans")


def joint_coverage_series(result: dict) -> tuple[np.ndarray, np.ndarray]:
    metrics = result["metrics"]
    drone_count = int(result["drone_count"])
    end_time = float(metrics["elapsed"])
    times = np.unique(np.append(np.arange(0.0, end_time, 2.0), end_time))
    per_agent: list[np.ndarray] = []
    history = metrics["mapping_coverage_history"]
    final_ratios = metrics["mapping_coverage_per_agent"]

    for drone_id in range(drone_count):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in history
                if int(item["drone_id"]) == drone_id
            ),
            key=lambda item: item[0],
        )
        sample_t = np.asarray([0.0] + [item[0] for item in records] + [end_time])
        sample_y = np.asarray(
            [0.0] + [item[1] for item in records] + [final_ratios[drone_id]]
        )
        sample_y = np.maximum.accumulate(sample_y)
        per_agent.append(np.interp(times, sample_t, sample_y))

    return times, np.max(np.vstack(per_agent), axis=0) * 100.0


def packet_outcomes(result: dict) -> dict[str, float]:
    stats = result["communication"]["statistics"]
    attempted = max(1.0, float(stats["attempted_packets"]))
    counts = {
        "成功交付": float(stats["delivered_packets"]),
        "PER 丢弃": float(stats["dropped_per"]),
        "无链路": float(stats["dropped_no_link"]),
        "TTL 丢弃": float(stats["dropped_ttl"]),
        "队列丢弃/排队": float(stats["dropped_queue"] + stats["queued_packets"]),
    }
    return {name: 100.0 * value / attempted for name, value in counts.items()}


def annotate_bars(axis: plt.Axes, bars, suffix: str = "") -> None:
    for bar in bars:
        height = float(bar.get_height())
        axis.annotate(
            f"{height:.1f}{suffix}",
            (bar.get_x() + bar.get_width() / 2.0, height),
            xytext=(0, 4),
            textcoords="offset points",
            ha="center",
            va="bottom",
            fontsize=8.5,
        )


def main() -> None:
    args = parse_args()
    ideal = load_result(args.ideal)
    sionna = load_result(args.sionna)
    drone_count = int(ideal["drone_count"])
    if drone_count != 5 or int(sionna["drone_count"]) != drone_count:
        raise ValueError("both results must contain the matched five-UAV experiment")

    ideal_metrics = ideal["metrics"]
    sionna_metrics = sionna["metrics"]
    ideal_stats = ideal["communication"]["statistics"]
    sionna_stats = sionna["communication"]["statistics"]
    ideal_coverage = 100.0 * float(ideal_metrics["mapping_coverage_joint"])
    sionna_coverage = 100.0 * float(sionna_metrics["mapping_coverage_joint"])
    ideal_paths = np.asarray(ideal_metrics["path_lengths"], dtype=float)
    sionna_paths = np.asarray(sionna_metrics["path_lengths"], dtype=float)
    duration = max(float(ideal_metrics["elapsed"]), float(sionna_metrics["elapsed"]))

    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "axes.edgecolor": "#9FB3C8",
            "axes.labelcolor": COLORS["text"],
            "xtick.color": COLORS["muted"],
            "ytick.color": COLORS["muted"],
        }
    )

    figure = plt.figure(figsize=(16, 10), facecolor="#F7F9FC")
    grid = figure.add_gridspec(
        2, 2, left=0.07, right=0.97, bottom=0.12, top=0.78,
        hspace=0.36, wspace=0.23,
    )
    axes = [figure.add_subplot(grid[index]) for index in range(4)]
    for axis in axes:
        axis.set_facecolor("white")
        axis.grid(axis="y", color=COLORS["grid"], linewidth=0.8, alpha=0.8)
        axis.set_axisbelow(True)

    figure.suptitle(
        "5 架 UAV 中央起飞：完美通信 vs Sionna",
        x=0.07,
        y=0.965,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["text"],
    )
    figure.text(
        0.07,
        0.918,
        "warehouse_full_with_industrial_ap · 相同中央起点 · 500 s · "
        "100 Hz 物理 · 10 Hz 传感器 · 76,800 rays · seed 42",
        fontsize=11.2,
        color=COLORS["muted"],
    )

    cards = (
        (
            "完美通信",
            COLORS["ideal"],
            ideal_coverage,
            ideal_paths.sum(),
            len(ideal["executed_drone_ids"]),
        ),
        (
            "Sionna 通信",
            COLORS["sionna"],
            sionna_coverage,
            sionna_paths.sum(),
            len(sionna["executed_drone_ids"]),
        ),
    )
    for index, (name, color, coverage, path, executed) in enumerate(cards):
        x = 0.37 + index * 0.30
        figure.text(x, 0.872, name, fontsize=11, color=color, fontweight="bold")
        figure.text(
            x,
            0.838,
            f"覆盖 {coverage:.2f}%   航程 {path:.1f} m   执行 {executed}/5",
            fontsize=11.5,
            color=COLORS["text"],
        )

    # A: aligned joint map coverage history.
    axis = axes[0]
    for result, color, label, label_offset in (
        (ideal, COLORS["ideal"], "完美通信", (-8, -16)),
        (sionna, COLORS["sionna"], "Sionna 通信", (-8, 8)),
    ):
        time_s, coverage = joint_coverage_series(result)
        axis.plot(time_s, coverage, color=color, linewidth=2.5, label=label)
        axis.scatter(time_s[-1], coverage[-1], color=color, s=38, zorder=3)
        axis.annotate(
            f"{coverage[-1]:.2f}%",
            (time_s[-1], coverage[-1]),
            xytext=label_offset,
            textcoords="offset points",
            ha="right",
            color=color,
            fontsize=9.5,
            fontweight="bold",
        )
    axis.set_title("A  联合地图覆盖率随仿真时间变化", loc="left", fontweight="bold")
    axis.set_xlabel("仿真时间 / s")
    axis.set_ylabel("已知体素覆盖率 / %")
    axis.set_xlim(0, duration)
    axis.set_ylim(0, 82)
    axis.legend(frameon=False, loc="lower right")

    # B: final per-UAV map coverage.
    axis = axes[1]
    uavs = np.arange(1, drone_count + 1)
    width = 0.36
    ideal_per_uav = 100.0 * np.asarray(ideal_metrics["mapping_coverage_per_agent"])
    sionna_per_uav = 100.0 * np.asarray(sionna_metrics["mapping_coverage_per_agent"])
    bars_ideal = axis.bar(
        uavs - width / 2, ideal_per_uav, width,
        color=COLORS["ideal"], label="完美通信",
    )
    bars_sionna = axis.bar(
        uavs + width / 2, sionna_per_uav, width,
        color=COLORS["sionna"], label="Sionna 通信",
    )
    annotate_bars(axis, bars_ideal, "%")
    annotate_bars(axis, bars_sionna, "%")
    axis.set_title("B  各 UAV 最终地图覆盖率", loc="left", fontweight="bold")
    axis.set_xlabel("UAV 编号")
    axis.set_ylabel("已知体素覆盖率 / %")
    axis.set_xticks(uavs)
    axis.set_ylim(0, 84)
    axis.legend(frameon=False, loc="lower right")

    # C: path length per UAV.
    axis = axes[2]
    bars_ideal = axis.bar(
        uavs - width / 2, ideal_paths, width,
        color=COLORS["ideal"], label="完美通信",
    )
    bars_sionna = axis.bar(
        uavs + width / 2, sionna_paths, width,
        color=COLORS["sionna"], label="Sionna 通信",
    )
    annotate_bars(axis, bars_ideal, " m")
    annotate_bars(axis, bars_sionna, " m")
    axis.set_title("C  各 UAV 累计飞行距离", loc="left", fontweight="bold")
    axis.set_xlabel("UAV 编号")
    axis.set_ylabel("路径长度 / m")
    axis.set_xticks(uavs)
    axis.set_ylim(0, max(ideal_paths.max(), sionna_paths.max()) * 1.18)
    axis.legend(frameon=False, loc="upper right")

    # D: packet outcomes normalized to attempted receiver packets.
    axis = axes[3]
    names = ("完美通信", "Sionna 通信")
    outcomes = (packet_outcomes(ideal), packet_outcomes(sionna))
    stack_colors = {
        "成功交付": COLORS["green"],
        "PER 丢弃": COLORS["red"],
        "无链路": COLORS["purple"],
        "TTL 丢弃": COLORS["gray"],
        "队列丢弃/排队": "#CBD5E1",
    }
    bottoms = np.zeros(2)
    for category, color in stack_colors.items():
        values = np.asarray([outcome[category] for outcome in outcomes])
        axis.bar(names, values, bottom=bottoms, color=color, label=category)
        bottoms += values
    ideal_delivery = 100.0 * ideal_stats["delivered_packets"] / ideal_stats["attempted_packets"]
    sionna_delivery = 100.0 * sionna_stats["delivered_packets"] / sionna_stats["attempted_packets"]
    axis.set_title("D  接收端数据包结果", loc="left", fontweight="bold")
    axis.set_ylabel("占尝试数据包比例 / %")
    axis.set_ylim(0, 100)
    axis.legend(frameon=False, fontsize=8.5, ncol=2, loc="lower left")
    axis.text(
        0.02,
        0.95,
        f"完美通信交付率 {ideal_delivery:.1f}%\n"
        f"Sionna 交付率 {sionna_delivery:.1f}% · 平均延迟 "
        f"{float(sionna_stats['mean_delivery_delay_ms']):.1f} ms",
        transform=axis.transAxes,
        va="top",
        fontsize=9.8,
        color=COLORS["text"],
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": COLORS["grid"]},
    )

    coverage_gap = sionna_coverage - ideal_coverage
    path_change = 100.0 * (sionna_paths.sum() / ideal_paths.sum() - 1.0)
    figure.text(
        0.07,
        0.052,
        f"500 s 终态：Sionna 覆盖率比完美通信高 {coverage_gap:.2f} 个百分点，"
        f"同时总航程增加 {path_change:.1f}%；两组均为 5/5 执行、零碰撞。",
        fontsize=10.5,
        color="#9A3412",
    )
    figure.text(
        0.07,
        0.022,
        "两组均由时限终止，未完成全员探索和返航；单次 seed=42 的覆盖差异不应单独解释为通信性能优劣。",
        fontsize=9.3,
        color=COLORS["muted"],
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=190, bbox_inches="tight", facecolor=figure.get_facecolor())
    figure.savefig(args.output.with_suffix(".pdf"), bbox_inches="tight", facecolor=figure.get_facecolor())
    plt.close(figure)
    print(args.output)


if __name__ == "__main__":
    main()
