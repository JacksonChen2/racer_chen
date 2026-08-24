#!/usr/bin/env python3
"""Plot the completed 10-UAV ideal and Sionna distributed experiments."""

import argparse
import json
from pathlib import Path

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "ideal": "#1677ff",
    "sionna": "#f59e0b",
    "grid": "#d9e2ec",
    "text": "#172b4d",
    "muted": "#52667a",
}


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


def joint_coverage_series(result: dict) -> tuple[np.ndarray, np.ndarray]:
    metrics = result["metrics"]
    drone_count = int(result["drone_count"])
    end_time = float(metrics["elapsed"])
    times = np.arange(0.0, end_time + 1.0e-6, 2.0)
    per_agent = []
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
        # Mapping coverage is cumulative; guard the visualization against
        # tiny asynchronous snapshot regressions without altering raw results.
        sample_y = np.maximum.accumulate(sample_y)
        per_agent.append(np.interp(times, sample_t, sample_y))
    return times, np.max(np.vstack(per_agent), axis=0) * 100.0


def packet_outcomes(result: dict) -> dict[str, float]:
    stats = result["communication"]["statistics"]
    attempted = max(1.0, float(stats["attempted_packets"]))
    components = {
        "Delivered": float(stats["delivered_packets"]),
        "No link": float(stats["dropped_no_link"]),
        "PER": float(stats["dropped_per"]),
        "Queue drop": float(stats["dropped_queue"]),
        "TTL drop": float(stats["dropped_ttl"]),
        "Still queued": float(stats["queued_packets"]),
    }
    return {name: value / attempted * 100.0 for name, value in components.items()}


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ideal", type=Path, required=True)
    parser.add_argument("--sionna", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    ideal = load_result(args.ideal)
    sionna = load_result(args.sionna)
    font = choose_font()
    plt.rcParams.update(
        {
            "font.family": font,
            "axes.unicode_minus": False,
            "axes.edgecolor": "#9fb3c8",
            "axes.labelcolor": COLORS["text"],
            "xtick.color": COLORS["muted"],
            "ytick.color": COLORS["muted"],
        }
    )

    fig = plt.figure(figsize=(16, 10), facecolor="#f7f9fc")
    grid = fig.add_gridspec(
        2, 2, left=0.07, right=0.97, bottom=0.11, top=0.78,
        hspace=0.34, wspace=0.22,
    )
    axes = [fig.add_subplot(grid[index]) for index in range(4)]
    for axis in axes:
        axis.set_facecolor("white")
        axis.grid(axis="y", color=COLORS["grid"], linewidth=0.8, alpha=0.8)
        axis.set_axisbelow(True)

    fig.suptitle(
        "10-UAV warehouse_full：无损分布式 vs 纯分布式 Sionna",
        x=0.07, y=0.965, ha="left", fontsize=21, fontweight="bold",
        color=COLORS["text"],
    )
    fig.text(
        0.07,
        0.918,
        "相同 seed=42、1200 s、60 Hz 物理、10 Hz 传感器、19,200 射线",
        fontsize=11.5,
        color=COLORS["muted"],
    )

    cards = (
        (
            "无损分布式",
            COLORS["ideal"],
            ideal["metrics"]["mapping_coverage_joint"] * 100.0,
            len(ideal["finished_drone_ids"]),
            ideal["metrics"]["collision_events"],
        ),
        (
            "纯分布式 Sionna",
            COLORS["sionna"],
            sionna["metrics"]["mapping_coverage_joint"] * 100.0,
            len(sionna["finished_drone_ids"]),
            sionna["metrics"]["collision_events"],
        ),
    )
    for index, (name, color, coverage, finished, collisions) in enumerate(cards):
        x = 0.37 + index * 0.30
        fig.text(x, 0.872, name, fontsize=11, color=color, fontweight="bold")
        fig.text(
            x,
            0.838,
            f"覆盖 {coverage:.1f}%   完成 {finished}/10   碰撞 {collisions}",
            fontsize=12,
            color=COLORS["text"],
        )

    # A: joint map coverage over aligned simulation time.
    ax = axes[0]
    for result, color, label in (
        (ideal, COLORS["ideal"], "无损分布式"),
        (sionna, COLORS["sionna"], "纯分布式 Sionna"),
    ):
        time_s, coverage = joint_coverage_series(result)
        ax.plot(time_s, coverage, color=color, linewidth=2.4, label=label)
        ax.scatter(time_s[-1], coverage[-1], color=color, s=35, zorder=3)
        ax.annotate(
            f"{coverage[-1]:.1f}%",
            (time_s[-1], coverage[-1]),
            xytext=(-8, 8), textcoords="offset points", ha="right",
            fontsize=10, color=color, fontweight="bold",
        )
    ax.set_title("A  联合地图覆盖率（虚拟时间对齐）", loc="left", fontweight="bold")
    ax.set_xlabel("虚拟世界时间 / s")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_xlim(0, 1200)
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, loc="lower right")

    # B: final map coverage per UAV.
    ax = axes[1]
    uavs = np.arange(1, 11)
    width = 0.38
    ideal_coverage = np.asarray(ideal["metrics"]["mapping_coverage_per_agent"]) * 100
    sionna_coverage = np.asarray(sionna["metrics"]["mapping_coverage_per_agent"]) * 100
    ax.bar(uavs - width / 2, ideal_coverage, width, color=COLORS["ideal"], label="无损分布式")
    ax.bar(uavs + width / 2, sionna_coverage, width, color=COLORS["sionna"], label="纯分布式 Sionna")
    ax.set_title("B  各 UAV 最终地图覆盖率", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_xticks(uavs)
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, loc="lower right")

    # C: path length per UAV.
    ax = axes[2]
    ideal_paths = np.asarray(ideal["metrics"]["path_lengths"])
    sionna_paths = np.asarray(sionna["metrics"]["path_lengths"])
    ax.bar(uavs - width / 2, ideal_paths, width, color=COLORS["ideal"], label="无损分布式")
    ax.bar(uavs + width / 2, sionna_paths, width, color=COLORS["sionna"], label="纯分布式 Sionna")
    ax.set_title("C  各 UAV 累计飞行距离", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("路径长度 / m")
    ax.set_xticks(uavs)
    ax.legend(frameon=False, loc="upper right")
    ax.text(
        0.01,
        0.95,
        f"总计：无损 {ideal_paths.sum():.0f} m · Sionna {sionna_paths.sum():.0f} m",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        color=COLORS["muted"],
    )

    # D: attempted packet outcomes, stacked to 100%.
    ax = axes[3]
    names = ("无损分布式", "纯分布式 Sionna")
    outcomes = (packet_outcomes(ideal), packet_outcomes(sionna))
    stack_colors = {
        "Delivered": "#22a06b",
        "No link": "#8b5cf6",
        "PER": "#ef4444",
        "Queue drop": "#f97316",
        "TTL drop": "#64748b",
        "Still queued": "#cbd5e1",
    }
    bottoms = np.zeros(2)
    for category in stack_colors:
        values = np.asarray([item[category] for item in outcomes])
        ax.bar(names, values, bottom=bottoms, color=stack_colors[category], label=category)
        bottoms += values
    ax.set_title("D  尝试发送数据包的最终结果", loc="left", fontweight="bold")
    ax.set_ylabel("占尝试数据包比例 / %")
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, fontsize=8.5, ncol=2, loc="lower left")
    ax.text(
        0.02,
        0.95,
        "无损：队列 0、丢包 0\nSionna：平均端到端延迟 556.7 ms",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        color=COLORS["text"],
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": COLORS["grid"]},
    )

    fig.text(
        0.07,
        0.045,
        "注意：两次实验起点布局不同（无损组为 10 架中心聚集；Sionna 组为 5 架中心 + 5 架分散），"
        "因此这是已完成实验的描述性比较，不能将全部差异单独归因于通信环境。",
        fontsize=10.5,
        color="#9a3412",
    )
    fig.text(
        0.07,
        0.018,
        "数据来源：两组 warehouse_full 结果 JSON；覆盖率曲线按每架 UAV 历史记录插值后取联合最大值。",
        fontsize=9,
        color=COLORS["muted"],
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
