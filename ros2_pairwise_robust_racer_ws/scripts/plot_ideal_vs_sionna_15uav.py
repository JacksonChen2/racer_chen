#!/usr/bin/env python3
"""Plot the completed aligned 15-UAV ideal and Sionna experiments."""

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
    "green": "#22a06b",
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
    times = np.unique(
        np.append(np.arange(0.0, end_time + 1.0e-6, 2.0), end_time)
    )
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
        sample_y = np.maximum.accumulate(sample_y)
        per_agent.append(np.interp(times, sample_t, sample_y))
    return times, np.max(np.vstack(per_agent), axis=0) * 100.0


def packet_outcomes(result: dict) -> dict[str, float]:
    stats = result["communication"]["statistics"]
    attempted = max(1.0, float(stats["attempted_packets"]))
    components = {
        "成功送达": float(stats["delivered_packets"]),
        "无链路": float(stats["dropped_no_link"]),
        "PER丢弃": float(stats["dropped_per"]),
        "队列丢弃": float(stats["dropped_queue"]),
        "TTL超时": float(stats["dropped_ttl"]),
        "仍在排队": float(stats["queued_packets"]),
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
    drone_count = int(ideal["drone_count"])
    if int(sionna["drone_count"]) != drone_count:
        raise ValueError("the two results use different UAV counts")

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
        2,
        2,
        left=0.07,
        right=0.97,
        bottom=0.105,
        top=0.78,
        hspace=0.35,
        wspace=0.22,
    )
    axes = [fig.add_subplot(grid[index]) for index in range(4)]
    for axis in axes:
        axis.set_facecolor("white")
        axis.grid(axis="y", color=COLORS["grid"], linewidth=0.8, alpha=0.8)
        axis.set_axisbelow(True)

    ideal_coverage = float(ideal["metrics"]["mapping_coverage_joint"]) * 100.0
    sionna_coverage = float(sionna["metrics"]["mapping_coverage_joint"]) * 100.0
    coverage_gap = ideal_coverage - sionna_coverage

    fig.suptitle(
        "15架 UAV 性能对比：完美通信 vs Sionna纯分布式",
        x=0.07,
        y=0.965,
        ha="left",
        fontsize=21,
        fontweight="bold",
        color=COLORS["text"],
    )
    fig.text(
        0.07,
        0.918,
        "相同 seed=42、相同两批式起点、1200 s、50 Hz物理、10 Hz深度相机、19,200射线",
        fontsize=11.5,
        color=COLORS["muted"],
    )

    cards = (
        (
            "完美通信",
            COLORS["ideal"],
            ideal_coverage,
            len(ideal["finished_drone_ids"]),
            ideal["metrics"]["collision_events"],
        ),
        (
            "Sionna纯分布式",
            COLORS["sionna"],
            sionna_coverage,
            len(sionna["finished_drone_ids"]),
            sionna["metrics"]["collision_events"],
        ),
    )
    for index, (name, color, coverage, finished, collisions) in enumerate(cards):
        x = 0.36 + index * 0.30
        fig.text(x, 0.872, name, fontsize=11, color=color, fontweight="bold")
        fig.text(
            x,
            0.838,
            f"覆盖 {coverage:.2f}%   FINISH {finished}/{drone_count}   碰撞 {collisions}",
            fontsize=12,
            color=COLORS["text"],
        )
    fig.text(
        0.07,
        0.845,
        f"最终覆盖率差距\n{coverage_gap:.2f}个百分点",
        fontsize=12,
        color=COLORS["text"],
        fontweight="bold",
        linespacing=1.35,
    )

    # A: aligned joint map coverage history.
    ax = axes[0]
    for result, color, label in (
        (ideal, COLORS["ideal"], "完美通信"),
        (sionna, COLORS["sionna"], "Sionna纯分布式"),
    ):
        time_s, coverage = joint_coverage_series(result)
        ax.plot(time_s, coverage, color=color, linewidth=2.4, label=label)
        ax.scatter(time_s[-1], coverage[-1], color=color, s=35, zorder=3)
        ax.annotate(
            f"{coverage[-1]:.2f}%",
            (time_s[-1], coverage[-1]),
            xytext=(-8, 8),
            textcoords="offset points",
            ha="right",
            fontsize=10,
            color=color,
            fontweight="bold",
        )
    ax.set_title("A  联合地图覆盖率（虚拟时间对齐）", loc="left", fontweight="bold")
    ax.set_xlabel("虚拟世界时间 / s")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_xlim(0, 1200)
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, loc="lower right")

    # B: final per-UAV map coverage. A narrow axis makes synchronization
    # differences visible; dots avoid visually implying a zero baseline.
    ax = axes[1]
    uavs = np.arange(1, drone_count + 1)
    ideal_per_uav = np.asarray(ideal["metrics"]["mapping_coverage_per_agent"]) * 100
    sionna_per_uav = np.asarray(sionna["metrics"]["mapping_coverage_per_agent"]) * 100
    for x, first, second in zip(uavs, ideal_per_uav, sionna_per_uav):
        ax.plot([x, x], [second, first], color="#b8c5d1", linewidth=1.2, zorder=1)
    ax.scatter(uavs - 0.08, ideal_per_uav, color=COLORS["ideal"], s=35, label="完美通信", zorder=3)
    ax.scatter(uavs + 0.08, sionna_per_uav, color=COLORS["sionna"], s=35, label="Sionna纯分布式", zorder=3)
    lower = min(ideal_per_uav.min(), sionna_per_uav.min()) - 0.5
    upper = max(ideal_per_uav.max(), sionna_per_uav.max()) + 0.5
    ax.set_title("B  各 UAV 最终地图覆盖率", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("已知体素覆盖率 / %（局部刻度）")
    ax.set_xticks(uavs)
    ax.set_ylim(lower, upper)
    ax.legend(frameon=False, loc="lower right")

    # C: execution and mission-completion outcomes.
    ax = axes[2]
    categories = ("执行过轨迹", "进入 FINISH", "记录为返航", "距起点 ≤1 m")
    ideal_values = np.asarray(
        [
            len(ideal["executed_drone_ids"]),
            len(ideal["finished_drone_ids"]),
            len(ideal["returned_drone_ids"]),
            sum(error <= 1.0 for error in ideal["return_position_errors_m"]),
        ]
    )
    sionna_values = np.asarray(
        [
            len(sionna["executed_drone_ids"]),
            len(sionna["finished_drone_ids"]),
            len(sionna["returned_drone_ids"]),
            sum(error <= 1.0 for error in sionna["return_position_errors_m"]),
        ]
    )
    positions = np.arange(len(categories))
    width = 0.36
    bars1 = ax.bar(positions - width / 2, ideal_values, width, color=COLORS["ideal"], label="完美通信")
    bars2 = ax.bar(positions + width / 2, sionna_values, width, color=COLORS["sionna"], label="Sionna纯分布式")
    ax.bar_label(bars1, padding=2, fontsize=9)
    ax.bar_label(bars2, padding=2, fontsize=9)
    ax.set_title("C  执行、完成与返航情况", loc="left", fontweight="bold")
    ax.set_ylabel("UAV 数量")
    ax.set_xticks(positions, categories)
    ax.set_ylim(0, drone_count + 2)
    ax.legend(frameon=False, loc="upper right")

    # D: attempted packet outcomes, stacked to 100%.
    ax = axes[3]
    names = ("完美通信", "Sionna纯分布式")
    outcomes = (packet_outcomes(ideal), packet_outcomes(sionna))
    stack_colors = {
        "成功送达": COLORS["green"],
        "无链路": "#8b5cf6",
        "PER丢弃": "#ef4444",
        "队列丢弃": "#f97316",
        "TTL超时": "#64748b",
        "仍在排队": "#cbd5e1",
    }
    bottoms = np.zeros(2)
    for category, color in stack_colors.items():
        values = np.asarray([item[category] for item in outcomes])
        ax.bar(names, values, bottom=bottoms, color=color, label=category)
        bottoms += values
    ax.set_title("D  尝试发送数据包的最终结果", loc="left", fontweight="bold")
    ax.set_ylabel("占尝试数据包比例 / %")
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, fontsize=8.5, ncol=2, loc="lower left")
    sionna_stats = sionna["communication"]["statistics"]
    ax.text(
        0.02,
        0.95,
        "完美通信：送达 100%，队列/丢包均为 0\n"
        f"Sionna：送达 {outcomes[1]['成功送达']:.1f}%，"
        f"端到端延迟 {float(sionna_stats['mean_end_to_end_delay_ms']):.1f} ms",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        color=COLORS["text"],
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": COLORS["grid"]},
    )

    fig.text(
        0.07,
        0.045,
        "结果：两组均无物理碰撞；Sionna通信拥塞使最终联合覆盖率降低1.86个百分点，"
        "进入FINISH的UAV由10架降至8架。",
        fontsize=10.5,
        color="#9a3412",
    )
    fig.text(
        0.07,
        0.018,
        "注：两组均由1200 s时限终止，均未实现全员完成/返航；覆盖率曲线按各UAV异步历史记录插值后取联合最大值。",
        fontsize=9,
        color=COLORS["muted"],
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    fig.savefig(args.output.with_suffix(".pdf"), facecolor=fig.get_facecolor())
    plt.close(fig)
    print(args.output)


if __name__ == "__main__":
    main()
