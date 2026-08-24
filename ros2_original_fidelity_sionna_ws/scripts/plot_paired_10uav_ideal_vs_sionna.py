#!/usr/bin/env python3
"""Plot a like-for-like comparison from paired RACER raw experiment logs."""

import argparse
import csv
import json
import re
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


def last_prefixed_json(path: Path, marker: str) -> dict:
    result = None
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            position = line.find(marker)
            if position >= 0:
                result = json.loads(line[position + len(marker):])
    if result is None:
        raise RuntimeError(f"{marker!r} not found in {path}")
    return result


def load_case(case_dir: Path) -> dict:
    isaac_log = case_dir / "warehouse_full_distributed_isaac.log"
    launch_log = case_dir / "warehouse_full_distributed_launch.log"
    metrics = last_prefixed_json(isaac_log, "RACER_3D_ISAAC_RESULT ")
    communication = last_prefixed_json(launch_log, "RACER_SIONNA_STATS ")
    launch_text = launch_log.read_text(encoding="utf-8", errors="replace")
    finished = sorted({
        int(value)
        for value in re.findall(
            r"racer_(?:original|recovery)_exploration_(\d+).*"
            r"(?:finish exploration|state: FINISH)",
            launch_text,
        )
    })
    return {"metrics": metrics, "communication": communication, "finished": finished}


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


def joint_coverage_series(case: dict) -> tuple[np.ndarray, np.ndarray]:
    metrics = case["metrics"]
    drone_count = len(metrics["mapping_coverage_per_agent"])
    end_time = float(metrics["elapsed"])
    times = np.arange(0.0, end_time + 1.0e-6, 2.0)
    per_agent = []
    for drone_id in range(drone_count):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in metrics["mapping_coverage_history"]
                if int(item["drone_id"]) == drone_id
            ),
            key=lambda item: item[0],
        )
        sample_t = np.asarray([0.0] + [item[0] for item in records] + [end_time])
        sample_y = np.asarray(
            [0.0] + [item[1] for item in records]
            + [float(metrics["mapping_coverage_per_agent"][drone_id])]
        )
        sample_y = np.maximum.accumulate(sample_y)
        per_agent.append(np.interp(times, sample_t, sample_y))
    return times, np.max(np.vstack(per_agent), axis=0) * 100.0


def packet_outcomes(case: dict) -> dict[str, float]:
    stats = case["communication"]
    attempted = max(1.0, float(stats["attempted_packets"]))
    return {
        "成功传输": float(stats["delivered_packets"]) / attempted * 100.0,
        "无链路": float(stats["dropped_no_link"]) / attempted * 100.0,
        "PER 丢包": float(stats["dropped_per"]) / attempted * 100.0,
        "队列丢包": float(stats["dropped_queue"]) / attempted * 100.0,
        "TTL 丢包": float(stats["dropped_ttl"]) / attempted * 100.0,
        "仍在队列": float(stats["queued_packets"]) / attempted * 100.0,
    }


def write_summary(path: Path, ideal: dict, sionna: dict) -> None:
    rows = []
    for name, case in (("ideal_no_loss", ideal), ("sionna", sionna)):
        metrics = case["metrics"]
        stats = case["communication"]
        rows.append({
            "case": name,
            "elapsed_s": metrics["elapsed"],
            "joint_coverage_percent": metrics["mapping_coverage_joint"] * 100.0,
            "finished_uavs": len(case["finished"]),
            "collision_events": metrics["collision_events"],
            "min_inter_drone_m": metrics["min_inter_drone"],
            "min_obstacle_clearance_m": metrics["min_obstacle_clearance"],
            "total_path_length_m": sum(metrics["path_lengths"]),
            "attempted_packets": stats["attempted_packets"],
            "delivered_packets": stats["delivered_packets"],
            "physical_delivery_ratio": (
                stats["delivered_packets"] / max(1, stats["attempted_packets"])
            ),
            "logical_delivery_ratio": stats["logical_delivery_ratio"],
            "mean_end_to_end_delay_ms": stats["mean_end_to_end_delay_ms"],
        })
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ideal-dir", type=Path, required=True)
    parser.add_argument("--sionna-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary-csv", type=Path, required=True)
    args = parser.parse_args()

    ideal = load_case(args.ideal_dir)
    sionna = load_case(args.sionna_dir)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    write_summary(args.summary_csv, ideal, sionna)

    plt.rcParams.update({
        "font.family": choose_font(),
        "axes.unicode_minus": False,
        "axes.edgecolor": "#9fb3c8",
        "axes.labelcolor": COLORS["text"],
        "xtick.color": COLORS["muted"],
        "ytick.color": COLORS["muted"],
    })
    fig = plt.figure(figsize=(16, 10), facecolor="#f7f9fc")
    grid = fig.add_gridspec(
        2, 2, left=0.07, right=0.97, bottom=0.09, top=0.77,
        hspace=0.34, wspace=0.22,
    )
    axes = [fig.add_subplot(grid[index]) for index in range(4)]
    for axis in axes:
        axis.set_facecolor("white")
        axis.grid(axis="y", color=COLORS["grid"], linewidth=0.8, alpha=0.8)
        axis.set_axisbelow(True)

    fig.suptitle(
        "10-UAV warehouse_full：Ideal 无损通信 vs Sionna 通信",
        x=0.07, y=0.965, ha="left", fontsize=21, fontweight="bold",
        color=COLORS["text"],
    )
    fig.text(
        0.07, 0.918,
        "同一布局与参数：seed=42 · 1200 s · 50 Hz 物理 · 10 Hz 传感器 · 19,200 射线",
        fontsize=11.5, color=COLORS["muted"],
    )

    for index, (name, color, case) in enumerate((
        ("Ideal 无损", COLORS["ideal"], ideal),
        ("Sionna", COLORS["sionna"], sionna),
    )):
        metrics = case["metrics"]
        x = 0.28 + index * 0.35
        fig.text(x, 0.865, name, fontsize=11, color=color, fontweight="bold")
        fig.text(
            x, 0.829,
            f"覆盖 {metrics['mapping_coverage_joint'] * 100:.2f}%   "
            f"完成 {len(case['finished'])}/10   碰撞 {metrics['collision_events']}",
            fontsize=12, color=COLORS["text"],
        )
        fig.text(
            x, 0.798,
            f"最小障碍净距 {metrics['min_obstacle_clearance']:+.3f} m",
            fontsize=10.5,
            color="#b42318" if metrics["min_obstacle_clearance"] < 0 else COLORS["muted"],
        )

    ax = axes[0]
    for case, color, label, annotation_y in (
        (ideal, COLORS["ideal"], "Ideal 无损", -15),
        (sionna, COLORS["sionna"], "Sionna", 8),
    ):
        times, coverage = joint_coverage_series(case)
        ax.plot(times, coverage, color=color, linewidth=2.4, label=label)
        ax.annotate(
            f"{coverage[-1]:.2f}%", (times[-1], coverage[-1]),
            xytext=(-8, annotation_y), textcoords="offset points", ha="right",
            color=color, fontweight="bold",
        )
    ax.set_title("A  联合地图覆盖率", loc="left", fontweight="bold")
    ax.set_xlabel("仿真时间 / s")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_xlim(0, 1200)
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, loc="lower right")

    ax = axes[1]
    uavs = np.arange(1, 11)
    width = 0.38
    ideal_coverage = np.asarray(ideal["metrics"]["mapping_coverage_per_agent"]) * 100
    sionna_coverage = np.asarray(sionna["metrics"]["mapping_coverage_per_agent"]) * 100
    ax.bar(uavs - width / 2, ideal_coverage, width, color=COLORS["ideal"], label="Ideal 无损")
    ax.bar(uavs + width / 2, sionna_coverage, width, color=COLORS["sionna"], label="Sionna")
    ax.set_title("B  各 UAV 最终地图覆盖率", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_xticks(uavs)
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, loc="lower right")

    ax = axes[2]
    ideal_paths = np.asarray(ideal["metrics"]["path_lengths"])
    sionna_paths = np.asarray(sionna["metrics"]["path_lengths"])
    ax.bar(uavs - width / 2, ideal_paths, width, color=COLORS["ideal"], label="Ideal 无损")
    ax.bar(uavs + width / 2, sionna_paths, width, color=COLORS["sionna"], label="Sionna")
    ax.set_title("C  各 UAV 累计飞行距离", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("路径长度 / m")
    ax.set_xticks(uavs)
    ax.legend(frameon=False, loc="upper right")
    ax.text(
        0.01, 0.96,
        f"总计：Ideal {ideal_paths.sum():.0f} m · Sionna {sionna_paths.sum():.0f} m",
        transform=ax.transAxes, va="top", fontsize=10, color=COLORS["muted"],
    )

    ax = axes[3]
    outcomes = (packet_outcomes(ideal), packet_outcomes(sionna))
    stack_colors = {
        "成功传输": "#22a06b",
        "无链路": "#8b5cf6",
        "PER 丢包": "#ef4444",
        "队列丢包": "#f97316",
        "TTL 丢包": "#64748b",
        "仍在队列": "#cbd5e1",
    }
    bottoms = np.zeros(2)
    for category, color in stack_colors.items():
        values = np.asarray([item[category] for item in outcomes])
        ax.bar(("Ideal 无损", "Sionna"), values, bottom=bottoms, color=color, label=category)
        bottoms += values
    ax.set_title("D  物理发送尝试结果", loc="left", fontweight="bold")
    ax.set_ylabel("占尝试数据包比例 / %")
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, fontsize=8.5, ncol=2, loc="lower left")
    ideal_delay = ideal["communication"]["mean_end_to_end_delay_ms"]
    sionna_delay = sionna["communication"]["mean_end_to_end_delay_ms"]
    ax.text(
        0.02, 0.96,
        f"平均端到端延迟：Ideal {ideal_delay:.2f} ms · Sionna {sionna_delay:.2f} ms",
        transform=ax.transAxes, va="top", fontsize=9.5, color=COLORS["text"],
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "edgecolor": COLORS["grid"]},
    )

    fig.text(
        0.07, 0.035,
        "说明：两组使用相同场景、起点和随机种子。完成数来自 RACER FSM 的 FINISH 日志；"
        "Sionna 组最小障碍净距为负，因此未通过原科学验收条件。",
        fontsize=10, color="#9a3412",
    )
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
