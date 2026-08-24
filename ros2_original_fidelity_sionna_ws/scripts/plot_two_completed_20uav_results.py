#!/usr/bin/env python3
"""Plot the two completed 20-UAV warehouse experiment results."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter


CASES = (
    ("ideal_no_loss", "全连接无损", "#1565C0"),
    ("sionna_distributed", "纯分布式 Sionna", "#E65100"),
)


def load_results(suite: Path) -> dict[str, dict]:
    results = {}
    for key, _, _ in CASES:
        path = next((suite / key).glob("*_result.json"))
        results[key] = json.loads(path.read_text())
    return results


def coverage_curve(result: dict) -> tuple[list[float], list[float]]:
    samples = sorted(
        result["metrics"]["mapping_coverage_history"],
        key=lambda item: float(item["time_s"]),
    )
    times = [0.0]
    values = [0.0]
    running = 0.0
    next_bin = 2.0
    for sample in samples:
        stamp = float(sample["time_s"])
        running = max(running, float(sample["ratio"]))
        if stamp + 1.0e-9 >= next_bin:
            times.append(stamp)
            values.append(running)
            next_bin += 2.0
    times.append(float(result["metrics"]["elapsed"]))
    values.append(float(result["metrics"]["mapping_coverage_joint"]))
    return times, values


def communication_parts(result: dict) -> list[float]:
    stats = result["communication"]["statistics"]
    attempted = max(1, int(stats["attempted_packets"]))
    fields = (
        "delivered_packets",
        "dropped_queue",
        "dropped_per",
        "dropped_ttl",
        "dropped_no_link",
        "queued_packets",
    )
    return [100.0 * int(stats.get(field, 0)) / attempted for field in fields]


def configure_style() -> None:
    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            # Matplotlib 3.5 reports the shared Noto CJK TTC under its JP
            # family name; the font still contains the Simplified Chinese glyphs.
            "font.sans-serif": ["Noto Sans CJK JP", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "font.size": 10.5,
        }
    )


def plot_dashboard(results: dict[str, dict], output: Path) -> None:
    configure_style()
    figure, axes = plt.subplots(2, 2, figsize=(15.5, 10), constrained_layout=True)
    coverage_axis, distance_axis, comm_axis, outcome_axis = axes.flat

    for key, label, color in CASES:
        times, coverage = coverage_curve(results[key])
        final = float(results[key]["metrics"]["mapping_coverage_joint"])
        coverage_axis.step(times, coverage, where="post", color=color,
                           linewidth=2.2, label=f"{label}（最终 {final:.1%}）")
        coverage_axis.scatter(times[-1], final, color=color, s=35, zorder=4)
    coverage_axis.set_title("A. 联合建图覆盖率")
    coverage_axis.set_xlabel("仿真时间 (s)")
    coverage_axis.set_ylabel("联合覆盖率")
    coverage_axis.set_xlim(0, 1200)
    coverage_axis.set_ylim(0, 1.0)
    coverage_axis.yaxis.set_major_formatter(PercentFormatter(1.0))
    coverage_axis.grid(linestyle="--", alpha=0.3)
    coverage_axis.legend(loc="lower right")

    drone_ids = np.arange(1, 21)
    width = 0.38
    for offset, (key, label, color) in zip((-width / 2, width / 2), CASES):
        distances = results[key]["metrics"]["path_lengths"]
        distance_axis.bar(drone_ids + offset, distances, width=width,
                          color=color, alpha=0.85, label=label)
    distance_axis.set_title("B. 各无人机累计航程")
    distance_axis.set_xlabel("无人机编号")
    distance_axis.set_ylabel("航程 (m)")
    distance_axis.set_xticks(drone_ids)
    distance_axis.grid(axis="y", linestyle="--", alpha=0.3)
    distance_axis.legend()

    part_labels = ("已传递", "队列丢弃", "PER 丢弃", "TTL 超时", "无链路", "仍在队列")
    part_colors = ("#2E7D32", "#C62828", "#EF6C00", "#6A1B9A", "#455A64", "#F9A825")
    bottoms = np.zeros(2)
    x = np.arange(2)
    for part_index, (part_label, part_color) in enumerate(zip(part_labels, part_colors)):
        values = [communication_parts(results[key])[part_index] for key, _, _ in CASES]
        comm_axis.bar(x, values, bottom=bottoms, color=part_color, label=part_label)
        bottoms += values
    comm_axis.set_title("C. 物理通信包结果占比")
    comm_axis.set_ylabel("占尝试发送包比例")
    comm_axis.set_xticks(x, [label for _, label, _ in CASES])
    comm_axis.set_ylim(0, max(101.0, float(bottoms.max()) * 1.02))
    comm_axis.yaxis.set_major_formatter(PercentFormatter(100.0))
    comm_axis.grid(axis="y", linestyle="--", alpha=0.3)
    comm_axis.legend(ncol=2, fontsize=9, loc="lower center")

    labels = [label for _, label, _ in CASES]
    returned = [len(results[key]["returned_drone_ids"]) for key, _, _ in CASES]
    collisions = [int(results[key]["metrics"]["collision_events"]) for key, _, _ in CASES]
    outcome_axis.bar(x - width / 2, returned, width=width, color="#00897B", label="返航无人机")
    outcome_axis.bar(x + width / 2, collisions, width=width, color="#D81B60", label="碰撞事件")
    for index, value in enumerate(returned):
        outcome_axis.text(index - width / 2, value + 0.35, f"{value}/20", ha="center", fontweight="bold")
    for index, value in enumerate(collisions):
        outcome_axis.text(index + width / 2, value + 0.35, str(value), ha="center", fontweight="bold")
    outcome_axis.set_title("D. 返航与碰撞")
    outcome_axis.set_ylabel("数量")
    outcome_axis.set_xticks(x, labels)
    outcome_axis.set_ylim(0, 21)
    outcome_axis.grid(axis="y", linestyle="--", alpha=0.3)
    outcome_axis.legend()

    figure.suptitle(
        "Warehouse Full｜20 架无人机｜1200 s｜物理 60 Hz｜深度 10 Hz｜19,200 rays/frame",
        fontsize=15,
        fontweight="bold",
    )
    figure.savefig(output, dpi=180, bbox_inches="tight")
    figure.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(figure)


def plot_trajectories(results: dict[str, dict], output: Path) -> None:
    configure_style()
    figure, axes = plt.subplots(1, 2, figsize=(16, 7.5), sharex=True, sharey=True,
                               constrained_layout=True)
    colors = plt.cm.turbo(np.linspace(0.02, 0.98, 20))
    for axis, (key, label, _) in zip(axes, CASES):
        history = results[key]["metrics"]["trajectory_history"]
        positions = np.asarray([sample["positions"] for sample in history], dtype=float)
        for drone in range(20):
            x = positions[:, drone, 0]
            y = positions[:, drone, 1]
            axis.plot(x, y, color=colors[drone], linewidth=1.15, alpha=0.88,
                      label=f"UAV {drone + 1}")
            axis.scatter(x[0], y[0], marker="^", color=colors[drone], s=28,
                         edgecolors="black", linewidths=0.25, zorder=3)
            axis.scatter(x[-1], y[-1], marker="o", color=colors[drone], s=20,
                         edgecolors="black", linewidths=0.25, zorder=3)
        coverage = float(results[key]["metrics"]["mapping_coverage_joint"])
        collisions = int(results[key]["metrics"]["collision_events"])
        axis.set_title(f"{label}\n覆盖率 {coverage:.1%}｜碰撞 {collisions}")
        axis.set_xlabel("X (m)")
        axis.grid(linestyle="--", alpha=0.25)
        axis.set_aspect("equal", adjustable="box")
    axes[0].set_ylabel("Y (m)")
    handles, labels = axes[1].get_legend_handles_labels()
    figure.legend(handles, labels, loc="lower center", ncol=10, fontsize=8,
                  bbox_to_anchor=(0.5, -0.035))
    figure.suptitle("20 架无人机俯视轨迹（▲ 起点，● 终点）", fontsize=15, fontweight="bold")
    figure.savefig(output, dpi=180, bbox_inches="tight")
    figure.savefig(output.with_suffix(".pdf"), bbox_inches="tight")
    plt.close(figure)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("suite", type=Path)
    args = parser.parse_args()
    results = load_results(args.suite)
    plot_dashboard(results, args.suite / "ideal_vs_sionna_result_dashboard.png")
    plot_trajectories(results, args.suite / "ideal_vs_sionna_trajectories.png")
    print(args.suite / "ideal_vs_sionna_result_dashboard.png")
    print(args.suite / "ideal_vs_sionna_trajectories.png")


if __name__ == "__main__":
    main()
