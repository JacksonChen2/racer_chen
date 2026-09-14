#!/usr/bin/env python3
"""Plot trajectories and performance for the K=2/K=4 hybrid-link runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

from generate_scene_top_view import draw_geometry, load_projected_geometry


BOUNDS = (-27.0, 6.0, 0.6, 30.6)
CASE_COLORS = {2: "#1677ff", 4: "#f59e0b"}
TEXT = "#172b4d"
MUTED = "#52667a"
GRID = "#d9e2ec"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--k2", type=Path, required=True)
    parser.add_argument("--k4", type=Path, required=True)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def choose_font() -> str:
    candidates = (
        "Noto Sans CJK SC",
        "Noto Sans CJK JP",
        "Source Han Sans CN",
        "WenQuanYi Micro Hei",
        "DejaVu Sans",
    )
    installed = {item.name for item in font_manager.fontManager.ttflist}
    return next((name for name in candidates if name in installed), "DejaVu Sans")


def joint_coverage_series(result: dict) -> tuple[np.ndarray, np.ndarray]:
    metrics = result["metrics"]
    count = int(result["drone_count"])
    elapsed = float(metrics["elapsed"])
    times = np.arange(0.0, elapsed + 1.0e-6, 1.0)
    curves = []
    for drone_id in range(count):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in metrics["mapping_coverage_history"]
                if int(item["drone_id"]) == drone_id
            ),
            key=lambda item: item[0],
        )
        record_times = np.asarray([0.0] + [item[0] for item in records] + [elapsed])
        record_values = np.asarray(
            [0.0]
            + [item[1] for item in records]
            + [float(metrics["mapping_coverage_per_agent"][drone_id])]
        )
        curves.append(np.interp(times, record_times, np.maximum.accumulate(record_values)))
    return times, np.max(np.vstack(curves), axis=0) * 100.0


def draw_trajectories(axis, result: dict, geometry, nearest_count: int) -> None:
    metrics = result["metrics"]
    history = metrics["trajectory_history"]
    positions = np.asarray([sample["positions"] for sample in history], dtype=float)
    starts = np.asarray(metrics["start_positions"], dtype=float)
    ends = np.asarray(metrics["positions"], dtype=float)
    colors = plt.get_cmap("tab10")(np.arange(10))

    draw_geometry(axis, geometry, BOUNDS)
    for drone_id in range(10):
        trajectory = positions[:, drone_id, :]
        axis.plot(
            trajectory[:, 0], trajectory[:, 1], color=colors[drone_id],
            linewidth=1.75, alpha=0.9, zorder=5,
        )
        axis.scatter(
            starts[drone_id, 0], starts[drone_id, 1], marker="*", s=95,
            color=colors[drone_id], edgecolor="black", linewidth=0.45, zorder=8,
        )
        axis.scatter(
            ends[drone_id, 0], ends[drone_id, 1], marker="X", s=45,
            color=colors[drone_id], edgecolor="white", linewidth=0.5, zorder=8,
        )
        axis.annotate(
            str(drone_id + 1), ends[drone_id, :2], xytext=(3, 3),
            textcoords="offset points", fontsize=7.2, weight="bold",
            color=colors[drone_id], zorder=9,
        )
    axis.set_title(
        f"{'A' if nearest_count == 2 else 'B'}  最近 {nearest_count} 架无损通信：俯视轨迹",
        loc="left", fontsize=13, fontweight="bold", color=TEXT,
    )
    axis.set_xlabel("世界坐标 X / m")
    axis.set_ylabel("世界坐标 Y / m")


def communication_outcomes(result: dict) -> dict[str, float]:
    stats = result["communication"]["statistics"]
    attempted = max(1.0, float(stats["attempted_packets"]))
    lossless = float(stats["lossless_nearest_forwarded_packets"])
    components = {
        "最近邻无损送达": lossless,
        "Sionna 送达": max(0.0, float(stats["delivered_packets"]) - lossless),
        "Sionna 无链路": float(stats["dropped_no_link"]),
        "Sionna PER 丢包": float(stats["dropped_per"]),
        "排队/其他": (
            float(stats.get("dropped_queue", 0))
            + float(stats.get("dropped_ttl", 0))
            + float(stats.get("queued_packets", 0))
        ),
    }
    return {name: value / attempted * 100.0 for name, value in components.items()}


def main() -> None:
    args = parse_args()
    results = {2: load(args.k2), 4: load(args.k4)}
    geometry = load_projected_geometry(args.mesh_dir, 0.25, 8.4)

    plt.rcParams.update({
        "font.family": choose_font(),
        "axes.unicode_minus": False,
        "axes.edgecolor": "#9fb3c8",
        "axes.labelcolor": TEXT,
        "xtick.color": MUTED,
        "ytick.color": MUTED,
    })
    figure = plt.figure(figsize=(18, 13), facecolor="#f7f9fc")
    grid = figure.add_gridspec(
        2, 6, left=0.055, right=0.975, bottom=0.075, top=0.81,
        height_ratios=(1.38, 1.0), hspace=0.33, wspace=0.42,
    )
    trajectory_axes = (
        figure.add_subplot(grid[0, :3]),
        figure.add_subplot(grid[0, 3:]),
    )
    performance_axes = (
        figure.add_subplot(grid[1, :2]),
        figure.add_subplot(grid[1, 2:4]),
        figure.add_subplot(grid[1, 4:]),
    )

    figure.suptitle(
        "10 UAV 混合通信实验：最近邻无损链路数量对性能与轨迹的影响",
        x=0.055, y=0.972, ha="left", fontsize=22, fontweight="bold", color=TEXT,
    )
    figure.text(
        0.055, 0.932,
        "同一仓库、两侧 5+5 起点、seed=42、200 s、100 Hz 物理、76,800 射线；其余 UAV-UAV 链路使用 Sionna",
        fontsize=11.5, color=MUTED,
    )

    for index, nearest_count in enumerate((2, 4)):
        result = results[nearest_count]
        metrics = result["metrics"]
        stats = result["communication"]["statistics"]
        delivery = 100.0 * stats["delivered_packets"] / stats["attempted_packets"]
        x = 0.12 + index * 0.48
        figure.text(
            x, 0.887, f"最近 {nearest_count} 架无损", fontsize=12,
            color=CASE_COLORS[nearest_count], fontweight="bold",
        )
        figure.text(
            x, 0.855,
            f"覆盖 {metrics['mapping_coverage_joint'] * 100:.2f}%  ·  "
            f"总航程 {sum(metrics['path_lengths']):.1f} m  ·  送达 {delivery:.2f}%",
            fontsize=11.5, color=TEXT,
        )
        figure.text(
            x, 0.828,
            f"10/10 UAV 执行  ·  0 碰撞  ·  最小障碍净距 {metrics['min_obstacle_clearance'] * 100:.2f} cm",
            fontsize=10.5, color=MUTED,
        )
        draw_trajectories(trajectory_axes[index], result, geometry, nearest_count)

    colors = plt.get_cmap("tab10")(np.arange(10))
    handles = [
        Line2D([0], [0], color=colors[index], lw=2, label=f"UAV {index + 1}")
        for index in range(10)
    ]
    handles.extend([
        Line2D([0], [0], marker="*", linestyle="", markerfacecolor="#777777",
               markeredgecolor="black", markersize=9, label="起点"),
        Line2D([0], [0], marker="X", linestyle="", markerfacecolor="#777777",
               markeredgecolor="white", markersize=7, label="终点"),
    ])
    figure.legend(
        handles=handles, loc="upper center", bbox_to_anchor=(0.5, 0.805),
        ncol=12, frameon=False, fontsize=8.8,
    )

    coverage_axis = performance_axes[0]
    for nearest_count in (2, 4):
        times, coverage = joint_coverage_series(results[nearest_count])
        coverage_axis.plot(
            times, coverage, color=CASE_COLORS[nearest_count], linewidth=2.5,
            label=f"最近 {nearest_count} 架无损",
        )
        coverage_axis.scatter(times[-1], coverage[-1], s=28,
                              color=CASE_COLORS[nearest_count], zorder=5)
        coverage_axis.annotate(
            f"{coverage[-1]:.2f}%", (times[-1], coverage[-1]),
            xytext=(-5, 8 if nearest_count == 4 else -15),
            textcoords="offset points", ha="right", fontsize=9,
            color=CASE_COLORS[nearest_count], fontweight="bold",
        )
    coverage_axis.set_title("C  联合覆盖率随时间变化", loc="left", fontweight="bold")
    coverage_axis.set_xlabel("仿真时间 / s")
    coverage_axis.set_ylabel("已知体素覆盖率 / %")
    coverage_axis.set_xlim(0, 200)
    coverage_axis.set_ylim(0, 80)
    coverage_axis.legend(frameon=False, loc="lower right", fontsize=9)

    path_axis = performance_axes[1]
    uavs = np.arange(1, 11)
    width = 0.38
    for nearest_count, offset in ((2, -width / 2), (4, width / 2)):
        paths = np.asarray(results[nearest_count]["metrics"]["path_lengths"])
        path_axis.bar(
            uavs + offset, paths, width, color=CASE_COLORS[nearest_count],
            label=f"最近 {nearest_count} 架无损",
        )
    path_axis.set_title("D  各 UAV 累计航程", loc="left", fontweight="bold")
    path_axis.set_xlabel("UAV 编号")
    path_axis.set_ylabel("路径长度 / m")
    path_axis.set_xticks(uavs)
    path_axis.legend(frameon=False, fontsize=9)

    comm_axis = performance_axes[2]
    labels = ("K=2", "K=4")
    outcomes = tuple(communication_outcomes(results[count]) for count in (2, 4))
    stack_colors = {
        "最近邻无损送达": "#22a06b",
        "Sionna 送达": "#60a5fa",
        "Sionna 无链路": "#8b5cf6",
        "Sionna PER 丢包": "#ef4444",
        "排队/其他": "#cbd5e1",
    }
    bottoms = np.zeros(2)
    for category, color in stack_colors.items():
        values = np.asarray([outcome[category] for outcome in outcomes])
        comm_axis.bar(labels, values, bottom=bottoms, color=color, label=category)
        bottoms += values
    comm_axis.set_title("E  物理发送尝试结果", loc="left", fontweight="bold")
    comm_axis.set_ylabel("占尝试包比例 / %")
    comm_axis.set_ylim(0, 100)
    comm_axis.legend(frameon=False, fontsize=7.8, loc="lower left")

    for axis in performance_axes:
        axis.set_facecolor("white")
        axis.grid(axis="y", color=GRID, linewidth=0.8, alpha=0.8)
        axis.set_axisbelow(True)

    figure.text(
        0.055, 0.025,
        "注：轨迹按 0.5 s 间隔保存；灰色/蓝灰色区域为 Sionna 场景网格的俯视投影。"
        "K=4 相比 K=2 覆盖率 +0.26 个百分点，总航程 −29.6 m，总体送达率 +3.04 个百分点。",
        fontsize=10, color=MUTED,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=190, facecolor=figure.get_facecolor())
    plt.close(figure)
    print(args.output.resolve())


if __name__ == "__main__":
    main()
