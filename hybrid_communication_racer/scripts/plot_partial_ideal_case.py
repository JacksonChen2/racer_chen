#!/usr/bin/env python3
"""Plot a manually stopped RACER case from launch logs and its stop summary."""

import argparse
import csv
import json
from pathlib import Path
import re

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np


COVERAGE_RE = re.compile(
    r"\[(?P<timestamp>\d+(?:\.\d+)?)\].*"
    r"racer_original_exploration_(?P<uav>\d+)\]: "
    r"RACER_MAP_COVERAGE known=(?P<known>\d+) "
    r"total=(?P<total>\d+) ratio=(?P<ratio>[0-9.]+)"
)
FINISH_RE = re.compile(
    r"\[(?P<timestamp>\d+(?:\.\d+)?)\].*"
    r"racer_(?:original|recovery)_exploration_(?P<uav>\d+).*"
    r"(?:finish exploration|state: FINISH)"
)
COVERAGE_PERIOD_S = 2.0


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


def load_launch_metrics(path: Path):
    raw_coverage = {uav: [] for uav in range(1, 11)}
    raw_finish_wall_s = {}
    with path.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = COVERAGE_RE.search(line)
            if match:
                uav = int(match.group("uav"))
                ratio = float(match.group("ratio")) * 100.0
                known = int(match.group("known"))
                total = int(match.group("total"))
                raw_coverage[uav].append(
                    (float(match.group("timestamp")), ratio, known, total)
                )
            finish = FINISH_RE.search(line)
            if finish:
                uav = int(finish.group("uav"))
                raw_finish_wall_s.setdefault(uav, float(finish.group("timestamp")))
    if not all(raw_coverage.values()):
        missing = [uav for uav, values in raw_coverage.items() if not values]
        raise RuntimeError(f"coverage records missing for UAVs {missing}")

    # The coverage callback is a ROS-time timer with a fixed 2 s period.  The
    # first callback is emitted as /clock starts, followed by one sample every
    # 2 s of Isaac physics time.  Console timestamps are wall clock, so use
    # callback ordinals for coverage and only use their median wall timestamps
    # as anchors to interpolate one-off FSM events onto the same /clock axis.
    coverage = {
        uav: [
            (index * COVERAGE_PERIOD_S, ratio, known, total)
            for index, (_, ratio, known, total) in enumerate(values)
        ]
        for uav, values in raw_coverage.items()
    }
    sample_count = max(len(values) for values in raw_coverage.values())
    anchor_virtual_s = []
    anchor_wall_s = []
    joint_events = []
    for index in range(sample_count):
        available = [
            values[index]
            for values in raw_coverage.values()
            if index < len(values)
        ]
        anchor_virtual_s.append(index * COVERAGE_PERIOD_S)
        anchor_wall_s.append(float(np.median([item[0] for item in available])))
        joint_events.append(
            (index * COVERAGE_PERIOD_S, max(item[1] for item in available))
        )
    finish_seconds = {
        uav: float(np.interp(wall_s, anchor_wall_s, anchor_virtual_s))
        for uav, wall_s in raw_finish_wall_s.items()
    }
    return coverage, finish_seconds, joint_events


def write_summary_csv(path: Path, coverage, finish_seconds) -> None:
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=(
                "uav_id", "latest_coverage_percent", "known_voxels",
                "total_voxels", "finished", "finish_virtual_time_s",
            ),
        )
        writer.writeheader()
        for uav in range(1, 11):
            _, ratio, known, total = coverage[uav][-1]
            writer.writerow({
                "uav_id": uav,
                "latest_coverage_percent": ratio,
                "known_voxels": known,
                "total_voxels": total,
                "finished": uav in finish_seconds,
                "finish_virtual_time_s": finish_seconds.get(uav, ""),
            })


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--case-dir", type=Path, required=True)
    parser.add_argument("--driver-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--summary-csv", type=Path, required=True)
    args = parser.parse_args()

    launch_log = args.case_dir / "warehouse_full_distributed_launch.log"
    stop_summary = json.loads(
        (args.case_dir / "manual_stop_summary.json").read_text(encoding="utf-8")
    )
    coverage, finish_seconds, joint_events = load_launch_metrics(launch_log)
    write_summary_csv(args.summary_csv, coverage, finish_seconds)

    plt.rcParams.update({
        "font.family": choose_font(),
        "axes.unicode_minus": False,
        "axes.edgecolor": "#9fb3c8",
        "axes.labelcolor": "#172b4d",
        "xtick.color": "#52667a",
        "ytick.color": "#52667a",
    })
    colors = plt.cm.tab10(np.linspace(0.0, 1.0, 10))
    fig = plt.figure(figsize=(16, 10), facecolor="#f7f9fc")
    grid = fig.add_gridspec(
        2, 2, left=0.07, right=0.97, bottom=0.10, top=0.78,
        hspace=0.36, wspace=0.24,
    )
    axes = [fig.add_subplot(grid[index]) for index in range(4)]
    for axis in axes:
        axis.set_facecolor("white")
        axis.grid(axis="y", color="#d9e2ec", linewidth=0.8)
        axis.set_axisbelow(True)

    latest = np.asarray([coverage[uav][-1][1] for uav in range(1, 11)])
    joint_latest = float(latest.max())
    stop_s = float(stop_summary["last_completed_diagnostic_time_s"])
    requested_s = float(stop_summary["requested_limit_s"])
    finished_count = len(finish_seconds)

    fig.suptitle(
        "第 1 组：10-UAV Ideal 全无损通信实验结果",
        x=0.07, y=0.965, ha="left", fontsize=21, fontweight="bold",
        color="#172b4d",
    )
    fig.text(
        0.07, 0.918,
        "warehouse_full · 三批 4+3+3 布局 · seed=42 · 50 Hz 物理 · 10 Hz 传感器",
        fontsize=11.5, color="#52667a",
    )
    fig.text(
        0.07, 0.858, "联合覆盖率", fontsize=10.5, color="#52667a"
    )
    fig.text(
        0.07, 0.818, f"{joint_latest:.2f}%", fontsize=23,
        color="#1677ff", fontweight="bold",
    )
    fig.text(0.27, 0.858, "完成探索", fontsize=10.5, color="#52667a")
    fig.text(
        0.27, 0.818, f"{finished_count}/10", fontsize=23,
        color="#22a06b", fontweight="bold",
    )
    fig.text(0.44, 0.858, "仿真停止时间", fontsize=10.5, color="#52667a")
    fig.text(
        0.44, 0.818, f"约 {stop_s:.0f} s", fontsize=23,
        color="#f59e0b", fontweight="bold",
    )
    fig.text(0.66, 0.858, "进程崩溃", fontsize=10.5, color="#52667a")
    fig.text(
        0.66, 0.818, str(stop_summary["observed_process_crashes"]),
        fontsize=23, color="#22a06b", fontweight="bold",
    )

    ax = axes[0]
    for uav in range(1, 11):
        points = coverage[uav]
        ax.plot(
            [item[0] for item in points], [item[1] for item in points],
            color=colors[uav - 1], linewidth=1.2, alpha=0.75,
            label=f"UAV {uav}",
        )
    if joint_events:
        ax.plot(
            [item[0] for item in joint_events],
            [item[1] for item in joint_events],
            color="#111827", linewidth=2.6, label="联合最大值",
        )
    ax.set_title("A  地图覆盖率随仿真虚拟时间变化", loc="left", fontweight="bold")
    ax.set_xlabel("仿真虚拟时间 / s")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_ylim(0, 100)
    ax.legend(frameon=False, fontsize=8, ncol=3, loc="lower right")

    ax = axes[1]
    uavs = np.arange(1, 11)
    bars = ax.bar(uavs, latest, color=colors, width=0.72)
    ax.set_title("B  停止时各 UAV 地图覆盖率", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("已知体素覆盖率 / %")
    ax.set_xticks(uavs)
    ax.set_ylim(84, 89)
    for bar, value in zip(bars, latest):
        ax.text(
            bar.get_x() + bar.get_width() / 2, value + 0.08,
            f"{value:.2f}", ha="center", va="bottom", fontsize=8,
        )

    ax = axes[2]
    finish_values = [finish_seconds.get(uav, np.nan) for uav in range(1, 11)]
    finished_mask = np.asarray([np.isfinite(value) for value in finish_values])
    ax.scatter(
        uavs[finished_mask], np.asarray(finish_values)[finished_mask],
        color="#22a06b", s=90, label="已完成", zorder=3,
    )
    unfinished = uavs[~finished_mask]
    ax.scatter(
        unfinished, np.full(len(unfinished), stop_s),
        color="#ef4444", marker="x", s=90, linewidths=2,
        label="停止时未完成", zorder=3,
    )
    ax.set_title("C  RACER FSM 完成状态与时间", loc="left", fontweight="bold")
    ax.set_xlabel("UAV 编号")
    ax.set_ylabel("仿真虚拟时间 / s")
    ax.set_xticks(uavs)
    ax.set_ylim(0, stop_s * 1.08)
    ax.legend(frameon=False, loc="upper left")

    ax = axes[3]
    ax.axis("off")
    ax.set_title("D  结果说明", loc="left", fontweight="bold")
    explanation = (
        f"请求上限：{requested_s:.0f} s\n"
        f"最后诊断点：{stop_s:.0f} s（超过 {stop_s-requested_s:.0f} s）\n"
        f"联合覆盖率：{joint_latest:.3f}%\n"
        f"最低单机覆盖率：{latest.min():.3f}%（UAV {latest.argmin()+1}）\n"
        f"完成 UAV：{', '.join(map(str, sorted(finish_seconds)))}\n"
        f"未完成 UAV：{', '.join(map(str, sorted(set(range(1, 11))-set(finish_seconds))))}\n"
        "通信模式：Ideal distributed，全链路无损\n"
        "最终结果 JSON：无（人工中止）"
    )
    ax.text(
        0.04, 0.92, explanation, transform=ax.transAxes,
        va="top", fontsize=13, color="#172b4d", linespacing=1.65,
        bbox={"boxstyle": "round,pad=0.7", "facecolor": "#f8fafc", "edgecolor": "#cbd5e1"},
    )

    fig.text(
        0.07, 0.045,
        "注意：本组原计划 1200 s；收到 900 s 上限要求时已超过 900 s，随后人工停止。"
        "横轴为 Isaac Physics /clock 虚拟时间；覆盖率和完成状态来自停止前最后日志。",
        fontsize=10.5, color="#9a3412",
    )
    fig.text(
        0.07, 0.018,
        "由于中止过程未生成 RACER_3D_ISAAC_RESULT，本图不报告最终路径长度、最小障碍净距或最终碰撞计数。",
        fontsize=9.5, color="#52667a",
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)


if __name__ == "__main__":
    main()
