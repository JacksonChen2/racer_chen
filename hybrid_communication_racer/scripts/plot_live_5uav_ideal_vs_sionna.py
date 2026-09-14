#!/usr/bin/env python3
"""Plot completed ideal coverage against a still-running Sionna experiment."""

import argparse
import json
import re
from datetime import datetime
from pathlib import Path

import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np


COLORS = {
    "ideal": "#1677ff",
    "sionna": "#f59e0b",
    "danger": "#d92d20",
    "grid": "#d9e2ec",
    "text": "#172b4d",
    "muted": "#52667a",
}


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


def ideal_joint_series(result: dict) -> tuple[np.ndarray, np.ndarray]:
    metrics = result["metrics"]
    end_time = float(metrics["elapsed"])
    times = np.arange(0.0, end_time + 1.0e-6, 2.0)
    per_agent = []
    drone_count = len(metrics["mapping_coverage_per_agent"])
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
            [0.0]
            + [item[1] for item in records]
            + [float(metrics["mapping_coverage_per_agent"][drone_id])]
        )
        per_agent.append(np.interp(times, sample_t, np.maximum.accumulate(sample_y)))
    return times, np.max(np.vstack(per_agent), axis=0) * 100.0


def live_sionna_joint_series(launch_log: Path) -> tuple[np.ndarray, np.ndarray, float]:
    pattern = re.compile(
        r"\[racer_original_exploration_(\d+)\]: "
        r"RACER_MAP_COVERAGE known=(\d+) total=(\d+) ratio=([0-9.]+)"
    )
    records = {drone_id: [] for drone_id in range(1, 6)}
    crash_drone = None
    with launch_log.open(encoding="utf-8", errors="replace") as stream:
        for line in stream:
            match = pattern.search(line)
            if match:
                records[int(match.group(1))].append(float(match.group(4)))
            if "racer_original_exploration_node-21" in line and "process has died" in line:
                crash_drone = 5

    sample_count = max(len(values) for values in records.values())
    if sample_count == 0:
        raise RuntimeError("No live Sionna coverage samples were found")
    times = np.arange(0.0, 2.0 * sample_count + 1.0e-6, 2.0)
    latest = np.zeros(5)
    joint = [0.0]
    for index in range(sample_count):
        for drone_id in range(1, 6):
            values = records[drone_id]
            if index < len(values):
                latest[drone_id - 1] = values[index]
        joint.append(float(np.max(latest)) * 100.0)
    crash_time = (
        2.0 * len(records[crash_drone]) if crash_drone is not None else float("nan")
    )
    return times, np.asarray(joint), crash_time


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ideal-result", type=Path, required=True)
    parser.add_argument("--sionna-launch-log", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()

    ideal = json.loads(args.ideal_result.read_text())
    ideal_t, ideal_y = ideal_joint_series(ideal)
    sionna_t, sionna_y, crash_time = live_sionna_joint_series(
        args.sionna_launch_log
    )
    current_time = float(sionna_t[-1])
    ideal_now = float(np.interp(current_time, ideal_t, ideal_y))
    sionna_now = float(sionna_y[-1])
    ideal_final = float(ideal_y[-1])

    plt.rcParams.update({
        "font.family": choose_font(),
        "axes.unicode_minus": False,
        "axes.edgecolor": "#9fb3c8",
        "axes.labelcolor": COLORS["text"],
        "xtick.color": COLORS["muted"],
        "ytick.color": COLORS["muted"],
    })
    fig, axis = plt.subplots(figsize=(13.5, 7.6), facecolor="#f7f9fc")
    axis.set_facecolor("white")
    axis.grid(color=COLORS["grid"], linewidth=0.8, alpha=0.85)
    axis.set_axisbelow(True)

    before = ideal_t <= current_time
    axis.plot(
        ideal_t[before], ideal_y[before], color=COLORS["ideal"], linewidth=2.8,
        label="第一组：完美通信",
    )
    if np.any(~before):
        axis.plot(
            ideal_t[~before], ideal_y[~before], color=COLORS["ideal"],
            linewidth=2.0, linestyle="--", alpha=0.40,
            label="第一组：当前时刻后的已完成数据",
        )
    axis.plot(
        sionna_t, sionna_y, color=COLORS["sionna"], linewidth=2.8,
        label="第二组：Sionna（实时）",
    )

    axis.axvline(current_time, color=COLORS["muted"], linestyle=":", linewidth=1.4)
    axis.scatter(
        [current_time, current_time], [ideal_now, sionna_now],
        color=[COLORS["ideal"], COLORS["sionna"]], s=58, zorder=5,
    )
    axis.annotate(
        f"同时间 Ideal  {ideal_now:.2f}%",
        (current_time, ideal_now), xytext=(-12, 16), textcoords="offset points",
        ha="right", color=COLORS["ideal"], fontweight="bold",
    )
    axis.annotate(
        f"当前 Sionna  {sionna_now:.2f}%",
        (current_time, sionna_now), xytext=(-12, -24), textcoords="offset points",
        ha="right", color=COLORS["sionna"], fontweight="bold",
    )
    axis.annotate(
        f"Ideal 最终  {ideal_final:.2f}%",
        (ideal_t[-1], ideal_final), xytext=(-10, 14), textcoords="offset points",
        ha="right", color=COLORS["ideal"], alpha=0.75,
    )
    if np.isfinite(crash_time):
        axis.axvline(
            crash_time, color=COLORS["danger"], linestyle="--", linewidth=1.5,
            alpha=0.85,
        )
        axis.text(
            crash_time + 8, 4.2, f"UAV 5 崩溃（约 {crash_time:.0f} s）",
            color=COLORS["danger"], fontsize=10, rotation=90, va="bottom",
        )

    axis.set_xlim(0, 900)
    upper = max(40.0, ideal_y.max() * 1.18)
    axis.set_ylim(0, upper)
    axis.set_xlabel("仿真时间 / s", fontsize=11)
    axis.set_ylabel("联合地图覆盖率 / %", fontsize=11)
    axis.legend(frameon=False, loc="upper left")

    fig.suptitle(
        "5-UAV warehouse_full：完美通信 vs Sionna 实时覆盖率",
        x=0.075, y=0.965, ha="left", fontsize=19, fontweight="bold",
        color=COLORS["text"],
    )
    fig.text(
        0.075, 0.915,
        "相同场景、四角+中心起点、seed=42、50 Hz 物理、10 Hz 传感器；"
        "联合覆盖率取各 UAV 已知体素比例的最大值。",
        fontsize=10.5, color=COLORS["muted"],
    )
    difference = sionna_now - ideal_now
    fig.text(
        0.075, 0.055,
        f"当前对齐时刻约 {current_time:.0f} s：Sionna {sionna_now:.2f}% · "
        f"Ideal {ideal_now:.2f}% · 差值 {difference:+.2f} 个百分点。"
        "注意：Sionna 组 UAV 5 已崩溃，后续曲线不是完整五机结果。",
        fontsize=10.5, color=COLORS["danger"],
    )
    fig.text(
        0.925, 0.055, datetime.now().astimezone().strftime("生成于 %Y-%m-%d %H:%M:%S"),
        fontsize=9, color=COLORS["muted"], ha="right",
    )
    fig.subplots_adjust(left=0.08, right=0.97, bottom=0.14, top=0.86)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)

    print(json.dumps({
        "current_time_s": current_time,
        "ideal_coverage_percent_at_current_time": ideal_now,
        "sionna_coverage_percent_at_current_time": sionna_now,
        "difference_percentage_points": difference,
        "ideal_final_coverage_percent": ideal_final,
        "uav5_crash_time_approx_s": crash_time,
        "output": str(args.output),
    }, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
