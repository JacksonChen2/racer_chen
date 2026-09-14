#!/usr/bin/env python3
"""Plot joint coverage for the two-centre Hybrid Communication RACER pair."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.font_manager as font_manager
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.ticker import PercentFormatter


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--ideal", type=Path, required=True)
    parser.add_argument("--sionna", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
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


def joint_curve(result: dict, sample_times: np.ndarray) -> np.ndarray:
    metrics = result["metrics"]
    histories = metrics["mapping_coverage_history"]
    final_ratios = metrics["mapping_coverage_per_agent"]
    end_time = float(metrics["elapsed"])
    per_agent = []
    for drone_id, final_ratio in enumerate(final_ratios):
        records = sorted(
            (
                (float(item["time_s"]), float(item["ratio"]))
                for item in histories
                if int(item["drone_id"]) == drone_id
            ),
            key=lambda item: item[0],
        )
        record_times = np.asarray([0.0] + [item[0] for item in records] + [end_time])
        record_values = np.asarray(
            [0.0] + [item[1] for item in records] + [float(final_ratio)]
        )
        record_values = np.maximum.accumulate(record_values)
        per_agent.append(np.interp(sample_times, record_times, record_values))
    curve = 100.0 * np.max(np.vstack(per_agent), axis=0)
    curve = np.maximum.accumulate(curve)
    curve[-1] = 100.0 * float(metrics["mapping_coverage_joint"])
    return curve


def main() -> None:
    args = parse_args()
    ideal = load(args.ideal)
    sionna = load(args.sionna)
    end_time = min(float(ideal["metrics"]["elapsed"]), float(sionna["metrics"]["elapsed"]))
    times = np.arange(0.0, end_time, 1.0)
    times = np.append(times, end_time)
    ideal_curve = joint_curve(ideal, times)
    sionna_curve = joint_curve(sionna, times)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.csv.parent.mkdir(parents=True, exist_ok=True)
    with args.csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["time_s", "ideal_joint_coverage_pct", "sionna_joint_coverage_pct"]
        )
        writer.writerows(zip(times, ideal_curve, sionna_curve))

    plt.rcParams.update(
        {
            "font.family": choose_font(),
            "axes.unicode_minus": False,
            "font.size": 11,
        }
    )
    figure, axis = plt.subplots(figsize=(11.5, 6.8), constrained_layout=True)
    ideal_color = "#1677ff"
    sionna_color = "#f59e0b"
    axis.plot(
        times,
        ideal_curve,
        color=ideal_color,
        linewidth=2.6,
        label="完美通信（10/10 UAV 执行）",
    )
    axis.plot(
        times,
        sionna_curve,
        color=sionna_color,
        linewidth=2.6,
        label="Sionna 分布式（9/10 UAV 执行，未通过完整性校验）",
    )
    axis.scatter(times[-1], ideal_curve[-1], color=ideal_color, s=45, zorder=5)
    axis.scatter(times[-1], sionna_curve[-1], color=sionna_color, s=45, zorder=5)
    axis.annotate(
        f"{ideal_curve[-1]:.2f}%",
        (times[-1], ideal_curve[-1]),
        xytext=(-10, 10),
        textcoords="offset points",
        ha="right",
        color=ideal_color,
        fontweight="bold",
    )
    axis.annotate(
        f"{sionna_curve[-1]:.2f}%",
        (times[-1], sionna_curve[-1]),
        xytext=(-10, -18),
        textcoords="offset points",
        ha="right",
        color=sionna_color,
        fontweight="bold",
    )
    upper = 5.0 * np.ceil((max(ideal_curve.max(), sionna_curve.max()) + 3.0) / 5.0)
    axis.set_xlim(0.0, 300.0)
    axis.set_ylim(0.0, max(10.0, upper))
    axis.set_xticks(np.arange(0.0, 301.0, 50.0))
    axis.yaxis.set_major_formatter(PercentFormatter(xmax=100.0, decimals=0))
    axis.set_xlabel("仿真时间 / s")
    axis.set_ylabel("联合地图覆盖率")
    axis.set_title(
        "双中心起飞：Hybrid Communication RACER 覆盖率变化\n"
        "10 UAV · 300 s · seed 42 · 100 Hz 物理 · 10 Hz 深度传感"
    )
    axis.grid(True, linestyle="--", alpha=0.3)
    axis.legend(loc="upper left", framealpha=0.95)
    axis.text(
        0.99,
        0.03,
        "注：Sionna 曲线来自首轮完整 300 s 运行；UAV8 未进入执行轨迹，因此仅作诊断比较。",
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        color="#9a3412",
        fontsize=9.5,
    )
    figure.savefig(args.output, dpi=200, facecolor="white")
    figure.savefig(args.output.with_suffix(".svg"), facecolor="white")
    plt.close(figure)


if __name__ == "__main__":
    main()
