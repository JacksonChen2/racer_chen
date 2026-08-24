#!/usr/bin/env python3
"""Plot joint mapping coverage for the paired 900 s warehouse experiment."""

from pathlib import Path
import json

import matplotlib.pyplot as plt
from matplotlib import font_manager


ROOT = Path(__file__).resolve().parent
SERIES = (
    (
        "BS Round Robin + external recovery",
        ROOT / "bs_round_robin" / "warehouse_loaded_bs_round_robin_result.json",
        "#d55e00",
    ),
    (
        "No BS / original distributed RACER",
        ROOT / "distributed" / "warehouse_loaded_distributed_result.json",
        "#0072b2",
    ),
)


def load_joint_history(path: Path):
    result = json.loads(path.read_text())
    history = result["metrics"]["mapping_coverage_history"]
    drone_count = result["drone_count"]

    times = [0.0]
    coverage = [0.0]
    for start in range(0, len(history), drone_count):
        batch = history[start : start + drone_count]
        if len(batch) != drone_count:
            raise ValueError(f"Incomplete coverage sample batch in {path}")
        if len({item["drone_id"] for item in batch}) != drone_count:
            raise ValueError(f"Unexpected drone IDs in coverage history in {path}")
        times.append(max(item["time_s"] for item in batch))
        coverage.append(100.0 * max(item["ratio"] for item in batch))

    expected = 100.0 * result["metrics"]["mapping_coverage_joint"]
    if abs(coverage[-1] - expected) > 1e-8:
        raise ValueError(f"Joint coverage mismatch in {path}")
    return times, coverage


def main():
    cjk_font_path = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
    font_manager.fontManager.addfont(cjk_font_path)
    cjk_family = font_manager.FontProperties(fname=cjk_font_path).get_name()
    plt.rcParams.update(
        {
            "font.family": cjk_family,
            "axes.unicode_minus": False,
            "font.size": 11,
        }
    )
    fig, ax = plt.subplots(figsize=(10.5, 6.2), constrained_layout=True)

    endpoints = []
    for label, path, color in SERIES:
        times, coverage = load_joint_history(path)
        ax.plot(times, coverage, color=color, linewidth=2.1, label=label)
        endpoints.append((label, times[-1], coverage[-1], color))

    ax.set_xlim(0, 900)
    ax.set_ylim(0, 100)
    ax.set_xticks(range(0, 901, 100))
    ax.set_yticks(range(0, 101, 10))
    ax.set_xlabel("时间 / s")
    ax.set_ylabel("联合地图覆盖率 / %")
    ax.set_title("Warehouse Loaded：覆盖率随时间变化（900 s）")
    ax.grid(True, which="major", color="#b0b0b0", alpha=0.35, linewidth=0.8)
    ax.legend(loc="lower right", frameon=True)

    for index, (_, time_s, value, color) in enumerate(endpoints):
        vertical_offset = -16 if index == 0 else 10
        ax.scatter([time_s], [value], color=color, s=32, zorder=5)
        ax.annotate(
            f"{value:.3f}%",
            (time_s, value),
            xytext=(-8, vertical_offset),
            textcoords="offset points",
            ha="right",
            va="center",
            color=color,
            fontweight="bold",
        )

    png = ROOT / "coverage_vs_time_comparison.png"
    svg = ROOT / "coverage_vs_time_comparison.svg"
    fig.savefig(png, dpi=200, bbox_inches="tight")
    fig.savefig(svg, bbox_inches="tight")
    print(png)
    print(svg)


if __name__ == "__main__":
    main()
