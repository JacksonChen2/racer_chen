#!/usr/bin/env python3
"""Plot joint mapping coverage over simulation time for the three-way suite."""

import argparse
import json
from collections import defaultdict
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter


CASES = {
    "ideal_no_loss": ("Ideal distributed", "#2563eb"),
    "sionna_distributed_no_bs": ("Sionna distributed (no BS)", "#dc2626"),
    "sionna_bs_round_robin_best_effort": (
        "Sionna BS round robin (best effort)",
        "#16a34a",
    ),
}


def joint_coverage_series(result):
    """Combine asynchronous per-UAV samples into two-second joint samples."""
    bins = defaultdict(list)
    for sample in result["metrics"]["mapping_coverage_history"]:
        bins[round(float(sample["time_s"]) / 2.0)].append(float(sample["ratio"]))

    times = [0.0]
    coverage = [0.0]
    running_max = 0.0
    for bin_index in sorted(bins):
        running_max = max(running_max, max(bins[bin_index]))
        times.append(2.0 * bin_index)
        coverage.append(running_max)

    final_coverage = float(result["metrics"]["mapping_coverage_joint"])
    times.append(float(result["metrics"]["elapsed"]))
    coverage.append(final_coverage)
    return times, coverage, final_coverage


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("suite_dir", type=Path)
    parser.add_argument("--output", type=Path)
    parser.add_argument(
        "--cases", nargs="+", choices=CASES, default=list(CASES)
    )
    parser.add_argument("--title", default="Warehouse Full: Mapping Coverage Performance")
    args = parser.parse_args()
    output = args.output or args.suite_dir / "mapping_coverage_vs_time.png"

    fig, ax = plt.subplots(figsize=(12, 7), constrained_layout=True)
    maximum_time = 0.0
    final_values = []
    for case_name in args.cases:
        label, color = CASES[case_name]
        result_path = next((args.suite_dir / case_name).glob("*_result.json"))
        result = json.loads(result_path.read_text())
        times, coverage, final_coverage = joint_coverage_series(result)
        maximum_time = max(maximum_time, times[-1])
        final_values.append((label, final_coverage, color))
        ax.plot(times, coverage, label=label, color=color, linewidth=2.5)
        ax.scatter(times[-1], final_coverage, color=color, s=45, zorder=3)
        ax.annotate(
            f"{final_coverage:.1%}",
            (times[-1], final_coverage),
            xytext=(-8, 8),
            textcoords="offset points",
            color=color,
            fontsize=10,
            fontweight="bold",
            ha="right",
        )

    ax.set_title(args.title, fontsize=16)
    ax.set_xlabel("Simulation time (s)", fontsize=12)
    ax.set_ylabel("Joint mapping coverage", fontsize=12)
    ax.set_xlim(0, maximum_time * 1.025)
    ax.set_ylim(0, 0.95)
    ax.yaxis.set_major_formatter(PercentFormatter(1.0))
    ax.grid(True, which="major", alpha=0.25)
    ax.legend(loc="lower right", framealpha=0.95)
    ax.text(
        0.015,
        0.975,
        "5 UAVs | Physics 100 Hz | Depth 10 Hz | 19,200 rays/frame\n"
        f"BS 40 dBm | UAV 23 dBm | Maximum duration {maximum_time:.0f} s",
        transform=ax.transAxes,
        va="top",
        fontsize=10,
        bbox={"boxstyle": "round,pad=0.4", "facecolor": "white", "alpha": 0.85},
    )
    if len(final_values) == 2:
        delta = final_values[0][1] - final_values[1][1]
        ax.text(
            0.985,
            0.31,
            f"Final coverage difference: {delta:+.1%}",
            transform=ax.transAxes,
            ha="right",
            fontsize=11,
            fontweight="bold",
            bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.9},
        )

    output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(output, dpi=180)
    fig.savefig(output.with_suffix(".pdf"))
    print(output)


if __name__ == "__main__":
    main()
