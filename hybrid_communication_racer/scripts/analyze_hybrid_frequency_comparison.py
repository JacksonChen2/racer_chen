#!/usr/bin/env python3
"""Compare perfect, 28 GHz/20 dBm, and 60 GHz/10 dBm hybrid runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_hybrid_communication_pair import joint_series, summarize


CASE_PATHS = {
    "perfect": "ideal_perfect_communication",
    "sionna_28ghz_20dbm": "sionna_actual_communication",
}


def load_case(case_dir: Path) -> tuple[dict, str]:
    files = list(case_dir.glob("*_result.json"))
    if len(files) != 1:
        raise RuntimeError(f"expected one result JSON in {case_dir}, found {len(files)}")
    result = json.loads(files[0].read_text(encoding="utf-8"))
    launch = (case_dir / "warehouse_full_distributed_launch.log").read_text(
        encoding="utf-8", errors="replace"
    )
    return result, launch


def delta(left: dict, right: dict) -> dict:
    return {
        "joint_coverage_percentage_points": (
            left["joint_coverage_percent"] - right["joint_coverage_percent"]
        ),
        "coverage_auc_percent_seconds": (
            left["coverage_auc_percent_seconds"] - right["coverage_auc_percent_seconds"]
        ),
        "coverage_growth_20_100_pp_per_s": (
            left["coverage_growth_20_100_pp_per_s"]
            - right["coverage_growth_20_100_pp_per_s"]
        ),
        "physical_delivery_percentage_points": 100.0 * (
            left["physical_delivery_ratio"] - right["physical_delivery_ratio"]
        ),
        "mean_end_to_end_delay_ms": (
            left["mean_end_to_end_delay_ms"] - right["mean_end_to_end_delay_ms"]
        ),
    }


def write_plot(path: Path, results: dict[str, dict], summaries: dict[str, dict]) -> None:
    keys = ("perfect", "sionna_28ghz_20dbm", "sionna_60ghz_10dbm")
    labels = ("Perfect", "28 GHz / 20 dBm", "60 GHz / 10 dBm")
    colors = ("#1677ff", "#22a06b", "#f59e0b")
    fig, axes = plt.subplots(2, 2, figsize=(14, 8), constrained_layout=True)
    fig.suptitle("Hybrid communication RACER: frequency/power comparison, 10 UAV, 200 s")
    for key, label, color in zip(keys, labels, colors):
        times, coverage = joint_series(results[key]["metrics"])
        axes[0, 0].plot(times, coverage, label=label, color=color, linewidth=2.2)
    axes[0, 0].set_title("Joint mapping coverage")
    axes[0, 0].set_xlabel("Simulation time (s)")
    axes[0, 0].set_ylabel("Coverage (%)")
    axes[0, 0].grid(alpha=0.25)
    axes[0, 0].legend(frameon=False)

    x = np.arange(len(keys))
    coverage_values = [summaries[key]["joint_coverage_percent"] for key in keys]
    axes[0, 1].bar(x, coverage_values, color=colors)
    axes[0, 1].set_title("Coverage at 200 s")
    axes[0, 1].set_ylabel("Coverage (%)")
    axes[0, 1].set_xticks(x, labels)
    for index, value in enumerate(coverage_values):
        axes[0, 1].text(index, value, f"{value:.2f}%", ha="center", va="bottom")

    delivery = [100.0 * summaries[key]["physical_delivery_ratio"] for key in keys]
    axes[1, 0].bar(x, delivery, color=colors)
    axes[1, 0].set_title("Physical packet delivery")
    axes[1, 0].set_ylabel("Delivered / attempted (%)")
    axes[1, 0].set_xticks(x, labels)
    axes[1, 0].set_ylim(0, 105)
    for index, value in enumerate(delivery):
        axes[1, 0].text(index, value, f"{value:.2f}%", ha="center", va="bottom")

    delay = [summaries[key]["mean_end_to_end_delay_ms"] for key in keys]
    axes[1, 1].bar(x, delay, color=colors)
    axes[1, 1].set_title("Mean end-to-end delay")
    axes[1, 1].set_ylabel("Delay (ms)")
    axes[1, 1].set_xticks(x, labels)
    for index, value in enumerate(delay):
        axes[1, 1].text(index, value, f"{value:.2f}", ha="center", va="bottom")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_suite", type=Path)
    parser.add_argument("frequency_suite", type=Path)
    args = parser.parse_args()
    results = {}
    summaries = {}
    for key, directory in CASE_PATHS.items():
        result, launch = load_case(args.baseline_suite / directory)
        results[key] = result
        summaries[key] = summarize(result, launch)
    result, launch = load_case(args.frequency_suite / "sionna_distributed_60ghz_10dbm")
    results["sionna_60ghz_10dbm"] = result
    summaries["sionna_60ghz_10dbm"] = summarize(result, launch)

    output = {
        "controlled_setup": {
            "algorithm": "hybrid_communication_racer/global_cooperative",
            "scene": "warehouse_full_with_industrial_ap",
            "drone_count": 10,
            "duration_s": 200,
            "takeoff": "central_rack_two_sides_5_plus_5",
            "random_seed": 42,
        },
        "material_note_60ghz": (
            "P.2040 plywood coefficients epsilon_r=2.71 and sigma=0.33 S/m "
            "were explicitly extrapolated beyond their tabulated 40 GHz limit"
        ),
        **summaries,
        "sionna_60ghz_10dbm_minus_sionna_28ghz_20dbm": delta(
            summaries["sionna_60ghz_10dbm"], summaries["sionna_28ghz_20dbm"]
        ),
        "sionna_60ghz_10dbm_minus_perfect": delta(
            summaries["sionna_60ghz_10dbm"], summaries["perfect"]
        ),
    }
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    (args.frequency_suite / "frequency_power_comparison.json").write_text(
        rendered, encoding="utf-8"
    )
    write_plot(
        args.frequency_suite / "frequency_power_comparison.png", results, summaries
    )
    print(rendered, end="")


if __name__ == "__main__":
    main()
