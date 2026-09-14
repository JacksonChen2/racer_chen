#!/usr/bin/env python3
"""Compare the fixed-MCS20 run with the prior controlled hybrid runs."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from analyze_hybrid_communication_pair import joint_series, summarize


CASES = (
    ("perfect", "Perfect communication", "#1677ff"),
    ("mcs14_28ghz_20dbm", "28 GHz / 20 dBm / MCS 14", "#22a06b"),
    ("mcs20_28ghz_20dbm", "28 GHz / 20 dBm / MCS 20", "#d4380d"),
    ("mcs14_60ghz_10dbm", "60 GHz / 10 dBm / MCS 14", "#f59e0b"),
)


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
            left["coverage_auc_percent_seconds"]
            - right["coverage_auc_percent_seconds"]
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


def add_radio_fields(summary: dict, result: dict) -> None:
    stats = result.get("communication", {}).get("statistics", {})
    phy = stats.get("phy", {})
    summary.update({
        "dropped_per": int(stats.get("dropped_per", 0)),
        "dropped_no_link": int(stats.get("dropped_no_link", 0)),
        "dropped_queue": int(stats.get("dropped_queue", 0)),
        "queued_packets": int(stats.get("queued_packets", 0)),
        "carrier_frequency_hz": float(phy.get("carrier_frequency_hz", 0.0)),
        "uav_tx_power_dbm": float(phy.get("uav_tx_power_dbm", 0.0)),
        "fixed_mcs_index": int(phy.get("fixed_mcs_index", -1)),
        "fixed_mcs_modulation": phy.get("fixed_mcs_modulation"),
        "fixed_mcs_code_rate": float(phy.get("fixed_mcs_code_rate", -1.0)),
        "mean_selected_initial_tbler": float(
            phy.get("mean_selected_initial_tbler", 0.0)
        ),
    })


def write_plot(path: Path, results: dict[str, dict], summaries: dict[str, dict]) -> None:
    keys = [case[0] for case in CASES]
    labels = [case[1] for case in CASES]
    colors = [case[2] for case in CASES]
    short = ("Perfect", "28G/20/M14", "28G/20/M20", "60G/10/M14")
    fig, axes = plt.subplots(2, 2, figsize=(15, 8.5), constrained_layout=True)
    fig.suptitle("Hybrid communication RACER: fixed-MCS comparison, 10 UAV, 200 s")
    for key, label, color in zip(keys, labels, colors):
        times, coverage = joint_series(results[key]["metrics"])
        axes[0, 0].plot(times, coverage, label=label, color=color, linewidth=2.1)
    axes[0, 0].set_title("Joint mapping coverage")
    axes[0, 0].set_xlabel("Simulation time (s)")
    axes[0, 0].set_ylabel("Coverage (%)")
    axes[0, 0].grid(alpha=0.25)
    axes[0, 0].legend(frameon=False, fontsize=9)

    x = np.arange(len(keys))
    coverage = [summaries[key]["joint_coverage_percent"] for key in keys]
    axes[0, 1].bar(x, coverage, color=colors)
    axes[0, 1].set_title("Coverage at 200 s")
    axes[0, 1].set_ylabel("Coverage (%)")
    axes[0, 1].set_xticks(x, short)
    for index, value in enumerate(coverage):
        axes[0, 1].text(index, value, f"{value:.2f}%", ha="center", va="bottom")

    delivery = [100.0 * summaries[key]["physical_delivery_ratio"] for key in keys]
    axes[1, 0].bar(x, delivery, color=colors)
    axes[1, 0].set_title("Physical packet delivery")
    axes[1, 0].set_ylabel("Delivered / attempted (%)")
    axes[1, 0].set_xticks(x, short)
    axes[1, 0].set_ylim(0, 105)
    for index, value in enumerate(delivery):
        axes[1, 0].text(index, value, f"{value:.2f}%", ha="center", va="bottom")

    delay = [summaries[key]["mean_end_to_end_delay_ms"] for key in keys]
    axes[1, 1].bar(x, delay, color=colors)
    axes[1, 1].set_title("Mean end-to-end delay")
    axes[1, 1].set_ylabel("Delay (ms)")
    axes[1, 1].set_xticks(x, short)
    for index, value in enumerate(delay):
        axes[1, 1].text(index, value, f"{value:.2f}", ha="center", va="bottom")
    fig.savefig(path, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("baseline_suite", type=Path)
    parser.add_argument("frequency_suite", type=Path)
    parser.add_argument("mcs20_suite", type=Path)
    args = parser.parse_args()
    directories = {
        "perfect": args.baseline_suite / "ideal_perfect_communication",
        "mcs14_28ghz_20dbm": args.baseline_suite / "sionna_actual_communication",
        "mcs20_28ghz_20dbm": (
            args.mcs20_suite / "sionna_distributed_28ghz_20dbm_mcs20"
        ),
        "mcs14_60ghz_10dbm": (
            args.frequency_suite / "sionna_distributed_60ghz_10dbm"
        ),
    }
    results = {}
    summaries = {}
    for key, directory in directories.items():
        result, launch = load_case(directory)
        results[key] = result
        summaries[key] = summarize(result, launch)
        add_radio_fields(summaries[key], result)

    output = {
        "controlled_setup": {
            "algorithm": "hybrid_communication_racer/global_cooperative",
            "scene": "warehouse_full_with_industrial_ap",
            "drone_count": 10,
            "duration_s": 200,
            "takeoff": "central_rack_two_sides_5_plus_5",
            "random_seed": 42,
            "sionna_topology": "distributed, no AP relay, no retries",
            "hybrid_global_state_channel": "direct_perfect_global_information",
        },
        **summaries,
        "mcs20_minus_mcs14_at_28ghz_20dbm": delta(
            summaries["mcs20_28ghz_20dbm"], summaries["mcs14_28ghz_20dbm"]
        ),
        "mcs20_28ghz_20dbm_minus_perfect": delta(
            summaries["mcs20_28ghz_20dbm"], summaries["perfect"]
        ),
        "integrity": {
            "all_reached_200s": all(
                item["elapsed_s"] >= 199.0 for item in summaries.values()
            ),
            "all_10_uavs_executed": all(
                sorted(item["executed_drone_ids"]) == list(range(1, 11))
                for item in summaries.values()
            ),
            "no_process_crashes": all(
                item["process_crashes"] == 0 for item in summaries.values()
            ),
        },
    }
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    (args.mcs20_suite / "mcs20_performance_comparison.json").write_text(
        rendered, encoding="utf-8"
    )
    write_plot(
        args.mcs20_suite / "mcs20_performance_comparison.png", results, summaries
    )
    print(rendered, end="")


if __name__ == "__main__":
    main()
