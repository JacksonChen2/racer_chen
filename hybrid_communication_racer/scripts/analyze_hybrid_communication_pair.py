#!/usr/bin/env python3
"""Summarize paired ideal/pure-distributed hybrid RACER experiments."""

from __future__ import annotations

import argparse
import json
import re
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


CASES = {
    "perfect_communication": "ideal_perfect_communication",
    "pure_distributed_communication": "sionna_actual_communication",
}


def load_case(case_dir: Path) -> tuple[dict, str]:
    result_files = list(case_dir.glob("*_result.json"))
    if len(result_files) != 1:
        raise RuntimeError(f"expected one result JSON in {case_dir}, found {len(result_files)}")
    result = json.loads(result_files[0].read_text(encoding="utf-8"))
    launch_log = case_dir / "warehouse_full_distributed_launch.log"
    return result, launch_log.read_text(encoding="utf-8", errors="replace")


def joint_series(metrics: dict) -> tuple[np.ndarray, np.ndarray]:
    elapsed = float(metrics["elapsed"])
    count = len(metrics["mapping_coverage_per_agent"])
    times = np.arange(0.0, elapsed + 1.0e-9, 2.0)
    series = []
    history = metrics.get("mapping_coverage_history", [])
    for drone in range(count):
        records = sorted(
            (float(item["time_s"]), float(item["ratio"]))
            for item in history if int(item["drone_id"]) == drone
        )
        sample_t = np.asarray([0.0] + [item[0] for item in records] + [elapsed])
        sample_y = np.asarray(
            [0.0] + [item[1] for item in records]
            + [float(metrics["mapping_coverage_per_agent"][drone])]
        )
        series.append(np.interp(times, sample_t, np.maximum.accumulate(sample_y)))
    return times, np.max(np.vstack(series), axis=0) * 100.0


def count_prefix(text: str, prefix: str) -> int:
    return len(re.findall(re.escape(prefix), text))


def summarize(result: dict, launch_text: str) -> dict:
    metrics = result.get("metrics", {})
    communication = result.get("communication", {})
    stats = communication.get("statistics", {})
    evidence = result.get("algorithm_evidence", {})
    times, coverage = joint_series(metrics)
    slope_mask = (times >= 20.0) & (times <= 100.0)
    slope = float(np.polyfit(times[slope_mask], coverage[slope_mask], 1)[0])
    pool_sizes = [int(value) for value in re.findall(r"RACER_HYBRID_POOL[^\n]* pool=(\d+)", launch_text)]
    attempted = int(stats.get("attempted_packets", 0))
    delivered = int(stats.get("delivered_packets", 0))
    return {
        "elapsed_s": float(metrics.get("elapsed", 0.0)),
        "joint_coverage_percent": float(metrics.get("mapping_coverage_joint", 0.0)) * 100.0,
        "coverage_auc_percent_seconds": float(np.trapz(coverage, times)),
        "coverage_growth_20_100_pp_per_s": slope,
        "executed_drone_ids": result.get("executed_drone_ids", []),
        "process_crashes": int(evidence.get("process_crashes", 0)),
        "collision_events": int(metrics.get("collision_events", 0)),
        "total_path_length_m": float(sum(metrics.get("path_lengths", []))),
        "communication_mode": communication.get("mode"),
        "network_topology": communication.get("network_topology"),
        "sionna_ready": communication.get("sionna_ready"),
        "exact_link_samples": int(communication.get("exact_link_samples", 0)),
        "attempted_packets": attempted,
        "delivered_packets": delivered,
        "physical_delivery_ratio": delivered / max(1, attempted),
        "logical_delivery_ratio": float(stats.get("logical_delivery_ratio", 0.0)),
        "mean_end_to_end_delay_ms": float(stats.get("mean_end_to_end_delay_ms", 0.0)),
        "hybrid": {
            "pool_updates": len(pool_sizes),
            "pool_size_min": min(pool_sizes) if pool_sizes else None,
            "pool_size_max": max(pool_sizes) if pool_sizes else None,
            "target_events": count_prefix(launch_text, "RACER_HYBRID_TARGET"),
            "help_events": count_prefix(launch_text, "RACER_HYBRID_HELP"),
            "reassignment_events": count_prefix(launch_text, "RACER_HYBRID_REASSIGN"),
            "unreachable_reports": count_prefix(launch_text, "RACER_HYBRID_UNREACHABLE_REPORT"),
            "globally_unreachable_events": count_prefix(launch_text, "RACER_HYBRID_GLOBAL_UNREACHABLE"),
            "takeover_events": len(re.findall(r"RACER_HYBRID_REASSIGN[^\n]*takeover=1", launch_text)),
            "global_finish_events": len(re.findall(r"RACER_HYBRID_POOL[^\n]*global_finish=1", launch_text)),
        },
    }


def write_plot(path: Path, results: dict[str, dict], summaries: dict[str, dict]) -> None:
    labels = ("Perfect", "Sionna distributed")
    keys = ("perfect_communication", "pure_distributed_communication")
    colors = ("#1677ff", "#f59e0b")
    fig, axes = plt.subplots(2, 2, figsize=(13, 8), constrained_layout=True)
    fig.suptitle("Hybrid communication RACER: 10-UAV central takeoff, 200 s")

    for key, label, color in zip(keys, labels, colors):
        times, coverage = joint_series(results[key]["metrics"])
        axes[0, 0].plot(times, coverage, label=label, color=color, linewidth=2.2)
    axes[0, 0].set_title("Joint mapping coverage")
    axes[0, 0].set_xlabel("Simulation time (s)")
    axes[0, 0].set_ylabel("Coverage (%)")
    axes[0, 0].grid(alpha=0.25)
    axes[0, 0].legend(frameon=False)

    coverage_values = [summaries[key]["joint_coverage_percent"] for key in keys]
    axes[0, 1].bar(labels, coverage_values, color=colors)
    axes[0, 1].set_title("Coverage at 200 s")
    axes[0, 1].set_ylabel("Coverage (%)")
    for index, value in enumerate(coverage_values):
        axes[0, 1].text(index, value, f"{value:.2f}%", ha="center", va="bottom")

    delivery = [100.0 * summaries[key]["physical_delivery_ratio"] for key in keys]
    axes[1, 0].bar(labels, delivery, color=colors)
    axes[1, 0].set_title("Physical packet delivery")
    axes[1, 0].set_ylabel("Delivered / attempted (%)")
    axes[1, 0].set_ylim(0, 105)
    for index, value in enumerate(delivery):
        axes[1, 0].text(index, value, f"{value:.2f}%", ha="center", va="bottom")

    event_names = ("Targets", "HELP", "Reassign", "Takeover")
    x = np.arange(len(event_names))
    width = 0.36
    for index, (key, label, color) in enumerate(zip(keys, labels, colors)):
        hybrid = summaries[key]["hybrid"]
        values = [hybrid["target_events"], hybrid["help_events"],
                  hybrid["reassignment_events"], hybrid["takeover_events"]]
        axes[1, 1].bar(x + (index - 0.5) * width, values, width,
                       label=label, color=color)
    axes[1, 1].set_title("Hybrid assignment events")
    axes[1, 1].set_xticks(x, event_names)
    axes[1, 1].set_ylabel("Log event count")
    axes[1, 1].legend(frameon=False)

    fig.savefig(path, dpi=170)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("suite", type=Path)
    args = parser.parse_args()
    summaries = {}
    results = {}
    for label, directory in CASES.items():
        result, launch_text = load_case(args.suite / directory)
        results[label] = result
        summaries[label] = summarize(result, launch_text)
    perfect = summaries["perfect_communication"]
    distributed = summaries["pure_distributed_communication"]
    output = {
        "experiment": {
            "algorithm": "hybrid_communication_racer",
            "exploration_assignment_mode": "global_cooperative",
            "scene": "warehouse_full_with_industrial_ap",
            "drone_count": 10,
            "duration_s": 200,
            "takeoff": "central_rack_two_sides_5_plus_5",
            "distributed_definition": "Sionna RT, distributed topology, no AP relay, no lossless overrides, no retries",
            "hybrid_global_state_channel": "direct perfect global information in both cases",
        },
        **summaries,
        "distributed_minus_perfect": {
            "joint_coverage_percentage_points": (
                distributed["joint_coverage_percent"] - perfect["joint_coverage_percent"]
            ),
            "coverage_auc_percent_seconds": (
                distributed["coverage_auc_percent_seconds"] - perfect["coverage_auc_percent_seconds"]
            ),
            "coverage_growth_20_100_pp_per_s": (
                distributed["coverage_growth_20_100_pp_per_s"]
                - perfect["coverage_growth_20_100_pp_per_s"]
            ),
        },
        "integrity": {
            "both_reached_200s": all(item["elapsed_s"] >= 199.0 for item in summaries.values()),
            "all_10_uavs_executed": all(
                sorted(item["executed_drone_ids"]) == list(range(1, 11))
                for item in summaries.values()
            ),
            "no_process_crashes": all(item["process_crashes"] == 0 for item in summaries.values()),
            "hybrid_active_both_cases": all(
                item["hybrid"]["pool_updates"] > 0 for item in summaries.values()
            ),
        },
    }
    rendered = json.dumps(output, indent=2, sort_keys=True) + "\n"
    (args.suite / "hybrid_communication_comparison.json").write_text(rendered, encoding="utf-8")
    write_plot(args.suite / "hybrid_communication_comparison.png", results, summaries)
    print(rendered, end="")


if __name__ == "__main__":
    main()
