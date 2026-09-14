#!/usr/bin/env python3
"""Summarize a short original/global_cooperative RACER A/B run."""

import argparse
import csv
import itertools
import json
import math
from pathlib import Path
import re


def result_file(case_dir: Path) -> Path:
    matches = sorted(case_dir.glob("*_result.json"))
    if len(matches) != 1:
        raise RuntimeError(f"expected one result JSON under {case_dir}, found {len(matches)}")
    return matches[0]


def launch_log(case_dir: Path) -> Path:
    matches = sorted(case_dir.glob("*_launch.log"))
    if len(matches) != 1:
        raise RuntimeError(f"expected one launch log under {case_dir}, found {len(matches)}")
    return matches[0]


def distances(points):
    values = []
    for first, second in itertools.combinations(points, 2):
        values.append(math.dist(first, second))
    return values


def coverage_curve(history):
    latest = {}
    curve = []
    for row in sorted(history, key=lambda item: item["time_s"]):
        latest[int(row["drone_id"])] = float(row["ratio"])
        curve.append((float(row["time_s"]), max(latest.values())))
    return curve


def coverage_at(curve, stamp):
    values = [value for time_s, value in curve if time_s <= stamp]
    return values[-1] if values else 0.0


def slope(curve, end_s=15.0):
    points = [(time_s, value) for time_s, value in curve if time_s <= end_s]
    if len(points) < 2:
        return 0.0
    mean_t = sum(point[0] for point in points) / len(points)
    mean_y = sum(point[1] for point in points) / len(points)
    denominator = sum((point[0] - mean_t) ** 2 for point in points)
    if denominator <= 0.0:
        return 0.0
    return sum((time_s - mean_t) * (value - mean_y) for time_s, value in points) / denominator


def initial_hybrid_targets(log_text):
    pattern = re.compile(
        r"RACER_HYBRID_TARGET uav=(\d+) grid=(-?\d+) "
        r"target=\[([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\]"
    )
    targets = {}
    for line in log_text.splitlines():
        if "RACER_HYBRID_POOL" in line:
            break
        match = pattern.search(line)
        if match:
            targets[int(match.group(1))] = tuple(float(match.group(i)) for i in range(3, 6))
    return targets


def premature_finish(log_text):
    first_finish = None
    first_global_finish = None
    for index, line in enumerate(log_text.splitlines()):
        if first_global_finish is None and "RACER_HYBRID_POOL" in line and "global_finish=1" in line:
            first_global_finish = index
        if first_finish is None and ("state: FINISH" in line or "finish exploration" in line):
            first_finish = index
    return first_finish is not None and (first_global_finish is None or first_finish < first_global_finish)


def summarize(case_dir: Path):
    result = json.loads(result_file(case_dir).read_text())
    metrics = result["metrics"]
    log_text = launch_log(case_dir).read_text(errors="replace")
    curve = coverage_curve(metrics.get("mapping_coverage_history", []))
    elapsed = float(metrics.get("elapsed", 0.0))
    final_coverage = metrics.get("mapping_coverage_joint")
    if final_coverage is not None and (not curve or curve[-1][0] < elapsed):
        curve.append((elapsed, float(final_coverage)))
    trajectories = metrics.get("trajectory_history", [])
    final_positions = trajectories[-1]["positions"] if trajectories else metrics.get("positions", [])
    endpoints = distances(final_positions)
    targets = initial_hybrid_targets(log_text)
    target_separations = distances(list(targets.values()))
    latest_contribution = {}
    maximum_idle = {}
    stats_pattern = re.compile(
        r"RACER_HYBRID_STATS uav=(\d+)[^\n]+idle=([-+0-9.eE]+) "
        r"coverage_contribution=([-+0-9.eE]+)"
    )
    for match in stats_pattern.finditer(log_text):
        drone_id = int(match.group(1))
        maximum_idle[drone_id] = max(maximum_idle.get(drone_id, 0.0), float(match.group(2)))
        latest_contribution[drone_id] = float(match.group(3))
    return {
        "result_file": str(result_file(case_dir)),
        "elapsed_s": metrics.get("elapsed"),
        "final_joint_coverage": metrics.get("mapping_coverage_joint"),
        "coverage_at_10s": coverage_at(curve, 10.0),
        "coverage_slope_0_15_per_s": slope(curve),
        "executed_uavs": len(result.get("executed_drone_ids", [])),
        "path_length_sum_m": sum(metrics.get("path_lengths", [])),
        "final_min_pair_distance_m": min(endpoints) if endpoints else None,
        "final_mean_pair_distance_m": sum(endpoints) / len(endpoints) if endpoints else None,
        "initial_joint_target_count": len(targets),
        "initial_target_min_separation_m": min(target_separations) if target_separations else None,
        "initial_target_pairs_within_3m": sum(value < 3.0 for value in target_separations),
        "help_mode_events": len(re.findall(r"RACER_HYBRID_HELP", log_text)),
        "reassignment_events": len(re.findall(r"RACER_HYBRID_REASSIGN", log_text)),
        "takeover_events": len(re.findall(r"RACER_HYBRID_REASSIGN[^\n]+takeover=1", log_text)),
        "unreachable_reports": len(re.findall(r"RACER_HYBRID_UNREACHABLE_REPORT", log_text)),
        "coverage_contribution_by_uav": latest_contribution,
        "maximum_logged_idle_s_by_uav": maximum_idle,
        "premature_finish_before_global_finish": premature_finish(log_text),
        "process_crashes": result.get("algorithm_evidence", {}).get("process_crashes", 0),
        "curve": curve,
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("original", type=Path)
    parser.add_argument("global_cooperative", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--csv", type=Path, required=True)
    args = parser.parse_args()
    original = summarize(args.original)
    cooperative = summarize(args.global_cooperative)
    summary = {
        "original": {key: value for key, value in original.items() if key != "curve"},
        "global_cooperative": {key: value for key, value in cooperative.items() if key != "curve"},
        "delta_global_minus_original": {
            "final_joint_coverage": cooperative["final_joint_coverage"] - original["final_joint_coverage"],
            "coverage_at_10s": cooperative["coverage_at_10s"] - original["coverage_at_10s"],
            "coverage_slope_0_15_per_s": cooperative["coverage_slope_0_15_per_s"] - original["coverage_slope_0_15_per_s"],
        },
    }
    args.output.write_text(json.dumps(summary, indent=2, sort_keys=True) + "\n")

    max_time = int(max(
        original["curve"][-1][0] if original["curve"] else 0,
        cooperative["curve"][-1][0] if cooperative["curve"] else 0,
    ))
    with args.csv.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(["time_s", "original_joint_coverage", "global_cooperative_joint_coverage"])
        for stamp in range(0, max_time + 1):
            writer.writerow([stamp, coverage_at(original["curve"], stamp), coverage_at(cooperative["curve"], stamp)])
    print(json.dumps(summary, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
