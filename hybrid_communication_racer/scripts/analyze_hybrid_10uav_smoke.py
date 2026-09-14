#!/usr/bin/env python3
"""Compare a paired original/global-cooperative 10-UAV smoke run."""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path
from statistics import fmean


def load_case(case_dir: Path) -> tuple[dict, str]:
    result_files = sorted(case_dir.glob("*_result.json"))
    if len(result_files) != 1:
        raise RuntimeError(
            f"expected one result JSON in {case_dir}, found {len(result_files)}"
        )
    result = json.loads(result_files[0].read_text(encoding="utf-8"))
    launch_files = sorted(case_dir.glob("*_launch.log"))
    if len(launch_files) != 1:
        raise RuntimeError(
            f"expected one launch log in {case_dir}, found {len(launch_files)}"
        )
    return result, launch_files[0].read_text(encoding="utf-8", errors="replace")


def interpolate(records: list[tuple[float, float]], time_s: float, final: float) -> float:
    points = [(0.0, 0.0)] + sorted(records) + [(time_s, final)]
    if not points:
        return 0.0
    if time_s <= points[0][0]:
        return points[0][1]
    previous_time, previous_value = points[0]
    previous_value = max(0.0, previous_value)
    for next_time, next_value in points[1:]:
        next_value = max(previous_value, next_value)
        if time_s <= next_time:
            interval = next_time - previous_time
            if interval <= 1.0e-9:
                return next_value
            alpha = (time_s - previous_time) / interval
            return previous_value + alpha * (next_value - previous_value)
        previous_time, previous_value = next_time, next_value
    return previous_value


def joint_coverage(result: dict, times: list[float]) -> list[float]:
    metrics = result["metrics"]
    drone_count = int(result["drone_count"])
    history = metrics.get("mapping_coverage_history", [])
    finals = metrics.get("mapping_coverage_per_agent", [])
    curves: list[list[float]] = []
    for drone_id in range(drone_count):
        records = [
            (float(item["time_s"]), float(item["ratio"]))
            for item in history
            if int(item["drone_id"]) == drone_id
        ]
        final = float(finals[drone_id] or 0.0) if drone_id < len(finals) else 0.0
        curves.append([interpolate(records, time_s, final) for time_s in times])
    if not curves:
        return [0.0 for _ in times]
    return [100.0 * max(curve[index] for curve in curves) for index in range(len(times))]


def trajectory_dispersion(result: dict, assessment_time_s: float) -> dict[str, float | int]:
    history = result["metrics"].get("trajectory_history", [])
    if history:
        sample = min(history, key=lambda item: abs(float(item["time_s"]) - assessment_time_s))
        positions = sample["positions"]
        actual_time = float(sample["time_s"])
    else:
        positions = result["metrics"].get("positions", [])
        actual_time = float(result["metrics"].get("elapsed", 0.0))
    distances = []
    for left in range(len(positions)):
        for right in range(left + 1, len(positions)):
            distances.append(
                math.dist(
                    [float(value) for value in positions[left]],
                    [float(value) for value in positions[right]],
                )
            )
    return {
        "assessment_time_s": actual_time,
        "mean_pair_distance_m": fmean(distances) if distances else 0.0,
        "minimum_pair_distance_m": min(distances) if distances else 0.0,
        "pairs_within_3m": sum(value < 3.0 for value in distances),
        "pairs_within_6m": sum(value < 6.0 for value in distances),
    }


def hybrid_log_metrics(launch_text: str) -> dict[str, object]:
    target_pattern = re.compile(
        r"RACER_HYBRID_TARGET uav=(\d+) grid=(-?\d+) "
        r"target=\[([-+0-9.eE]+),([-+0-9.eE]+),([-+0-9.eE]+)\] "
        r"utility=([-+0-9.eE]+) gain=([-+0-9.eE]+) "
        r"travel_time=([-+0-9.eE]+) overlap=([-+0-9.eE]+) "
        r"R=([-+0-9.eE]+) takeover=(\d+)"
    )
    batches: list[list[tuple[float, float, float]]] = []
    current_batch: list[tuple[float, float, float]] = []
    target_events = []
    finish_confirmed = False
    premature_finish = []
    for line_number, line in enumerate(launch_text.splitlines(), start=1):
        target_match = target_pattern.search(line)
        if target_match:
            current_batch.append(
                (
                    float(target_match.group(3)),
                    float(target_match.group(4)),
                    float(target_match.group(5)),
                )
            )
            target_events.append(
                {
                    "uav": int(target_match.group(1)),
                    "grid": int(target_match.group(2)),
                    "utility": float(target_match.group(6)),
                    "gain": float(target_match.group(7)),
                    "travel_time_s": float(target_match.group(8)),
                    "overlap_penalty": float(target_match.group(9)),
                    "distance_limit_m": float(target_match.group(10)),
                    "takeover": bool(int(target_match.group(11))),
                }
            )
        if "RACER_HYBRID_POOL" in line:
            if current_batch:
                batches.append(current_batch)
                current_batch = []
            if "global_finish=1" in line:
                finish_confirmed = True
        if re.search(r"(?:to FINISH|state: FINISH|finish exploration)", line):
            if not finish_confirmed:
                premature_finish.append(line_number)
    if current_batch:
        batches.append(current_batch)

    minimum_separations = []
    adjacent_pairs = 0
    pair_count = 0
    for batch in batches:
        distances = [
            math.dist(batch[left], batch[right])
            for left in range(len(batch))
            for right in range(left + 1, len(batch))
        ]
        if distances:
            minimum_separations.append(min(distances))
            adjacent_pairs += sum(distance < 3.0 for distance in distances)
            pair_count += len(distances)

    pool_sizes = [
        int(value)
        for value in re.findall(r"RACER_HYBRID_POOL[^\n]* pool=(\d+)", launch_text)
    ]
    return {
        "pool_updates": len(pool_sizes),
        "pool_size_min": min(pool_sizes) if pool_sizes else None,
        "pool_size_max": max(pool_sizes) if pool_sizes else None,
        "target_events": len(target_events),
        "unique_target_grids": len({item["grid"] for item in target_events}),
        "mean_target_gain": (
            fmean(item["gain"] for item in target_events) if target_events else None
        ),
        "mean_target_travel_time_s": (
            fmean(item["travel_time_s"] for item in target_events)
            if target_events
            else None
        ),
        "mean_target_overlap_penalty": (
            fmean(item["overlap_penalty"] for item in target_events)
            if target_events
            else None
        ),
        "mean_batch_min_target_separation_m": (
            fmean(minimum_separations) if minimum_separations else None
        ),
        "adjacent_target_pair_fraction": (
            adjacent_pairs / pair_count if pair_count else None
        ),
        "help_events": launch_text.count("RACER_HYBRID_HELP"),
        "reassignment_events": launch_text.count("RACER_HYBRID_REASSIGN"),
        "takeover_events": sum(item["takeover"] for item in target_events),
        "unreachable_reports": launch_text.count("RACER_HYBRID_UNREACHABLE_REPORT"),
        "globally_unreachable_events": launch_text.count(
            "RACER_HYBRID_GLOBAL_UNREACHABLE"
        ),
        "premature_finish_before_global_confirmation": len(premature_finish),
        "premature_finish_line_numbers": premature_finish,
    }


def summarize(result: dict, launch_text: str) -> dict[str, object]:
    metrics = result["metrics"]
    elapsed = float(metrics["elapsed"])
    times = [float(value) for value in range(math.floor(elapsed) + 1)]
    if not times or times[-1] < elapsed:
        times.append(elapsed)
    coverage = joint_coverage(result, times)
    early_start = min(5.0, elapsed * 0.25)
    early_end = min(20.0, elapsed * 0.75)
    if early_end <= early_start:
        early_start, early_end = 0.0, elapsed
    early_values = joint_coverage(result, [early_start, early_end])
    slope = (
        (early_values[1] - early_values[0]) / (early_end - early_start)
        if early_end > early_start
        else 0.0
    )
    auc = sum(
        0.5 * (coverage[index - 1] + coverage[index]) * (times[index] - times[index - 1])
        for index in range(1, len(times))
    )
    evidence = result.get("algorithm_evidence", {})
    communication = result.get("communication", {}).get("statistics", {})
    return {
        "elapsed_s": elapsed,
        "joint_coverage_percent": coverage[-1] if coverage else 0.0,
        "early_window_s": [early_start, early_end],
        "early_coverage_growth_pp_per_s": slope,
        "coverage_auc_percent_seconds": auc,
        "dispersion": trajectory_dispersion(result, min(20.0, elapsed)),
        "executed_drone_ids": result.get("executed_drone_ids", []),
        "process_crashes": int(evidence.get("process_crashes", 0)),
        "ideal_attempted_packets": int(communication.get("attempted_packets", 0)),
        "ideal_delivered_packets": int(communication.get("delivered_packets", 0)),
        "hybrid": hybrid_log_metrics(launch_text),
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("suite", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    original_result, original_log = load_case(args.suite / "original")
    hybrid_result, hybrid_log = load_case(args.suite / "global_cooperative")
    original = summarize(original_result, original_log)
    hybrid = summarize(hybrid_result, hybrid_log)
    comparison = {
        "suite": str(args.suite.resolve()),
        "original": original,
        "global_cooperative": hybrid,
        "difference_global_minus_original": {
            "joint_coverage_percentage_points": (
                hybrid["joint_coverage_percent"] - original["joint_coverage_percent"]
            ),
            "early_growth_pp_per_s": (
                hybrid["early_coverage_growth_pp_per_s"]
                - original["early_coverage_growth_pp_per_s"]
            ),
            "coverage_auc_percent_seconds": (
                hybrid["coverage_auc_percent_seconds"]
                - original["coverage_auc_percent_seconds"]
            ),
            "mean_pair_distance_m": (
                hybrid["dispersion"]["mean_pair_distance_m"]
                - original["dispersion"]["mean_pair_distance_m"]
            ),
            "pairs_within_6m": (
                hybrid["dispersion"]["pairs_within_6m"]
                - original["dispersion"]["pairs_within_6m"]
            ),
        },
        "smoke_checks": {
            "all_10_uavs_executed_original": len(original["executed_drone_ids"]) == 10,
            "all_10_uavs_executed_global": len(hybrid["executed_drone_ids"]) == 10,
            "no_process_crash": (
                original["process_crashes"] == 0 and hybrid["process_crashes"] == 0
            ),
            "global_no_premature_finish": (
                hybrid["hybrid"]["premature_finish_before_global_confirmation"] == 0
            ),
            "global_joint_targets_observed": hybrid["hybrid"]["target_events"] > 0,
            "global_early_slope_above_original": (
                hybrid["early_coverage_growth_pp_per_s"]
                > original["early_coverage_growth_pp_per_s"]
            ),
            "global_observed_final_coverage_above_original": (
                hybrid["joint_coverage_percent"] > original["joint_coverage_percent"]
            ),
        },
    }
    rendered = json.dumps(comparison, indent=2, sort_keys=True) + "\n"
    output = args.output or args.suite / "comparison.json"
    output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")


if __name__ == "__main__":
    main()
