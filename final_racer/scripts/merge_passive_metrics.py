#!/usr/bin/env python3
"""Merge the passive 10 Hz task-quality stream into a RACER result JSON."""

import json
import math
from pathlib import Path
import sys


def main() -> None:
    if len(sys.argv) != 6:
        raise SystemExit(
            "usage: merge_passive_metrics.py RESULT HISTORY GT PERIOD_S DURATION_S"
        )
    result_path = Path(sys.argv[1])
    history_path = Path(sys.argv[2])
    gt_path = Path(sys.argv[3])
    sample_period_s = float(sys.argv[4])
    duration_s = float(sys.argv[5])
    if not result_path.is_file() or not history_path.is_file() or not gt_path.is_file():
        raise SystemExit("PASSIVE_METRIC_ERROR missing result, history, or GT file")

    deduplicated = {}
    for line in history_path.read_text().splitlines():
        if not line.strip():
            continue
        item = json.loads(line)
        values = (
            float(item["time_s"]),
            float(item["redundant_exploration_ratio"]),
            float(item["bs_global_map_iou"]),
        )
        if not all(math.isfinite(value) for value in values):
            raise SystemExit("PASSIVE_METRIC_ERROR history contains NaN or Inf")
        stamp = round(values[0], 9)
        if stamp <= duration_s + sample_period_s:
            deduplicated[stamp] = item
    history = [deduplicated[key] for key in sorted(deduplicated)]
    if not history:
        raise SystemExit("PASSIVE_METRIC_ERROR history is empty")

    result = json.loads(result_path.read_text())
    statistics = result.setdefault("communication", {}).setdefault("statistics", {})
    statistics["task_quality_history"] = history
    statistics["task_metric_observer_mode"] = "async_passive_final_racer_overlay"
    statistics["task_metric_sample_period_s"] = sample_period_s
    statistics["ground_truth_occupied_voxels"] = sum(
        1 for line in gt_path.read_text().splitlines() if line.strip()
    )
    statistics["redundant_exploration_ratio"] = float(
        history[-1]["redundant_exploration_ratio"]
    )
    statistics["bs_global_map_iou"] = float(history[-1]["bs_global_map_iou"])
    temporary = result_path.with_suffix(result_path.suffix + ".tmp")
    temporary.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n")
    temporary.replace(result_path)
    print(
        "PASSIVE_METRIC_MERGE_OK"
        f" samples={len(history)} first={history[0]['time_s']}"
        f" last={history[-1]['time_s']}"
    )


if __name__ == "__main__":
    main()
