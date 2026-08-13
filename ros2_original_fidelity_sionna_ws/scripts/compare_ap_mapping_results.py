#!/usr/bin/env python3
"""Create machine-readable and human-readable reports for the AP A/B run."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Tuple


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--distributed", type=Path, required=True)
    parser.add_argument("--ap-assisted", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--sample-period", type=float, default=10.0)
    return parser.parse_args()


def finite(value: Any) -> Optional[float]:
    try:
        number = float(value)
    except (TypeError, ValueError):
        return None
    return number if math.isfinite(number) else None


def mean(values: Iterable[Any]) -> Optional[float]:
    numbers = [number for value in values if (number := finite(value)) is not None]
    return sum(numbers) / len(numbers) if numbers else None


def ratio(numerator: Any, denominator: Any) -> Optional[float]:
    top = finite(numerator)
    bottom = finite(denominator)
    if top is None or bottom is None or bottom == 0.0:
        return None
    return top / bottom


def delta(assisted: Any, distributed: Any) -> Dict[str, Optional[float]]:
    ap_value = finite(assisted)
    base_value = finite(distributed)
    if ap_value is None or base_value is None:
        return {"absolute": None, "relative_percent": None}
    absolute = ap_value - base_value
    relative = None if base_value == 0.0 else 100.0 * absolute / base_value
    return {"absolute": absolute, "relative_percent": relative}


def coverage_curve(result: dict) -> List[Tuple[float, float]]:
    records = result.get("metrics", {}).get("mapping_coverage_history", [])
    latest: Dict[int, float] = {}
    curve: List[Tuple[float, float]] = []
    for record in sorted(records, key=lambda item: float(item.get("time_s", 0.0))):
        drone_id = int(record["drone_id"])
        value = finite(record.get("ratio"))
        stamp = finite(record.get("time_s"))
        if value is None or stamp is None:
            continue
        latest[drone_id] = value
        curve.append((stamp, max(latest.values())))
    final_value = finite(result.get("metrics", {}).get("mapping_coverage_joint"))
    elapsed = finite(result.get("metrics", {}).get("elapsed"))
    if final_value is not None and elapsed is not None:
        curve.append((elapsed, final_value))
    curve.sort()
    return curve


def sample_hold(curve: List[Tuple[float, float]], stamp: float) -> Optional[float]:
    value = None
    for curve_stamp, curve_value in curve:
        if curve_stamp > stamp + 1.0e-9:
            break
        value = curve_value
    return value


def normalized_auc(curve: List[Tuple[float, float]], elapsed: float) -> Optional[float]:
    if elapsed <= 0.0 or not curve:
        return None
    area = 0.0
    previous_stamp = 0.0
    previous_value = 0.0
    for stamp, value in curve:
        clipped = min(max(stamp, 0.0), elapsed)
        if clipped < previous_stamp:
            continue
        area += (clipped - previous_stamp) * previous_value
        previous_stamp = clipped
        previous_value = value
        if clipped >= elapsed:
            break
    if previous_stamp < elapsed:
        area += (elapsed - previous_stamp) * previous_value
    return area / elapsed


def summarize(result: dict) -> dict:
    metrics = result.get("metrics", {})
    communication = result.get("communication", {})
    statistics = communication.get("statistics", {})
    coverage = finite(metrics.get("mapping_coverage_joint"))
    path_lengths = metrics.get("path_lengths", [])
    total_path = sum(
        value for item in path_lengths if (value := finite(item)) is not None
    )
    elapsed = finite(metrics.get("elapsed")) or 0.0
    curve = coverage_curve(result)
    return {
        "passed": bool(result.get("passed")),
        "network_topology": communication.get("network_topology"),
        "stop_reason": metrics.get("stop_reason"),
        "elapsed_s": elapsed,
        "normal_completion": bool(
            result.get("acceptance", {}).get("normal_completion")
        ),
        "finished_agents": len(result.get("finished_drone_ids", [])),
        "returned_agents": len(result.get("returned_drone_ids", [])),
        "mapping_coverage_joint": coverage,
        "mapping_coverage_mean_agent": mean(
            metrics.get("mapping_coverage_per_agent", [])
        ),
        "mapping_coverage_auc": normalized_auc(curve, elapsed),
        "total_path_length_m": total_path,
        "coverage_per_path_meter": ratio(coverage, total_path),
        "collision_events": metrics.get("collision_events"),
        "min_inter_drone_m": metrics.get("min_inter_drone"),
        "min_obstacle_clearance_m": metrics.get("min_obstacle_clearance"),
        "logical_delivery_ratio": statistics.get("logical_delivery_ratio"),
        "mean_end_to_end_delay_ms": statistics.get("mean_end_to_end_delay_ms"),
        "logical_attempted_packets": statistics.get("logical_attempted_packets"),
        "logical_delivered_packets": statistics.get("logical_delivered_packets"),
        "physical_attempted_packets": statistics.get("attempted_packets"),
        "physical_delivered_packets": statistics.get("delivered_packets"),
        "dropped_no_link": statistics.get("dropped_no_link"),
        "dropped_per": statistics.get("dropped_per"),
        "dropped_queue": statistics.get("dropped_queue"),
        "dropped_ttl": statistics.get("dropped_ttl"),
        "ap_global_updates_received": statistics.get("ap_global_updates_received", 0),
        "ap_selective_forwards_enqueued": statistics.get(
            "ap_selective_forwards_enqueued", 0
        ),
        "ap_relay_wins": statistics.get("ap_relay_wins", 0),
        "duplicates_suppressed": statistics.get("duplicates_suppressed", 0),
        "sionna_exact_samples": communication.get("exact_link_samples"),
    }


def paired_checks(distributed: dict, assisted: dict) -> dict:
    checks = {
        "same_algorithm": distributed.get("algorithm") == assisted.get("algorithm"),
        "same_scene": distributed.get("scene") == assisted.get("scene"),
        "same_sionna_scene": distributed.get("sionna_scene")
        == assisted.get("sionna_scene"),
        "same_vehicle": distributed.get("vehicle") == assisted.get("vehicle"),
        "same_drone_count": distributed.get("drone_count")
        == assisted.get("drone_count"),
        "both_sionna_rt_active": all(
            result.get("acceptance", {}).get("sionna_rt_active", False)
            for result in (distributed, assisted)
        ),
        "distributed_topology_selected": distributed.get("communication", {}).get(
            "network_topology"
        )
        == "distributed",
        "ap_assisted_topology_selected": assisted.get("communication", {}).get(
            "network_topology"
        )
        == "ap_assisted",
    }
    checks["valid_paired_experiment"] = all(checks.values())
    return checks


def markdown_value(value: Any) -> str:
    if isinstance(value, bool):
        return "yes" if value else "no"
    number = finite(value)
    if number is not None:
        return f"{number:.6g}"
    return "n/a" if value is None else str(value)


def main() -> None:
    args = arguments()
    if args.sample_period <= 0.0:
        raise ValueError("sample period must be positive")
    distributed = json.loads(args.distributed.read_text())
    assisted = json.loads(args.ap_assisted.read_text())
    output_dir = args.output_dir.resolve()
    output_dir.mkdir(parents=True, exist_ok=True)

    base_summary = summarize(distributed)
    ap_summary = summarize(assisted)
    comparison = {
        "distributed_result": str(args.distributed.resolve()),
        "ap_assisted_result": str(args.ap_assisted.resolve()),
        "paired_checks": paired_checks(distributed, assisted),
        "distributed": base_summary,
        "ap_assisted": ap_summary,
        "delta_ap_minus_distributed": {
            name: delta(ap_summary.get(name), base_summary.get(name))
            for name in (
                "mapping_coverage_joint",
                "mapping_coverage_mean_agent",
                "mapping_coverage_auc",
                "elapsed_s",
                "total_path_length_m",
                "coverage_per_path_meter",
                "logical_delivery_ratio",
                "mean_end_to_end_delay_ms",
            )
        },
    }
    (output_dir / "comparison.json").write_text(
        json.dumps(comparison, indent=2, sort_keys=True) + "\n"
    )

    curves = {
        "distributed": coverage_curve(distributed),
        "ap_assisted": coverage_curve(assisted),
    }
    max_elapsed = max(base_summary["elapsed_s"], ap_summary["elapsed_s"])
    stamps = []
    stamp = 0.0
    while stamp <= max_elapsed + 1.0e-9:
        stamps.append(stamp)
        stamp += args.sample_period
    if not stamps or stamps[-1] < max_elapsed:
        stamps.append(max_elapsed)
    with (output_dir / "coverage_curve.csv").open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            ["time_s", "distributed_joint_coverage", "ap_assisted_joint_coverage"]
        )
        for stamp in stamps:
            writer.writerow(
                [
                    f"{stamp:.6f}",
                    sample_hold(curves["distributed"], stamp),
                    sample_hold(curves["ap_assisted"], stamp),
                ]
            )

    rows = (
        ("任务完成", "normal_completion"),
        ("结束原因", "stop_reason"),
        ("仿真时长 (s)", "elapsed_s"),
        ("全局建图覆盖率", "mapping_coverage_joint"),
        ("平均单机覆盖率", "mapping_coverage_mean_agent"),
        ("覆盖率曲线 AUC", "mapping_coverage_auc"),
        ("总航程 (m)", "total_path_length_m"),
        ("每米覆盖效率", "coverage_per_path_meter"),
        ("逻辑包送达率", "logical_delivery_ratio"),
        ("端到端时延 (ms)", "mean_end_to_end_delay_ms"),
        ("碰撞次数", "collision_events"),
        ("AP 收到全局更新", "ap_global_updates_received"),
        ("AP 选择性转发", "ap_selective_forwards_enqueued"),
        ("AP 成功补发", "ap_relay_wins"),
    )
    report = [
        "# Warehouse Simple：纯分布式与 AP 辅助 RACER 对比",
        "",
        f"配对实验有效：{markdown_value(comparison['paired_checks']['valid_paired_experiment'])}",
        "",
        "| 指标 | 纯分布式 | AP 辅助 |",
        "|---|---:|---:|",
    ]
    for label, key in rows:
        report.append(
            f"| {label} | {markdown_value(base_summary.get(key))} | "
            f"{markdown_value(ap_summary.get(key))} |"
        )
    report.extend(
        [
            "",
            "两组都使用同一个含 AP 实体的 Warehouse Simple 场景和 Sionna RT 环境；",
            "纯分布式组仅关闭 AP 无线中继，UAV 间链路模型、随机种子、带宽和 RACER 参数保持一致。",
            "",
        ]
    )
    (output_dir / "comparison.md").write_text("\n".join(report))
    print(json.dumps(comparison, indent=2, sort_keys=True))


if __name__ == "__main__":
    main()
