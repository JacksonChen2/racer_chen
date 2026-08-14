#!/usr/bin/env python3
"""Plot joint coverage for the paired BS and distributed formal runs."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
from typing import Dict, List, Tuple

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.ticker import PercentFormatter


Curve = List[Tuple[float, float]]


def arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--bs", type=Path, required=True)
    parser.add_argument("--distributed", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--deadline", type=float, default=900.0)
    parser.add_argument(
        "--title",
        default="Warehouse 多无人机建图：联合覆盖率随运行时间变化",
    )
    return parser.parse_args()


def load(path: Path) -> dict:
    return json.loads(path.read_text())


def coverage_curve(result: dict, deadline: float) -> Curve:
    records = result["metrics"]["mapping_coverage_history"]
    latest: Dict[int, float] = {}
    curve: Curve = [(0.0, 0.0)]
    for record in sorted(records, key=lambda item: float(item["time_s"])):
        latest[int(record["drone_id"])] = 100.0 * float(record["ratio"])
        curve.append((min(float(record["time_s"]), deadline), max(latest.values())))

    elapsed = min(float(result["metrics"]["elapsed"]), deadline)
    final_coverage = 100.0 * float(result["metrics"]["mapping_coverage_joint"])
    curve.append((elapsed, final_coverage))
    curve.sort()
    return curve


def sample_hold(curve: Curve, stamp: float) -> float:
    value = 0.0
    for curve_stamp, curve_value in curve:
        if curve_stamp > stamp + 1.0e-9:
            break
        value = curve_value
    return value


def draw_curve(axis, curve: Curve, color: str, label: str, linewidth: float = 2.4):
    x, y = zip(*curve)
    axis.step(x, y, where="post", color=color, linewidth=linewidth, label=label)


def threshold_time(curve: Curve, threshold: float) -> float | None:
    for stamp, value in curve:
        if value >= threshold:
            return stamp
    return None


def time_weighted_coverage(curve: Curve, deadline: float) -> float:
    points = sorted(curve)
    value = 0.0
    previous = 0.0
    area = 0.0
    for stamp, next_value in points:
        stamp = min(max(stamp, 0.0), deadline)
        if stamp > previous:
            area += value * (stamp - previous)
            previous = stamp
        value = next_value
        if previous >= deadline:
            break
    if previous < deadline:
        area += value * (deadline - previous)
    return area / deadline if deadline > 0.0 else value


def fmt_time(value: float | None) -> str:
    return "未达到" if value is None else f"{value:.2f} s"


def main() -> None:
    args = arguments()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    bs = load(args.bs)
    distributed = load(args.distributed)
    bs_curve = coverage_curve(bs, args.deadline)
    distributed_curve = coverage_curve(distributed, args.deadline)

    bs_elapsed = float(bs["metrics"]["elapsed"])
    bs_plot_elapsed = min(bs_elapsed, args.deadline)
    bs_coverage = 100.0 * float(bs["metrics"]["mapping_coverage_joint"])
    distributed_elapsed = float(distributed["metrics"]["elapsed"])
    distributed_plot_elapsed = min(distributed_elapsed, args.deadline)
    distributed_coverage = 100.0 * float(
        distributed["metrics"]["mapping_coverage_joint"]
    )
    bs_finished = len(bs["finished_drone_ids"])
    bs_drone_count = int(bs["drone_count"])
    distributed_finished = len(distributed["finished_drone_ids"])
    distributed_drone_count = int(distributed["drone_count"])
    bs_stop_reason = str(bs["metrics"].get("stop_reason", "unknown"))
    distributed_stop_reason = str(
        distributed["metrics"].get("stop_reason", "unknown")
    )

    plt.rcParams.update(
        {
            "font.family": "sans-serif",
            "font.sans-serif": ["Noto Sans CJK JP", "DejaVu Sans"],
            "axes.unicode_minus": False,
            "font.size": 11,
        }
    )
    bs_color = "#1565C0"
    distributed_color = "#E65100"
    figure, axis = plt.subplots(figsize=(12.0, 7.2), constrained_layout=True)

    draw_curve(
        axis,
        bs_curve,
        bs_color,
        f"BS Round Robin（{bs_finished}/{bs_drone_count} 完成）",
    )
    draw_curve(
        axis,
        distributed_curve,
        distributed_color,
        f"纯分布式 RACER（{distributed_finished}/{distributed_drone_count} 完成）",
    )
    axis.hlines(
        bs_coverage,
        bs_plot_elapsed,
        args.deadline,
        colors=bs_color,
        linestyles="--",
        linewidth=1.5,
        alpha=0.7,
        label="BS 停止后的覆盖率保持线",
    )
    axis.axvline(
        bs_plot_elapsed, color=bs_color, linestyle=":", linewidth=1.3, alpha=0.8
    )
    axis.axvline(
        args.deadline, color="#555555", linestyle=":", linewidth=1.3, alpha=0.8
    )
    axis.scatter([bs_plot_elapsed], [bs_coverage], color=bs_color, s=48, zorder=5)
    axis.scatter(
        [distributed_plot_elapsed],
        [distributed_coverage],
        color=distributed_color,
        s=48,
        zorder=5,
    )

    axis.annotate(
        f"{bs_stop_reason}\n{bs_elapsed:.2f} s, {bs_coverage:.2f}%",
        xy=(bs_plot_elapsed, bs_coverage),
        xytext=(-155, -65),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": bs_color},
        color=bs_color,
        bbox={"boxstyle": "round,pad=0.35", "fc": "white", "ec": bs_color},
    )
    axis.annotate(
        f"{distributed_stop_reason}，"
        f"{distributed_finished}/{distributed_drone_count} 完成\n"
        f"{distributed_plot_elapsed:.2f} s, "
        f"{distributed_coverage:.2f}%",
        xy=(
            distributed_plot_elapsed,
            distributed_coverage,
        ),
        xytext=(-205, -125),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": distributed_color},
        color=distributed_color,
        bbox={
            "boxstyle": "round,pad=0.35",
            "fc": "white",
            "ec": distributed_color,
        },
    )

    axis.set_title(args.title, pad=14)
    axis.set_xlabel("仿真运行时间 (s)")
    axis.set_ylabel("联合覆盖率")
    axis.set_xlim(0.0, args.deadline)
    axis.set_ylim(0.0, 101.0)
    tick_step = max(1, int(args.deadline // 9))
    axis.set_xticks(range(0, int(args.deadline) + 1, tick_step))
    axis.yaxis.set_major_formatter(PercentFormatter(xmax=100.0, decimals=0))
    axis.grid(True, linestyle="--", alpha=0.3)
    axis.legend(loc="lower right", framealpha=0.95)

    inset = axis.inset_axes([0.46, 0.21, 0.49, 0.34])
    draw_curve(inset, bs_curve, bs_color, "")
    draw_curve(inset, distributed_curve, distributed_color, "")
    inset.hlines(
        bs_coverage,
        bs_plot_elapsed,
        args.deadline,
        colors=bs_color,
        linestyles="--",
        linewidth=1.2,
        alpha=0.7,
    )
    inset.axvline(bs_plot_elapsed, color=bs_color, linestyle=":", linewidth=1.0)
    inset.set_xlim(args.deadline / 3.0, args.deadline)
    late_coverage = min(
        sample_hold(bs_curve, args.deadline / 3.0),
        sample_hold(distributed_curve, args.deadline / 3.0),
    )
    inset_lower = max(0.0, 5.0 * math.floor((late_coverage - 5.0) / 5.0))
    inset.set_ylim(inset_lower, 100.1)
    inset.set_title("后期覆盖率局部放大", fontsize=10)
    inset.yaxis.set_major_formatter(PercentFormatter(xmax=100.0, decimals=0))
    inset.grid(True, linestyle=":", alpha=0.35)

    png = args.output_dir / "joint_coverage_vs_time.png"
    svg = args.output_dir / "joint_coverage_vs_time.svg"
    figure.savefig(png, dpi=180, bbox_inches="tight")
    figure.savefig(svg, bbox_inches="tight")
    plt.close(figure)

    csv_path = args.output_dir / "joint_coverage_vs_time.csv"
    with csv_path.open("w", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(
            [
                "time_s",
                "bs_round_robin_joint_coverage_pct",
                "distributed_joint_coverage_pct",
            ]
        )
        for stamp in range(int(args.deadline) + 1):
            bs_value = sample_hold(bs_curve, min(float(stamp), bs_plot_elapsed))
            writer.writerow(
                [
                    stamp,
                    f"{bs_value:.9f}",
                    f"{sample_hold(distributed_curve, float(stamp)):.9f}",
                ]
            )

    def summarize(result: dict, curve: Curve) -> dict:
        metrics = result["metrics"]
        stats = result["communication"]["statistics"]
        per_agent = [100.0 * float(v) for v in metrics["mapping_coverage_per_agent"]]
        direct_attempts = int(stats.get("direct_attempted_packets", 0))
        direct_deliveries = int(stats.get("direct_delivered_packets", 0))
        return {
            "network_topology": result["communication"]["network_topology"],
            "passed": bool(result["passed"]),
            "stop_reason": metrics.get("stop_reason"),
            "elapsed_sim_s": float(metrics["elapsed"]),
            "finished_uavs": len(result["finished_drone_ids"]),
            "uav_count": int(result["drone_count"]),
            "joint_coverage_pct": 100.0 * float(metrics["mapping_coverage_joint"]),
            "mean_per_uav_coverage_pct": sum(per_agent) / len(per_agent),
            "minimum_per_uav_coverage_pct": min(per_agent),
            "time_weighted_joint_coverage_pct": time_weighted_coverage(
                curve, args.deadline
            ),
            "time_to_90_pct_s": threshold_time(curve, 90.0),
            "time_to_95_pct_s": threshold_time(curve, 95.0),
            "time_to_98_pct_s": threshold_time(curve, 98.0),
            "total_path_length_m": sum(float(v) for v in metrics["path_lengths"]),
            "collision_events": int(metrics["collision_events"]),
            "minimum_inter_uav_distance_m": float(metrics["min_inter_drone"]),
            "minimum_obstacle_clearance_m": float(metrics["min_obstacle_clearance"]),
            "direct_physical_delivery_ratio": (
                direct_deliveries / direct_attempts if direct_attempts else 0.0
            ),
            "sionna_exact_samples": int(stats.get("sionna_exact_samples", 0)),
            "bs_round_robin_turns": int(stats.get("bs_round_robin_turns", 0)),
            "bs_uplink_chunks_received": int(
                stats.get("bs_incremental_chunks_received_uplink", 0)
            ),
            "bs_downlink_chunks_delivered": int(
                stats.get("bs_missing_chunks_delivered_downlink", 0)
            ),
        }

    summary = {
        "scenario": "warehouse_loaded",
        "deadline_sim_s": args.deadline,
        "depth_camera": {"width": 640, "height": 480, "rate_hz": 30},
        "physics_rate_hz": 200,
        "camera_ray_budget": 76800,
        "bs_round_robin": summarize(bs, bs_curve),
        "distributed": summarize(distributed, distributed_curve),
    }
    summary_json = args.output_dir / "comparison_summary.json"
    summary_json.write_text(json.dumps(summary, indent=2, ensure_ascii=False) + "\n")

    b = summary["bs_round_robin"]
    d = summary["distributed"]
    summary_md = args.output_dir / "COMPARISON.md"
    summary_md.write_text(
        "\n".join(
            [
                "# Warehouse Loaded：BS Round Robin 与纯分布式 RACER",
                "",
                "两组均使用 5 架 UAV、640×480@30 Hz 深度相机、76,800 ray、"
                f"200 Hz 物理频率和 {args.deadline:g} s 仿真截止时间。",
                "",
                "| 指标 | BS Round Robin | 纯分布式 |",
                "|---|---:|---:|",
                f"| 任务完成 | {b['finished_uavs']}/{b['uav_count']} | "
                f"{d['finished_uavs']}/{d['uav_count']} |",
                f"| 停止原因 | {b['stop_reason']} | {d['stop_reason']} |",
                f"| 停止时间 | {b['elapsed_sim_s']:.2f} s | {d['elapsed_sim_s']:.2f} s |",
                f"| 最终联合覆盖率 | {b['joint_coverage_pct']:.4f}% | "
                f"{d['joint_coverage_pct']:.4f}% |",
                f"| 时间加权联合覆盖率 | {b['time_weighted_joint_coverage_pct']:.4f}% | "
                f"{d['time_weighted_joint_coverage_pct']:.4f}% |",
                f"| 达到 90% | {fmt_time(b['time_to_90_pct_s'])} | "
                f"{fmt_time(d['time_to_90_pct_s'])} |",
                f"| 达到 95% | {fmt_time(b['time_to_95_pct_s'])} | "
                f"{fmt_time(d['time_to_95_pct_s'])} |",
                f"| 达到 98% | {fmt_time(b['time_to_98_pct_s'])} | "
                f"{fmt_time(d['time_to_98_pct_s'])} |",
                f"| UAV 最低本地覆盖率 | {b['minimum_per_uav_coverage_pct']:.4f}% | "
                f"{d['minimum_per_uav_coverage_pct']:.4f}% |",
                f"| 总飞行路径 | {b['total_path_length_m']:.2f} m | "
                f"{d['total_path_length_m']:.2f} m |",
                f"| UAV 直链物理交付率 | {100*b['direct_physical_delivery_ratio']:.4f}% | "
                f"{100*d['direct_physical_delivery_ratio']:.4f}% |",
                f"| Sionna 精确链路样本 | {b['sionna_exact_samples']} | "
                f"{d['sionna_exact_samples']} |",
                f"| BS 轮询次数 | {b['bs_round_robin_turns']} | "
                f"{d['bs_round_robin_turns']} |",
                f"| BS 收到/下发 map chunks | {b['bs_uplink_chunks_received']} / "
                f"{b['bs_downlink_chunks_delivered']} | "
                f"{d['bs_uplink_chunks_received']} / {d['bs_downlink_chunks_delivered']} |",
                "",
            ]
        )
    )

    print(png)
    print(svg)
    print(csv_path)
    print(summary_json)
    print(summary_md)


if __name__ == "__main__":
    main()
