#!/usr/bin/env python3
"""Plot mapping coverage over simulation time for one completed episode."""

from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


def configure_font() -> None:
    font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
    if font_path.is_file():
        font_name = font_manager.FontProperties(fname=str(font_path)).get_name()
        plt.rcParams["font.family"] = font_name
    plt.rcParams["axes.unicode_minus"] = False


def history_arrays(
    history: list[dict], time_key: str, value_key: str
) -> tuple[np.ndarray, np.ndarray]:
    samples = sorted(history, key=lambda item: float(item[time_key]))
    times = np.asarray([0.0] + [float(item[time_key]) for item in samples])
    values = np.asarray([0.0] + [100.0 * float(item[value_key]) for item in samples])
    return times, values


def first_crossing(times: np.ndarray, values: np.ndarray, threshold: float) -> float | None:
    indices = np.flatnonzero(values >= threshold)
    return None if not len(indices) else float(times[int(indices[0])])


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--episode", default="Episode")
    parser.add_argument("--reference-coverage", type=float, default=None)
    args = parser.parse_args()

    result = json.loads(args.result.read_text(encoding="utf-8"))
    metrics = result["metrics"]
    communication = result["communication"]["statistics"]
    joint_t, joint_y = history_arrays(
        metrics["mapping_coverage_joint_history"], "time_s", "ratio"
    )
    bs_t, bs_y = history_arrays(
        communication["task_quality_history"],
        "source_sim_time_s",
        "bs_global_map_coverage",
    )
    elapsed = float(metrics["elapsed"])
    final_joint = 100.0 * float(metrics["mapping_coverage_joint"])
    final_bs = 100.0 * float(communication["bs_global_map_coverage"])

    if not np.isclose(joint_y[-1], final_joint, atol=1.0e-3):
        raise ValueError("joint coverage history does not match the final metric")
    if not np.isclose(bs_y[-1], final_bs, atol=1.0e-3):
        raise ValueError("BS coverage history does not match the final metric")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    csv_path = args.output.with_suffix(".csv")
    timeline = np.unique(np.concatenate((joint_t, bs_t)))
    joint_index = np.searchsorted(joint_t, timeline, side="right") - 1
    bs_index = np.searchsorted(bs_t, timeline, side="right") - 1
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.writer(stream)
        writer.writerow(("simulation_time_s", "joint_coverage_pct", "bs_coverage_pct"))
        writer.writerows(zip(timeline, joint_y[joint_index], bs_y[bs_index]))

    configure_font()
    figure, axis = plt.subplots(figsize=(16, 9), dpi=180, facecolor="#f5f7fa")
    axis.set_facecolor("white")
    axis.step(
        joint_t,
        joint_y,
        where="post",
        color="#172033",
        linewidth=3.6,
        label=f"联合覆盖率（最终 {final_joint:.2f}%）",
        zorder=4,
    )
    axis.step(
        bs_t,
        bs_y,
        where="post",
        color="#2f80c3",
        linewidth=2.0,
        linestyle="--",
        alpha=0.9,
        label=f"BS 地图覆盖率（最终 {final_bs:.2f}%）",
        zorder=3,
    )
    axis.fill_between(
        joint_t, joint_y, step="post", color="#2a9d8f", alpha=0.10, zorder=1
    )

    if args.reference_coverage is not None:
        reference_pct = 100.0 * args.reference_coverage
        axis.axhline(
            reference_pct,
            color="#d79000",
            linewidth=2.2,
            linestyle=(0, (7, 4)),
            label=f"参考覆盖率 {reference_pct:.2f}%",
            zorder=2,
        )
        axis.text(
            elapsed * 0.985,
            reference_pct + 0.7,
            f"距参考值 {final_joint - reference_pct:+.2f} 个百分点",
            ha="right",
            va="bottom",
            fontsize=11,
            color="#9a6700",
            weight="bold",
        )

    milestones: list[tuple[float, float]] = []
    for threshold in (25.0, 50.0, 60.0, 70.0):
        crossing = first_crossing(joint_t, joint_y, threshold)
        if crossing is not None:
            milestones.append((threshold, crossing))
            axis.scatter(
                crossing,
                threshold,
                s=46,
                color="#2a9d8f",
                edgecolor="white",
                linewidth=1.2,
                zorder=6,
            )
            axis.annotate(
                f"{threshold:.0f}% @ {crossing:.1f}s",
                xy=(crossing, threshold),
                xytext=(7, 8),
                textcoords="offset points",
                fontsize=9.5,
                color="#176d62",
                weight="bold",
            )

    axis.scatter(
        [elapsed], [final_joint], s=80, color="#172033", edgecolor="white", zorder=7
    )
    axis.annotate(
        f"最终 {final_joint:.2f}%",
        xy=(elapsed, final_joint),
        xytext=(-100, -34),
        textcoords="offset points",
        arrowprops={"arrowstyle": "->", "color": "#172033", "lw": 1.2},
        fontsize=12,
        weight="bold",
        bbox={
            "boxstyle": "round,pad=0.35",
            "facecolor": "#fff8db",
            "edgecolor": "#e0b44d",
        },
        zorder=8,
    )

    axis.set_title(
        f"{args.episode} 内地图覆盖率随仿真时间变化",
        fontsize=21,
        weight="bold",
        color="#172033",
        pad=17,
    )
    axis.set_xlabel("仿真时间 / s", fontsize=13)
    axis.set_ylabel("覆盖率 / %", fontsize=13)
    axis.set_xlim(0.0, elapsed)
    upper = max(
        75.0,
        final_joint + 6.0,
        100.0 * args.reference_coverage + 4.0
        if args.reference_coverage is not None
        else 0.0,
    )
    axis.set_ylim(0.0, upper)
    axis.set_xticks(np.arange(0.0, elapsed + 0.1, 30.0))
    axis.grid(True, color="#d8dee8", linewidth=0.8, alpha=0.75)
    axis.spines[["top", "right"]].set_visible(False)
    axis.legend(loc="upper left", frameon=True, framealpha=0.95, fontsize=11)
    axis.text(
        0.995,
        0.015,
        f"联合覆盖率采样点：{len(joint_t) - 1}  ·  BS 采样点：{len(bs_t) - 1}",
        transform=axis.transAxes,
        ha="right",
        va="bottom",
        fontsize=9.5,
        color="#667085",
    )
    figure.tight_layout(pad=2.4)
    figure.savefig(args.output, facecolor=figure.get_facecolor())
    plt.close(figure)

    print(
        json.dumps(
            {
                "output": str(args.output),
                "csv": str(csv_path),
                "episode": args.episode,
                "elapsed_s": elapsed,
                "joint_samples": len(joint_t) - 1,
                "bs_samples": len(bs_t) - 1,
                "final_joint_coverage_pct": final_joint,
                "final_bs_coverage_pct": final_bs,
                "milestones_s": {
                    f"{threshold:g}%": crossing for threshold, crossing in milestones
                },
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
