#!/usr/bin/env python3
"""Plot UAV trajectories and mapping coverage for one completed episode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from matplotlib import font_manager


def _configure_font() -> None:
    font_path = Path("/usr/share/fonts/opentype/noto/NotoSansCJK-Medium.ttc")
    if font_path.exists():
        name = font_manager.FontProperties(fname=str(font_path)).get_name()
        plt.rcParams["font.family"] = name
    plt.rcParams["axes.unicode_minus"] = False


def _joint_coverage(history: list[dict], n_uavs: int) -> tuple[np.ndarray, np.ndarray]:
    latest = np.zeros(n_uavs, dtype=float)
    times: list[float] = [0.0]
    values: list[float] = [0.0]
    for sample in sorted(history, key=lambda item: float(item["time_s"])):
        drone_id = int(sample["drone_id"])
        if 0 <= drone_id < n_uavs:
            latest[drone_id] = float(sample["ratio"])
        times.append(float(sample["time_s"]))
        values.append(float(np.max(latest)))
    return np.asarray(times), np.asarray(values)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("result", type=Path)
    parser.add_argument("output", type=Path)
    parser.add_argument("--episode", default="Episode 3")
    args = parser.parse_args()

    with args.result.open("r", encoding="utf-8") as handle:
        result = json.load(handle)

    metrics = result["metrics"]
    trajectory = metrics["trajectory_history"]
    coverage_history = metrics["mapping_coverage_history"]
    final_coverage = np.asarray(metrics["mapping_coverage_per_agent"], dtype=float)
    final_joint = float(metrics["mapping_coverage_joint"])
    elapsed = float(metrics["elapsed"])
    n_uavs = len(final_coverage)

    traj_t = np.asarray([item["time_s"] for item in trajectory], dtype=float)
    positions = np.asarray([item["positions"] for item in trajectory], dtype=float)
    if positions.ndim != 3 or positions.shape[1:] != (n_uavs, 3):
        raise ValueError(f"Unexpected trajectory shape: {positions.shape}")

    coverage_by_uav: list[tuple[np.ndarray, np.ndarray]] = []
    for drone_id in range(n_uavs):
        samples = [item for item in coverage_history if int(item["drone_id"]) == drone_id]
        coverage_by_uav.append(
            (
                np.asarray([item["time_s"] for item in samples], dtype=float),
                np.asarray([item["ratio"] for item in samples], dtype=float),
            )
        )
    union_history = metrics.get("mapping_coverage_joint_history", [])
    if union_history:
        union_history = sorted(
            union_history, key=lambda item: float(item["time_s"])
        )
        joint_t = np.asarray(
            [0.0] + [float(item["time_s"]) for item in union_history],
            dtype=float,
        )
        joint_y = np.asarray(
            [0.0] + [float(item["ratio"]) for item in union_history],
            dtype=float,
        )
    else:
        # Compatibility for results produced before the true union bitmap metric.
        joint_t, joint_y = _joint_coverage(coverage_history, n_uavs)

    if not np.isclose(joint_y[-1], final_joint, atol=1e-4):
        raise ValueError(
            f"Joint coverage mismatch: reconstructed={joint_y[-1]}, recorded={final_joint}"
        )

    _configure_font()
    colors = plt.get_cmap("tab10")(np.arange(n_uavs))
    fig = plt.figure(figsize=(19.2, 10.8), dpi=160, facecolor="#f7f8fa")
    grid = fig.add_gridspec(
        2, 2, width_ratios=(1.05, 1.35), height_ratios=(1.5, 0.72),
        left=0.065, right=0.975, top=0.885, bottom=0.09, wspace=0.18, hspace=0.25,
    )
    ax_xy = fig.add_subplot(grid[0, 0])
    ax_z = fig.add_subplot(grid[1, 0])
    ax_cov = fig.add_subplot(grid[:, 1])

    for axis in (ax_xy, ax_z, ax_cov):
        axis.set_facecolor("white")
        axis.grid(True, color="#d7dce2", linewidth=0.7, alpha=0.72)
        axis.spines[["top", "right"]].set_visible(False)

    for drone_id in range(n_uavs):
        xyz = positions[:, drone_id, :]
        color = colors[drone_id]
        label = f"UAV {drone_id + 1}"
        ax_xy.plot(xyz[:, 0], xyz[:, 1], color=color, linewidth=1.65, alpha=0.9, label=label)
        ax_xy.scatter(
            xyz[0, 0], xyz[0, 1], s=55, marker="o", color=color,
            edgecolor="black", linewidth=0.7, zorder=4,
        )
        ax_xy.scatter(
            xyz[-1, 0], xyz[-1, 1], s=105, marker="*", color=color,
            edgecolor="black", linewidth=0.7, zorder=5,
        )
        ax_xy.text(
            xyz[-1, 0] + 0.18, xyz[-1, 1] + 0.18, str(drone_id + 1),
            color=color, fontsize=8.5, weight="bold",
        )

        ax_z.plot(traj_t, xyz[:, 2], color=color, linewidth=1.15, alpha=0.87)

        cov_t, cov_y = coverage_by_uav[drone_id]
        ax_cov.step(
            cov_t, cov_y * 100.0, where="post", color=color, linewidth=1.35,
            alpha=0.72, label=f"UAV {drone_id + 1}  {final_coverage[drone_id] * 100:.1f}%",
        )

    ax_xy.set_title("俯视轨迹（XY）", fontsize=14, weight="bold", pad=10)
    ax_xy.set_xlabel("X / m")
    ax_xy.set_ylabel("Y / m")
    ax_xy.set_aspect("equal", adjustable="box")
    all_x = positions[:, :, 0]
    all_y = positions[:, :, 1]
    x_margin = max(1.0, 0.04 * float(np.ptp(all_x)))
    y_margin = max(1.0, 0.04 * float(np.ptp(all_y)))
    ax_xy.set_xlim(float(all_x.min() - x_margin), float(all_x.max() + x_margin))
    ax_xy.set_ylim(float(all_y.min() - y_margin), float(all_y.max() + y_margin))
    ax_xy.text(
        0.015, 0.02, "● 起点    ★ 终点（数字为 UAV 编号）", transform=ax_xy.transAxes,
        fontsize=9.5, color="#343a40",
        bbox={"boxstyle": "round,pad=0.35", "facecolor": "white", "alpha": 0.9, "edgecolor": "#ced4da"},
    )

    ax_z.set_title("飞行高度", fontsize=12.5, weight="bold", pad=8)
    ax_z.set_xlabel("仿真时间 / s")
    ax_z.set_ylabel("Z / m")
    ax_z.set_xlim(0, elapsed)
    ax_z.set_ylim(bottom=0)

    ax_cov.step(
        joint_t, joint_y * 100.0, where="post", color="#111827", linewidth=3.0,
        label=f"联合覆盖率  {final_joint * 100:.2f}%", zorder=5,
    )
    ax_cov.scatter([elapsed], [final_joint * 100.0], s=85, color="#111827", zorder=6)
    ax_cov.annotate(
        f"最终联合覆盖率\n{final_joint * 100:.2f}%",
        xy=(elapsed, final_joint * 100.0), xytext=(-128, 42), textcoords="offset points",
        fontsize=12, weight="bold", ha="left", va="bottom",
        arrowprops={"arrowstyle": "->", "color": "#111827", "lw": 1.4},
        bbox={"boxstyle": "round,pad=0.45", "facecolor": "#fff8db", "edgecolor": "#f0c36d"},
    )
    ax_cov.set_title("地图覆盖率随仿真时间变化", fontsize=14, weight="bold", pad=10)
    ax_cov.set_xlabel("仿真时间 / s")
    ax_cov.set_ylabel("覆盖率 / %")
    ax_cov.set_xlim(0, elapsed)
    ax_cov.set_ylim(0, max(72.0, float(final_joint * 100.0 + 6.0)))
    ax_cov.legend(loc="upper left", ncol=2, frameon=True, framealpha=0.94, fontsize=9.2)

    fig.suptitle(
        f"最新完整训练回合：{args.episode} — UAV 轨迹与地图覆盖率",
        fontsize=20, weight="bold", y=0.965, color="#172033",
    )
    fig.text(
        0.5, 0.917,
        f"10 架 UAV  ·  仿真 {elapsed:.2f} s  ·  {len(trajectory)} 组轨迹快照  ·  "
        f"覆盖率定义：已知 SDF 体素 / 规划区域体素",
        ha="center", va="center", fontsize=11.5, color="#505968",
    )
    fig.text(
        0.5, 0.025,
        "轨迹为约 0.5 s 间隔记录；覆盖率为约 1 Hz/每 UAV 的记录。联合覆盖率取 peer-fused UAV 地图中的最大覆盖率。",
        ha="center", fontsize=9.5, color="#626b78",
    )

    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=180, facecolor=fig.get_facecolor())
    plt.close(fig)
    print(
        json.dumps(
            {
                "output": str(args.output),
                "trajectory_snapshots": len(trajectory),
                "coverage_samples": len(coverage_history),
                "final_joint_coverage": final_joint,
                "reconstructed_joint_coverage": float(joint_y[-1]),
            },
            ensure_ascii=False,
        )
    )


if __name__ == "__main__":
    main()
