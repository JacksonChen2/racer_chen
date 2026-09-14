"""Run paired deterministic baselines used to calibrate Gamma_task."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .baseline import BASELINE_POLICIES, deterministic_baseline_action
from .config import load_config
from .factory import make_env


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--policy", choices=BASELINE_POLICIES, required=True)
    parser.add_argument("--episodes", type=int, default=1)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument("--perfect-reference", type=Path)
    args = parser.parse_args()
    if args.episodes < 1:
        raise ValueError("--episodes must be positive")
    config = load_config(args.config)
    if args.seed is not None:
        config["seed"] = args.seed
    if args.perfect_reference is not None:
        config["environment"]["backend"]["perfect_reference_result"] = str(
            args.perfect_reference.expanduser().resolve()
        )
    env = make_env(config, force_mock=args.mock, disable_qwen=True)
    action = deterministic_baseline_action(args.policy, int(config["n_uavs"]))
    records = []
    for episode in range(args.episodes):
        _, initial = env.reset(seed=int(config["seed"]) + episode)
        done = False
        sum_cost = 0.0
        bs_total = 0.0
        u2u_total = 0.0
        final_info = initial
        while not done:
            _, _, terminated, truncated, final_info = env.step(action)
            sum_cost += float(final_info["cost"])
            bs_total += float(final_info["C_BS"])
            u2u_total += float(final_info["C_U2U"])
            done = terminated or truncated
        records.append(
            {
                "episode": episode,
                "seed": int(config["seed"]) + episode,
                "L_task_initial": float(initial["L_task_initial"]),
                "L_task_final": float(final_info["L_task"]),
                "sum_CRPO_cost": sum_cost,
                "C_BS_total": bs_total,
                "C_U2U_total": u2u_total,
                "telescoping_error": float(
                    final_info["task_cost_telescoping_error"]
                ),
                "D_traj": float(final_info["D_traj"]),
                "D_cov": float(final_info["D_cov"]),
                "D_red": float(final_info["D_red"]),
                "D_map": float(final_info["D_map"]),
                "coverage": float(final_info["coverage"]),
                "coverage_pc": float(final_info["coverage_pc"]),
                "redundancy": float(final_info["redundancy"]),
                "redundancy_pc": float(final_info["redundancy_pc"]),
                "map_iou": float(final_info["map_iou"]),
                "map_iou_pc": float(final_info["map_iou_pc"]),
                "trajectory_deviation_m": float(
                    final_info["trajectory_deviation_m"]
                ),
            }
        )
    summary = {
        "policy": args.policy,
        "episodes": records,
        "mean_final_task_loss": float(
            np.mean([item["L_task_final"] for item in records])
        ),
        "mean_C_BS_total": float(
            np.mean([item["C_BS_total"] for item in records])
        ),
    }
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    env.close()


if __name__ == "__main__":
    main()
