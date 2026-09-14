"""Deterministic checkpoint evaluation."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np

from .config import load_config
from .constraint import resolve_constraint
from .crpo_ppo import CRPOPPO
from .factory import make_env


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--checkpoint", required=True)
    parser.add_argument("--episodes", type=int, default=5)
    parser.add_argument("--mock", action="store_true")
    parser.add_argument(
        "--perfect-reference",
        type=Path,
        help="override environment.backend.perfect_reference_result",
    )
    args = parser.parse_args()
    config = load_config(args.config)
    if args.perfect_reference is not None:
        config["environment"]["backend"]["perfect_reference_result"] = str(
            args.perfect_reference.expanduser().resolve()
        )
    env = make_env(config, force_mock=args.mock, disable_qwen=args.mock)
    constraint = resolve_constraint(config)
    model = CRPOPPO.load(
        args.checkpoint, env=env, device=config["training"]["device"]
    )
    model.gamma_task = constraint.gamma_task
    model.eta = constraint.eta
    records = []
    for episode in range(args.episodes):
        observation, _ = env.reset(seed=int(config["seed"]) + episode)
        done = False
        reward_sum = 0.0
        cost_sum = 0.0
        bs_sum = 0.0
        u2u_sum = 0.0
        steps = 0
        max_proposed_fanout = 0
        max_executed_fanout = 0
        final_info = {}
        while not done:
            action, _ = model.predict(observation, deterministic=True)
            observation, reward, terminated, truncated, final_info = env.step(action)
            reward_sum += reward
            cost_sum += float(final_info["cost"])
            bs_sum += float(final_info["C_BS"])
            u2u_sum += float(final_info["C_U2U"])
            steps += 1
            max_proposed_fanout = max(
                max_proposed_fanout,
                int(final_info.get("relay_fanout_proposed_max", 0)),
            )
            max_executed_fanout = max(
                max_executed_fanout,
                int(final_info.get("relay_fanout_executed_max", 0)),
            )
            done = terminated or truncated
        constraint_mode = str(final_info.get("constraint_mode", "task_loss"))
        constraint_value = (
            cost_sum / max(1, steps)
            if constraint_mode == "relay_fanout"
            else float(final_info["L_task"])
        )
        records.append(
            {
                "episode": episode,
                "reward": reward_sum,
                "sum_cost": cost_sum,
                "total_C_BS": bs_sum,
                "total_C_U2U": u2u_sum,
                "L_task_final": final_info["L_task"],
                "Gamma_task": constraint.gamma_task,
                "constraint_mode": constraint_mode,
                "constraint_value": constraint_value,
                "constraint_violation": constraint_value
                - constraint.gamma_task,
                "max_relay_fanout_proposed": max_proposed_fanout,
                "max_relay_fanout_executed": max_executed_fanout,
                "D_traj": final_info["D_traj"],
                "D_cov": final_info["D_cov"],
                "D_red": final_info["D_red"],
                "D_map": final_info["D_map"],
                "coverage": final_info["coverage"],
                "coverage_pc": final_info["coverage_pc"],
                "redundancy": final_info["redundancy"],
                "redundancy_pc": final_info["redundancy_pc"],
                "map_iou": final_info["map_iou"],
                "map_iou_pc": final_info["map_iou_pc"],
                "trajectory_deviation_m": final_info[
                    "trajectory_deviation_m"
                ],
                "telescoping_error": final_info["task_cost_telescoping_error"],
            }
        )
    summary = {
        "episodes": records,
        "mean_reward": float(np.mean([item["reward"] for item in records])),
        "mean_final_task_loss": float(
            np.mean([item["L_task_final"] for item in records])
        ),
        "constraint_calibration": constraint.as_dict(),
    }
    output = Path(config["training"]["output_dir"]) / "evaluation.json"
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(summary, indent=2), encoding="utf-8")
    print(json.dumps(summary, indent=2))
    env.close()


if __name__ == "__main__":
    main()
