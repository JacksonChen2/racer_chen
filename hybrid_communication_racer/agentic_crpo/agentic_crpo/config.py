"""Strict-enough YAML loading while preserving experiment metadata."""

from __future__ import annotations

import copy
from pathlib import Path
from typing import Any

import yaml


def load_config(path: str | Path) -> dict[str, Any]:
    source = Path(path).expanduser().resolve()
    with source.open("r", encoding="utf-8") as stream:
        value = yaml.safe_load(stream)
    if not isinstance(value, dict):
        raise ValueError("CRPO config root must be a mapping")
    config = copy.deepcopy(value)
    config["_config_path"] = str(source)
    required = (
        "seed",
        "n_uavs",
        "qwen",
        "environment",
        "task_loss",
        "constraint",
        "crpo",
        "training",
    )
    missing = [name for name in required if name not in config]
    if missing:
        raise ValueError(f"missing config sections: {missing}")
    if int(config["n_uavs"]) < 1:
        raise ValueError("n_uavs must be positive")
    reward_type = str(
        config.get("reward", {}).get("type", "negative_bs_resource")
    )
    if reward_type not in {
        "negative_bs_resource",
        "coverage",
        "coverage_delta",
    }:
        raise ValueError(f"unsupported reward.type: {reward_type}")
    constraint = config["constraint"]
    constraint_type = str(constraint.get("type", "task_loss"))
    if constraint_type not in {"task_loss", "relay_fanout"}:
        raise ValueError(f"unsupported constraint.type: {constraint_type}")
    if constraint_type == "relay_fanout":
        limit = int(constraint.get("max_relay_recipients_per_uav", -1))
        if not 0 <= limit <= int(config["n_uavs"]) - 1:
            raise ValueError(
                "constraint.max_relay_recipients_per_uav must be between "
                "0 and n_uavs - 1"
            )
    return config


def save_resolved_config(config: dict[str, Any], path: str | Path) -> None:
    output = Path(path)
    output.parent.mkdir(parents=True, exist_ok=True)
    with output.open("w", encoding="utf-8") as stream:
        yaml.safe_dump(config, stream, allow_unicode=True, sort_keys=False)
