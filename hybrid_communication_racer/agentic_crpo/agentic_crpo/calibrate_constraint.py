"""Print and persist the resolved task-constraint calibration."""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from .config import load_config
from .constraint import print_calibration, resolve_constraint


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    value = resolve_constraint(load_config(args.config))
    print_calibration(value)
    if args.output is not None:
        output = args.output.expanduser().resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(value.as_dict(), indent=2), encoding="utf-8")


if __name__ == "__main__":
    main()
