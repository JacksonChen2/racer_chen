#!/usr/bin/env python3
"""Run the common Isaac USD audit for the C++ workspace."""

import os
from pathlib import Path
import sys


def main() -> None:
    audit = None
    for ancestor in Path(__file__).resolve().parents:
        candidate = (
            ancestor / "ros2_3d_ws" / "src" / "racer_3d" /
            "isaac_sim" / "audit_usd_scene.py"
        )
        if candidate.is_file():
            audit = candidate
            break
    if audit is None:
        raise SystemExit("USD audit tool was not found beside ros2_3d_C_ws")
    os.execv(sys.executable, [sys.executable, str(audit), *sys.argv[1:]])


if __name__ == "__main__":
    main()
