#!/usr/bin/env python3
"""Run the repository's validated Sionna RT solver with C2 message types.

The radio solver is shared with the RACER communication experiments so both
algorithms see the same geometry, PathSolver settings, antenna arrays and link
budget.  Only its ROS interface package is substituted; no channel equation or
Sionna result is changed here.
"""

from __future__ import annotations

import os
from pathlib import Path
import runpy
import sys
import types

from c2_explorer_msgs.msg import LinkQuality, LinkQualityArray


def main() -> None:
    source_text = os.environ.get("C2_SIONNA_CHANNEL_SOURCE", "")
    if not source_text:
        raise SystemExit("C2_SIONNA_CHANNEL_SOURCE is not configured")
    source = Path(source_text).expanduser().resolve()
    if not source.is_file():
        raise SystemExit(f"validated Sionna channel source is missing: {source}")

    interface_module = types.ModuleType("racer_sionna_interfaces.msg")
    interface_module.LinkQuality = LinkQuality
    interface_module.LinkQualityArray = LinkQualityArray
    package_module = types.ModuleType("racer_sionna_interfaces")
    package_module.__path__ = []
    package_module.msg = interface_module
    sys.modules["racer_sionna_interfaces"] = package_module
    sys.modules["racer_sionna_interfaces.msg"] = interface_module
    runpy.run_path(str(source), run_name="__main__")


if __name__ == "__main__":
    main()
