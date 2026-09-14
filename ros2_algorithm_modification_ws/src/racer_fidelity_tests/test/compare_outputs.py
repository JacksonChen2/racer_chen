#!/usr/bin/env python3
"""Run the ROS1-baseline and ROS2-port fixtures with identical input."""

import subprocess
import sys


def run(path: str) -> bytes:
    return subprocess.run([path], check=True, stdout=subprocess.PIPE,
                          stderr=subprocess.PIPE).stdout


baseline = run(sys.argv[1])
ported = run(sys.argv[2])
if baseline != ported:
    raise SystemExit(
        "same-input B-spline output differs: baseline=%d bytes port=%d bytes"
        % (len(baseline), len(ported)))
print("PASS: ROS1 baseline and ROS2 port B-spline outputs are byte-identical")
