#!/usr/bin/env python3
"""Ensure experiment time starts only after every sensor publisher matches."""

from pathlib import Path
import sys


source = Path(sys.argv[1]).read_text()
required = (
    "expected_subscribers_per_sensor",
    "publisher.get_subscription_count()",
    "for count in odom_subscribers + cloud_subscribers",
    "count >= 2",
    "graph_deadline = time.monotonic() + 60.0",
    "raise RuntimeError(",
)
missing = [item for item in required if item not in source]
if missing:
    raise SystemExit(f"Isaac ROS graph readiness barrier is incomplete: {missing}")

barrier = source.index("graph_deadline = time.monotonic() + 60.0")
experiment_reset = source.index("bridge.elapsed = 0.0", barrier)
ready_marker = source.index("RACER_3D_ISAAC_READY", experiment_reset)
if not barrier < experiment_reset < ready_marker:
    raise SystemExit("ROS graph barrier must precede reset and experiment-ready marker")

print("PASS: Isaac waits for all odometry and point-cloud subscribers")
