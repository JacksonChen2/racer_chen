#!/usr/bin/env python3
"""Ensure the Isaac start trigger cannot outrun all-agent sensor discovery."""

from pathlib import Path
import sys


source = Path(sys.argv[1]).read_text()
required = (
    'declare_parameter<int>("drone_count", 5)',
    'declare_parameter<int>("minimum_cloud_frames", 25)',
    'create_subscription<nav_msgs::msg::Odometry>',
    'create_subscription<sensor_msgs::msg::PointCloud2>',
    'ready_count < drone_count_',
    'publisher_->get_subscription_count()',
)
missing = [item for item in required if item not in source]
if missing:
    raise SystemExit(f"trigger readiness barrier is incomplete: {missing}")

publish = source.index("publisher_->publish(message)")
for guard in ("ready_count < drone_count_", "publisher_->get_subscription_count()"):
    if source.index(guard) > publish:
        raise SystemExit(f"{guard} is not checked before publishing the trigger")

print("PASS: trigger waits for odometry, point-cloud history, and all FSM subscribers")
