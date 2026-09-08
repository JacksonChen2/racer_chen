#!/usr/bin/env python3
"""Guard the ROS1 callback-queue semantics used by the unmodified core."""

from pathlib import Path
import sys


header = Path(sys.argv[1]).read_text(errors="strict")

required = (
    "rclcpp::executors::SingleThreadedExecutor executor",
    "rclcpp::spin_until_future_complete",
    "auto client_node = std::make_shared<rclcpp::Node>",
    "throttle(period, __FILE__, __LINE__)",
    "last_by_callsite",
    "TransportHints &bestEffort",
    "if (hints.best_effort_) qos.best_effort(); else qos.reliable();",
)
missing = [text for text in required if text not in header]
if missing:
    raise SystemExit(f"missing ROS1 callback-semantics guards: {missing}")
if "rclcpp::executors::MultiThreadedExecutor" in header:
    raise SystemExit("algorithm callbacks must not use a multithreaded executor")

print("PASS: ROS1 algorithm callbacks are serialized; blocking services use a transport node")
