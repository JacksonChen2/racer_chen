#!/usr/bin/env python3
"""Verify incremental monitoring waits for every original FSM."""

import os
from pathlib import Path
import subprocess
import sys
import tempfile
import time


monitor_source = Path(sys.argv[1]).resolve()
with tempfile.TemporaryDirectory(prefix="racer_completion_monitor_") as root_text:
    root = Path(root_text)
    log = root / "launch.log"
    log.touch()
    called = root / "called.txt"
    fake_bin = root / "bin"
    fake_bin.mkdir()
    fake_ros2 = fake_bin / "ros2"
    fake_ros2.write_text(
        "#!/bin/sh\nprintf '%s\\n' \"$*\" > \"$RACER_MONITOR_CALLED\"\n"
    )
    fake_ros2.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = str(fake_bin) + os.pathsep + environment["PATH"]
    environment["RACER_MONITOR_CALLED"] = str(called)
    launch = subprocess.Popen(["sleep", "10"])
    monitor = subprocess.Popen(
        [sys.executable, str(monitor_source), str(log), "5", str(launch.pid)],
        env=environment,
    )
    try:
        with log.open("a") as stream:
            for drone_id in range(1, 5):
                stream.write(
                    f"[racer_original_exploration_{drone_id}-1] "
                    "finish exploration.\n"
                )
            stream.flush()
            time.sleep(0.5)
            if called.exists() or monitor.poll() is not None:
                raise SystemExit("monitor closed before all five FSMs finished")
            stream.write(
                "[racer_original_exploration_5-21] [FSM]: "
                "Drone 5 state: FINISH\n"
            )
            stream.flush()
        monitor.wait(timeout=5.0)
        if monitor.returncode != 0 or not called.exists():
            raise SystemExit("monitor did not close after all five FSMs finished")
        arguments = called.read_text()
        if "/racer_3d/mission_complete" not in arguments:
            raise SystemExit("monitor published to the wrong shutdown topic")
    finally:
        if monitor.poll() is None:
            monitor.terminate()
            monitor.wait(timeout=2.0)
        launch.terminate()
        launch.wait(timeout=2.0)

print("PASS: completion monitor waits for all five original FSMs")
