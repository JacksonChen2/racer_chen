#!/usr/bin/env python3
"""Run matched 300-second ideal and distributed Sionna experiments sequentially."""
import datetime
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import time

ROOT = Path(__file__).resolve().parents[1]
WS = ROOT / "ros2_ws"
SUITE = Path(os.environ.get("RACER_SUITE_DIR", ROOT / "results" /
             ("matched_perfect_sionna_300s_" + datetime.datetime.now().strftime("%Y%m%d_%H%M%S"))))
SUITE.mkdir(parents=True, exist_ok=True)
(SUITE / "supervisor.pid").write_text(str(os.getpid()) + "\n")
layout = json.loads((ROOT / "config/warehouse_full_10uav_five_sites_layout.json").read_text())
(SUITE / "start_layout.json").write_text(json.dumps(layout, indent=2) + "\n")
env = {k: v for k, v in os.environ.items() if not k.startswith("RACER_")}
settings = {
 "ROS_DOMAIN_ID": "84", "ROS_LOCALHOST_ONLY": "1", "RMW_IMPLEMENTATION": "rmw_fastrtps_cpp",
 "FASTDDS_DEFAULT_PROFILES_FILE": str(WS / "config/fastdds_large_scale.xml"),
 "FASTRTPS_DEFAULT_PROFILES_FILE": str(WS / "config/fastdds_large_scale.xml"),
 "SIONNA_RUNTIME_DIR": str(WS / ".sionna_runtime"),
 "RACER_FIDELITY_SCENARIO": "warehouse_full", "RACER_FIDELITY_DURATION": "300",
 "RACER_FIDELITY_DRONE_COUNT": "10", "RACER_PHYSICS_RATE_HZ": "100",
 "RACER_SENSOR_RATE_HZ": "10", "RACER_CAMERA_RAY_BUDGET": "76800",
 "RACER_SENSOR_WORKER_COUNT": "8", "RACER_TRIGGER_MINIMUM_CLOUD_FRAMES": "0",
 "RACER_TRIGGER_DELAY_S": "5.0", "RACER_SCENE_QUERY_RATE_HZ": "20",
 "RACER_STARTUP_FREE_SPACE_YAW": "0", "RACER_STARTUP_SCAN_DURATION": "0.0",
 "RACER_STARTUP_UNKNOWN_CORRIDOR_DISTANCE": "0.0", "RACER_FIDELITY_HEADLESS": "1",
 "RACER_FIDELITY_VISUALIZE": "0", "RACER_REQUIRE_COMPLETION": "0",
 "RACER_STOP_ON_COMPLETION": "0", "RACER_MAPPING_COVERAGE_TARGET": "0",
 "RACER_RECORD_TRAJECTORY_HISTORY": "1", "RACER_RANDOM_SEED": "42",
 "RACER_START_POSITIONS": " ".join(str(x) for xyz in layout["start_positions"] for x in xyz),
 "RACER_COVERAGE_UPDATE_RATE_HZ": "0.5", "RACER_TASK_METRIC_OBSERVER_MODE": "off",
 "RACER_DEBUG_OVERLAY_SETUP": str(ROOT / "passive_metrics_overlay_ws/install/setup.bash"),
 "RACER_COMM_OVERLAY_SETUP": str(ROOT / "sionna_distributed_overlay_ws/install/setup.bash"),
 "RACER_EXPLORATION_ASSIGNMENT_MODE": "original",
 "RACER_SCENE_USD": str(ROOT / "assets/warehouse_scenes/isaac/warehouse_full_with_industrial_ap.usda"),
 "RACER_VEHICLE_USD": str(ROOT / "assets/isaac_assets/racer_so3_quadrotor/usd/crazyflie_with_racer_dynamics.usd"),
 "RACER_SIONNA_SCENE_XML": str(ROOT / "assets/sionna_scene/warehouse.xml"),
 "RACER_SIONNA_RADIO_MAP_CACHE": str(SUITE / "no_radio_map_cache.npz"),
 "RACER_NETWORK_TOPOLOGY": "distributed", "RACER_MAP_PERFECT_DELIVERY": "false",
 "RACER_INITIAL_ASSIGNMENT_PERFECT_DELIVERY": "false",
 "RACER_RL_BS_SCHEDULER_ENABLED": "false", "RACER_RL_SYNC_ENABLED": "false",
 "RACER_NEAREST_NEIGHBOR_COUNT": "0", "RACER_LOSSLESS_NEAREST_NEIGHBOR_COUNT": "0",
 "RACER_LOSSLESS_COMMUNICATION_RANGE_M": "0.0", "RACER_LOSSLESS_CONTROL_ONLY": "false",
 "RACER_DIRECTED_MESSAGE_UNICAST": "false", "RACER_CHUNK_DATA_PRE_ENQUEUE_DEDUP": "false",
 "RACER_CHUNK_DATA_MAX_PENDING_PER_LINK": "0", "RACER_COMMUNICATION_RANGE_M": "4.0",
 "RACER_UAV_TX_POWER_DBM": "23", "RACER_BANDWIDTH_HZ": "100000000.0",
 "RACER_RESOURCE_BLOCKS": "66", "RACER_MAX_RETRIES": "0", "RACER_BS_MAX_RETRIES": "0",
 "RACER_FIXED_MCS_INDEX": "14", "RACER_IDEAL_COALESCE_WINDOW_MS": "20.0",
}
env.update(settings)
expected = {
 "RACER_SCENE_USD": "e23ed69250e6ff0391faf21e12715ac65bed0f28eab7afed80c9b5315d191c1e",
 "RACER_SIONNA_SCENE_XML": "b8837c2124d49cd34cce025eebdbf6d22e8196ed609a3d28205f9d1b4c6ee168",
}
for key, digest in expected.items():
 assert hashlib.sha256(Path(settings[key]).read_bytes()).hexdigest() == digest, key
(SUITE / "experiment_manifest.json").write_text(json.dumps({
 "settings_shared": settings, "start_positions": layout["start_positions"],
 "modes": ["ideal", "sionna"], "duration_s_per_mode": 300,
 "asset_sha256": expected,
}, indent=2) + "\n")
child = None
def stop(signum, frame):
 if child is not None and child.poll() is None:
  child.terminate()
  child.wait(timeout=45)
 (SUITE / "run_state.txt").write_text("stopped\n")
 raise SystemExit(130)
for sig in (signal.SIGTERM, signal.SIGINT, signal.SIGHUP):
 signal.signal(sig, stop)
summaries = {}
for label, mode in (("perfect", "ideal"), ("sionna_distributed", "sionna")):
 case = SUITE / label / "formal_300s"
 case.mkdir(parents=True)
 (SUITE / "run_state.txt").write_text("running:" + label + "\n")
 case_env = dict(env, RACER_COMMUNICATION_MODE=mode,
                 RACER_REQUIRE_SIONNA="true" if mode == "sionna" else "false",
                 RACER_RESULT_DIR=str(case),
                 RACER_LKH_DIR="/tmp/" + SUITE.name + "_" + label + "_lkh",
                 RACER_ALGORITHM_LABEL="matched_300s_" + label)
 (case / "run_environment.json").write_text(json.dumps({k:v for k,v in case_env.items()
   if k in settings or k.startswith("RACER_")}, indent=2) + "\n")
 print("START", label, datetime.datetime.now().isoformat(), flush=True)
 with (case / "runner.log").open("w") as log:
  child = subprocess.Popen(["bash", str(WS / "run_warehouse_simple_sionna.sh")],
                           env=case_env, stdout=log, stderr=subprocess.STDOUT)
  (case / "runner.pid").write_text(str(child.pid) + "\n")
  status = child.wait()
 (case / "runner_exit_status.txt").write_text(str(status) + "\n")
 result_path = case / "warehouse_full_distributed_result.json"
 info = {"runner_exit_status": status, "result": str(result_path)}
 if result_path.exists():
  d = json.loads(result_path.read_text())
  m = d["metrics"]
  st = d["communication"]["statistics"]
  errors = []
  if m["elapsed"] < 299: errors.append("duration below 299 seconds")
  if m["start_positions"] != layout["start_positions"]: errors.append("start positions mismatch")
  if d["random_seed"] != 42: errors.append("seed mismatch")
  if d["communication"]["mode"] != mode: errors.append("communication mode mismatch")
  if st.get("map_perfect_delivery_enabled"): errors.append("map-only bypass enabled")
  if st.get("initial_assignment_perfect_delivery_enabled"): errors.append("startup bypass enabled")
  if st.get("ap_enabled") or st.get("network_topology") != "distributed": errors.append("topology mismatch")
  if mode == "ideal" and (not st.get("ideal_direct_enabled") or st.get("dropped_per", 0)):
   errors.append("ideal communication validation failed")
  if mode == "sionna" and (st.get("ideal_logical_messages", 0) or not st.get("shared_uav_ofdma_enabled")
       or st.get("sionna_exact_samples", 0) <= 0):
   errors.append("pure Sionna communication validation failed")
  info.update(elapsed_s=m["elapsed"], coverage=m["mapping_coverage_joint"],
      collision_events=m["collision_events"], executed_drone_ids=d["executed_drone_ids"],
      process_crashes=d["algorithm_evidence"]["process_crashes"], acceptance=d["acceptance"],
      passed=d["passed"], configuration_errors=errors,
      attempted_packets=st.get("attempted_packets"), delivered_packets=st.get("delivered_packets"),
      ideal_logical_messages=st.get("ideal_logical_messages"),
      uav_udp_datagrams_enqueued=st.get("uav_udp_datagrams_enqueued"),
      uav_udp_receiver_successes=st.get("uav_udp_receiver_successes"),
      uav_udp_receiver_per_failures=st.get("uav_udp_receiver_per_failures"))
 else:
  info["configuration_errors"] = ["result missing"]
 summaries[label] = info
 (case / "validation_summary.json").write_text(json.dumps(info, indent=2) + "\n")
 (SUITE / "comparison_summary.json").write_text(json.dumps(summaries, indent=2) + "\n")
 print("FINISH", label, json.dumps(info), flush=True)
 if label == "perfect":
  (SUITE / "run_state.txt").write_text("cooldown\n")
  time.sleep(30)
(SUITE / "run_state.txt").write_text("completed\n")
