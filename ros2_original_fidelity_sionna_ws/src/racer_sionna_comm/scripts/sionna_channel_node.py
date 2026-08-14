#!/usr/bin/env python3
"""Low-rate Sionna RT channel solver with a fast radio-map cache fallback.

Only this node consumes every vehicle's simulator-truth odometry. It publishes
link metrics, never peer poses. The C++ communication emulator turns those
metrics into bandwidth, latency, retransmission, and packet-delivery outcomes.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
import threading
import time
from typing import Dict, Optional, Tuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

from racer_sionna_interfaces.msg import LinkQuality, LinkQualityArray


SPEED_OF_LIGHT_MPS = 299_792_458.0


def _seconds(stamp) -> float:
    return float(stamp.sec) + 1.0e-9 * float(stamp.nanosec)


def _as_numpy(value) -> np.ndarray:
    if hasattr(value, "numpy"):
        return np.asarray(value.numpy())
    return np.asarray(value)


def _quaternion_to_rpy(x: float, y: float, z: float, w: float) -> np.ndarray:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    # Sionna uses alpha, beta, gamma rotations. For the current isotropic
    # profile the orientation has no gain effect, but keeping it synchronized
    # makes directional antenna profiles a drop-in replacement.
    return np.asarray([yaw, pitch, roll], dtype=np.float64)


class HybridRadioMap:
    """Nearest-TX-anchor plus trilinear-RX lookup cache."""

    def __init__(self, filename: Path):
        values = np.load(filename, allow_pickle=False)
        required = ("tx_positions", "rx_x", "rx_y", "rx_z", "path_gain_db")
        missing = [name for name in required if name not in values]
        if missing:
            raise ValueError(f"radio-map cache is missing {missing}")
        self.tx_positions = np.asarray(values["tx_positions"], dtype=np.float64)
        self.rx_x = np.asarray(values["rx_x"], dtype=np.float64)
        self.rx_y = np.asarray(values["rx_y"], dtype=np.float64)
        self.rx_z = np.asarray(values["rx_z"], dtype=np.float64)
        self.path_gain_db = np.asarray(values["path_gain_db"], dtype=np.float64)
        expected = (
            self.tx_positions.shape[0],
            self.rx_z.size,
            self.rx_y.size,
            self.rx_x.size,
        )
        if self.tx_positions.ndim != 2 or self.tx_positions.shape[1] != 3:
            raise ValueError("tx_positions must have shape [anchors,3]")
        if self.path_gain_db.shape != expected:
            raise ValueError(
                f"path_gain_db shape {self.path_gain_db.shape} != {expected}"
            )
        self.los = None
        if "line_of_sight" in values:
            candidate = np.asarray(values["line_of_sight"], dtype=bool)
            if candidate.shape == expected:
                self.los = candidate
        self.metadata = {}
        if "metadata_json" in values:
            self.metadata = json.loads(str(values["metadata_json"].item()))

    @staticmethod
    def _bounds(axis: np.ndarray, value: float) -> Tuple[int, int, float]:
        if axis.size == 1:
            return 0, 0, 0.0
        upper = int(np.searchsorted(axis, value, side="right"))
        upper = max(1, min(upper, axis.size - 1))
        lower = upper - 1
        span = float(axis[upper] - axis[lower])
        ratio = 0.0 if span <= 0.0 else (value - axis[lower]) / span
        return lower, upper, float(np.clip(ratio, 0.0, 1.0))

    def lookup(self, tx_position: np.ndarray, rx_position: np.ndarray):
        anchor = int(
            np.argmin(np.linalg.norm(self.tx_positions - tx_position[None, :], axis=1))
        )
        x0, x1, ax = self._bounds(self.rx_x, float(rx_position[0]))
        y0, y1, ay = self._bounds(self.rx_y, float(rx_position[1]))
        z0, z1, az = self._bounds(self.rx_z, float(rx_position[2]))
        grid = self.path_gain_db[anchor]

        def bilinear(z_index: int) -> float:
            v00 = grid[z_index, y0, x0]
            v10 = grid[z_index, y0, x1]
            v01 = grid[z_index, y1, x0]
            v11 = grid[z_index, y1, x1]
            return float(
                (1.0 - ay) * ((1.0 - ax) * v00 + ax * v10)
                + ay * ((1.0 - ax) * v01 + ax * v11)
            )

        gain = (1.0 - az) * bilinear(z0) + az * bilinear(z1)
        los = False
        if self.los is not None:
            nearest_x = x0 if ax < 0.5 else x1
            nearest_y = y0 if ay < 0.5 else y1
            nearest_z = z0 if az < 0.5 else z1
            los = bool(self.los[anchor, nearest_z, nearest_y, nearest_x])
        return gain, los, anchor


class SionnaChannelNode(Node):
    def __init__(self) -> None:
        super().__init__("racer_sionna_channel")
        self.drone_count = int(self.declare_parameter("drone_count", 3).value)
        self.network_topology = str(
            self.declare_parameter("network_topology", "distributed").value
        )
        self.ap_enabled = self.network_topology in (
            "ap_assisted", "bs_round_robin"
        )
        self.ap_node_id = self.drone_count if self.ap_enabled else None
        self.node_count = self.drone_count + int(self.ap_enabled)
        self.ap_position = np.asarray(
            self.declare_parameter(
                "ap_position", [-0.5, 2.85, 8.10]
            ).value,
            dtype=np.float64,
        )
        self.scene_xml = Path(
            str(self.declare_parameter("scene_xml", "").value)
        ).expanduser()
        cache_text = str(self.declare_parameter("radio_map_cache", "").value)
        self.cache_file = Path(cache_text).expanduser() if cache_text else None
        self.require_sionna = bool(
            self.declare_parameter("require_sionna", True).value
        )
        self.allow_analytic_fallback = bool(
            self.declare_parameter("allow_analytic_fallback", False).value
        )
        self.frequency_hz = float(
            self.declare_parameter("carrier_frequency_hz", 28.0e9).value
        )
        self.bandwidth_hz = float(
            self.declare_parameter("bandwidth_hz", 100.0e6).value
        )
        self.tx_power_dbm = float(
            self.declare_parameter("tx_power_dbm", 23.0).value
        )
        self.ap_tx_power_dbm = float(
            self.declare_parameter("ap_tx_power_dbm", 33.0).value
        )
        self.uav_antenna_gain_dbi = float(
            self.declare_parameter("uav_antenna_gain_dbi", 0.0).value
        )
        self.ap_antenna_gain_dbi = float(
            self.declare_parameter("ap_antenna_gain_dbi", 0.0).value
        )
        self.uav_array_rows = int(
            self.declare_parameter("uav_array_rows", 4).value
        )
        self.uav_array_cols = int(
            self.declare_parameter("uav_array_cols", 4).value
        )
        self.ap_array_rows = int(
            self.declare_parameter("ap_array_rows", 8).value
        )
        self.ap_array_cols = int(
            self.declare_parameter("ap_array_cols", 8).value
        )
        self.directional_beamforming = bool(
            self.declare_parameter("directional_beamforming", True).value
        )
        self.small_scale_fading = str(
            self.declare_parameter("small_scale_fading", "none").value
        )
        self.noise_figure_db = float(
            self.declare_parameter("receiver_noise_figure_db", 7.0).value
        )
        self.temperature_k = float(
            self.declare_parameter("temperature_k", 290.0).value
        )
        self.solver_rate_hz = float(
            self.declare_parameter("solver_rate_hz", 2.0).value
        )
        self.publish_rate_hz = float(
            self.declare_parameter("publish_rate_hz", 10.0).value
        )
        self.validity_s = float(
            self.declare_parameter("link_validity_s", 0.35).value
        )
        self.max_exact_age_s = float(
            self.declare_parameter("max_exact_age_s", 2.0).value
        )
        self.recompute_distance_m = float(
            self.declare_parameter("recompute_distance_m", 0.25).value
        )
        self.max_depth = int(self.declare_parameter("max_depth", 2).value)
        self.samples_per_source = int(
            self.declare_parameter("samples_per_source", 200000).value
        )
        self.enable_diffuse = bool(
            self.declare_parameter("diffuse_reflection", False).value
        )
        self.enable_refraction = bool(
            self.declare_parameter("refraction", True).value
        )
        self.seed = int(self.declare_parameter("random_seed", 42).value)
        self.fallback_path_loss_exponent = float(
            self.declare_parameter("fallback_path_loss_exponent", 2.2).value
        )

        if self.network_topology not in (
            "distributed", "ap_assisted", "bs_round_robin"
        ):
            raise ValueError(
                "network_topology must be distributed, ap_assisted, or "
                "bs_round_robin"
            )
        if self.ap_position.shape != (3,) or not np.all(
            np.isfinite(self.ap_position)
        ):
            raise ValueError("ap_position must contain three finite coordinates")
        if (
            self.drone_count <= 0
            or self.solver_rate_hz <= 0.0
            or self.publish_rate_hz <= 0.0
            or min(
                self.uav_array_rows,
                self.uav_array_cols,
                self.ap_array_rows,
                self.ap_array_cols,
            ) <= 0
        ):
            raise ValueError("drone_count and channel rates must be positive")
        if self.small_scale_fading != "none":
            raise ValueError(
                "small_scale_fading must be 'none'; Sionna RT paths are used "
                "without an added Rayleigh process"
            )
        self.noise_power_dbm = (
            -174.0
            + 10.0 * math.log10(self.bandwidth_hz)
            + self.noise_figure_db
        )
        self.poses: Dict[int, dict] = {}
        if self.ap_enabled:
            self.poses[self.ap_node_id] = {
                "position": self.ap_position.copy(),
                "orientation": np.zeros(3, dtype=np.float64),
                "velocity": np.zeros(3, dtype=np.float64),
                "stamp": 0.0,
            }
        self.pose_lock = threading.Lock()
        self.result_lock = threading.Lock()
        self.exact_results: Dict[Tuple[int, int], dict] = {}
        self.last_solve_poses: Dict[int, np.ndarray] = {}
        self.last_solve_sim_stamp = -math.inf
        self.solve_requested = threading.Event()
        self.stop_requested = threading.Event()
        self.sionna_ready = False
        self.scene = None
        self.path_solver = None
        self.transmitters = []
        self.receivers = []

        self.cache: Optional[HybridRadioMap] = None
        if self.cache_file and self.cache_file.is_file():
            try:
                self.cache = HybridRadioMap(self.cache_file)
                self.get_logger().info(f"loaded hybrid radio-map cache {self.cache_file}")
            except Exception as exception:  # noqa: BLE001
                self.get_logger().error(f"radio-map cache rejected: {exception}")

        self._initialize_sionna()
        qos = rclpy.qos.QoSProfile(
            depth=20,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
        )
        for drone_id in range(self.drone_count):
            self.create_subscription(
                Odometry,
                f"/drone_{drone_id}/odom",
                lambda message, index=drone_id: self._odom(index, message),
                qos,
            )
        self.link_publisher = self.create_publisher(
            LinkQualityArray, "/racer_sionna/link_quality", qos
        )
        self.status_publisher = self.create_publisher(
            String, "/racer_sionna/channel_status", qos
        )
        self.create_timer(1.0 / self.publish_rate_hz, self._publish)
        self.create_timer(1.0 / self.solver_rate_hz, self._request_solve)
        self.worker = threading.Thread(
            target=self._solver_worker,
            name="sionna-path-solver",
            daemon=True,
        )
        self.worker.start()
        self._publish_status()

    def _initialize_sionna(self) -> None:
        if not self.scene_xml.is_file():
            self.get_logger().error(f"Sionna scene XML does not exist: {self.scene_xml}")
            return
        try:
            from sionna.rt import (  # pylint: disable=import-outside-toplevel
                PathSolver,
                PlanarArray,
                Receiver,
                Transmitter,
                load_scene,
            )

            self.scene = load_scene(str(self.scene_xml), merge_shapes=True)
            self.scene.frequency = self.frequency_hz
            self.scene.bandwidth = self.bandwidth_hz
            self.scene.temperature = self.temperature_k
            self.scene.tx_array = PlanarArray(
                num_rows=1,
                num_cols=1,
                vertical_spacing=0.5,
                horizontal_spacing=0.5,
                pattern="iso",
                polarization="V",
            )
            self.scene.rx_array = PlanarArray(
                num_rows=1,
                num_cols=1,
                vertical_spacing=0.5,
                horizontal_spacing=0.5,
                pattern="iso",
                polarization="V",
            )
            for node_id in range(self.node_count):
                is_ap = self.ap_enabled and node_id == self.ap_node_id
                tx = Transmitter(
                    name="industrial_ap_tx" if is_ap else f"uav_tx_{node_id}",
                    position=[0.0, 0.0, -1000.0],
                    power_dbm=(self.ap_tx_power_dbm if is_ap else self.tx_power_dbm),
                )
                rx = Receiver(
                    name="industrial_ap_rx" if is_ap else f"uav_rx_{node_id}",
                    position=[0.0, 0.0, -1000.0],
                )
                self.scene.add(tx)
                self.scene.add(rx)
                self.transmitters.append(tx)
                self.receivers.append(rx)
            self.path_solver = PathSolver()
            self.sionna_ready = True
            self.get_logger().info(
                f"loaded Sionna RT scene {self.scene_xml}; "
                f"topology={self.network_topology} radio_nodes={self.node_count}"
            )
        except Exception as exception:  # noqa: BLE001
            self.get_logger().error(f"Sionna RT initialization failed: {exception}")
            self.sionna_ready = False

    def _publish_status(self) -> None:
        message = String()
        message.data = json.dumps(
            {
                "sionna_ready": self.sionna_ready,
                "cache_ready": self.cache is not None,
                "require_sionna": self.require_sionna,
                "allow_analytic_fallback": self.allow_analytic_fallback,
                "scene_xml": str(self.scene_xml),
                "cache_file": str(self.cache_file) if self.cache_file else "",
                "network_topology": self.network_topology,
                "radio_node_count": self.node_count,
                "ap_node_id": self.ap_node_id,
                "ap_position": self.ap_position.tolist(),
                "carrier_frequency_hz": self.frequency_hz,
                "bandwidth_hz": self.bandwidth_hz,
                "uav_tx_power_dbm": self.tx_power_dbm,
                "bs_tx_power_dbm": self.ap_tx_power_dbm,
                "uav_upa": [self.uav_array_rows, self.uav_array_cols],
                "bs_upa": [self.ap_array_rows, self.ap_array_cols],
                "directional_beamforming": self.directional_beamforming,
                "small_scale_fading": self.small_scale_fading,
            },
            sort_keys=True,
        )
        self.status_publisher.publish(message)

    def _odom(self, drone_id: int, message: Odometry) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        velocity = message.twist.twist.linear
        data = {
            "position": np.asarray([position.x, position.y, position.z], dtype=np.float64),
            "orientation": _quaternion_to_rpy(
                orientation.x, orientation.y, orientation.z, orientation.w
            ),
            "velocity": np.asarray([velocity.x, velocity.y, velocity.z], dtype=np.float64),
            "stamp": _seconds(message.header.stamp),
        }
        with self.pose_lock:
            self.poses[drone_id] = data

    def _request_solve(self) -> None:
        if not self.sionna_ready:
            return
        with self.pose_lock:
            if len(self.poses) != self.node_count:
                return
            moved = not self.last_solve_poses
            if not moved:
                moved = any(
                    np.linalg.norm(
                        self.poses[index]["position"] - self.last_solve_poses[index]
                    )
                    >= self.recompute_distance_m
                    for index in range(self.node_count)
                )
        solve_due = (
            time.monotonic() - getattr(self, "last_solve_wall", -math.inf)
            >= 1.0 / self.solver_rate_hz
        )
        if moved or solve_due:
            self.solve_requested.set()

    def _solver_worker(self) -> None:
        while not self.stop_requested.is_set():
            if not self.solve_requested.wait(timeout=0.1):
                continue
            if self.stop_requested.is_set():
                break
            self.solve_requested.clear()
            with self.pose_lock:
                if len(self.poses) != self.node_count:
                    continue
                snapshot = {
                    index: {
                        name: value.copy() if isinstance(value, np.ndarray) else value
                        for name, value in data.items()
                    }
                    for index, data in self.poses.items()
                }
            started = time.monotonic()
            try:
                for index in range(self.node_count):
                    data = snapshot[index]
                    tx = self.transmitters[index]
                    rx = self.receivers[index]
                    tx.position = data["position"].tolist()
                    rx.position = data["position"].tolist()
                    tx.orientation = data["orientation"].tolist()
                    rx.orientation = data["orientation"].tolist()
                    tx.velocity = data["velocity"].tolist()
                    rx.velocity = data["velocity"].tolist()
                paths = self.path_solver(
                    scene=self.scene,
                    max_depth=self.max_depth,
                    samples_per_src=self.samples_per_source,
                    synthetic_array=True,
                    los=True,
                    specular_reflection=True,
                    diffuse_reflection=self.enable_diffuse,
                    refraction=self.enable_refraction,
                    diffraction=False,
                    seed=self.seed,
                )
                results = self._extract_paths(paths, snapshot)
                solved_stamp = max(data["stamp"] for data in snapshot.values())
                with self.result_lock:
                    self.exact_results = results
                    self.last_solve_sim_stamp = solved_stamp
                with self.pose_lock:
                    self.last_solve_poses = {
                        index: data["position"].copy()
                        for index, data in snapshot.items()
                    }
                self.last_solve_wall = time.monotonic()
                self.get_logger().debug(
                    f"Sionna solve completed in {self.last_solve_wall-started:.3f}s"
                )
            except Exception as exception:  # noqa: BLE001
                self.get_logger().error(f"Sionna PathSolver failed: {exception}")

    def _extract_paths(self, paths, snapshot: Dict[int, dict]) -> Dict[Tuple[int, int], dict]:
        coefficients = paths.a
        if isinstance(coefficients, tuple):
            coefficients = _as_numpy(coefficients[0]) + 1j * _as_numpy(coefficients[1])
        else:
            coefficients = _as_numpy(coefficients)
        power = np.abs(coefficients) ** 2
        if power.ndim == 5:
            gain = np.sum(power, axis=(1, 3, 4))
        else:
            squeezed = np.squeeze(power)
            if squeezed.ndim != 3:
                raise RuntimeError(f"unexpected Sionna coefficient shape {power.shape}")
            gain = np.sum(squeezed, axis=-1)
        if gain.shape != (self.node_count, self.node_count):
            raise RuntimeError(f"unexpected path-gain matrix shape {gain.shape}")

        # With ``synthetic_array=True``, Sionna RT exposes per-path tensors as
        # [rx, tx, path], while interactions keep their leading depth axis as
        # [depth, rx, tx, path].  Do not blindly squeeze interactions: when
        # max_depth happens to equal a drone index (e.g. depth=2, drone 2), the
        # depth axis can be mistaken for the receiver axis.
        doppler = np.squeeze(_as_numpy(paths.doppler))
        valid = np.squeeze(_as_numpy(paths.valid)).astype(bool)
        if doppler.ndim != 3 or doppler.shape[:2] != (
            self.node_count,
            self.node_count,
        ):
            raise RuntimeError(f"unexpected Sionna Doppler shape {doppler.shape}")
        if valid.ndim != 3 or valid.shape[:2] != (
            self.node_count,
            self.node_count,
        ):
            raise RuntimeError(f"unexpected Sionna valid-path shape {valid.shape}")
        interactions = None
        if hasattr(paths, "interactions"):
            interactions = np.asarray(_as_numpy(paths.interactions))
            # Sionna may retain singleton antenna axes in some releases.
            if interactions.ndim == 6:
                interactions = np.squeeze(interactions, axis=(2, 4))
            if interactions.ndim != 4 or interactions.shape[1:3] != (
                self.node_count,
                self.node_count,
            ):
                raise RuntimeError(
                    f"unexpected Sionna interaction shape {interactions.shape}"
                )
        output = {}
        for receiver in range(self.node_count):
            for sender in range(self.node_count):
                if sender == receiver:
                    continue
                value = float(gain[receiver, sender])
                gain_db = 10.0 * math.log10(max(value, 1.0e-30))
                path_doppler = 0.0
                link_valid = valid[receiver, sender]
                if np.any(link_valid):
                    path_doppler = float(
                        np.max(np.abs(doppler[receiver, sender][link_valid]))
                    )
                los = False
                if interactions is not None and np.any(link_valid):
                    link_interactions = interactions[:, receiver, sender, :]
                    # InteractionType.NONE == 0. A valid path is LoS only when
                    # every depth entry is NONE; reflected paths also contain
                    # NONE padding after their final interaction.
                    los_paths = np.all(link_interactions == 0, axis=0)
                    los = bool(np.any(los_paths & link_valid))
                cache_gain = None
                if (
                    self.cache is not None
                    and sender < self.drone_count
                    and receiver < self.drone_count
                ):
                    cache_gain, _, _ = self.cache.lookup(
                        snapshot[sender]["position"], snapshot[receiver]["position"]
                    )
                output[(sender, receiver)] = {
                    "path_gain_db": gain_db,
                    "doppler_hz": path_doppler,
                    "los": los,
                    "cache_gain_at_solve": cache_gain,
                }
        return output

    def _analytic_gain(self, distance: float) -> float:
        distance = max(distance, 0.05)
        reference_loss = 20.0 * math.log10(
            4.0 * math.pi * self.frequency_hz / SPEED_OF_LIGHT_MPS
        )
        return -reference_loss - 10.0 * self.fallback_path_loss_exponent * math.log10(distance)

    def _metric(self, sender: int, receiver: int, positions, stamp: float) -> dict:
        tx_position = positions[sender]
        rx_position = positions[receiver]
        distance = float(np.linalg.norm(tx_position - rx_position))
        cached = None
        if (
            self.cache is not None
            and sender < self.drone_count
            and receiver < self.drone_count
        ):
            cached = self.cache.lookup(tx_position, rx_position)
        with self.result_lock:
            exact = self.exact_results.get((sender, receiver))
            exact_age = stamp - self.last_solve_sim_stamp

        if exact is not None and exact_age <= self.max_exact_age_s:
            gain_db = float(exact["path_gain_db"])
            model = "sionna_exact"
            los = bool(exact["los"])
            if cached is not None and exact["cache_gain_at_solve"] is not None:
                gain_db += float(cached[0] - exact["cache_gain_at_solve"])
                model = "sionna_cache_corrected"
                los = bool(cached[1] or exact["los"])
            doppler = float(exact["doppler_hz"])
        elif cached is not None:
            gain_db = float(cached[0])
            los = bool(cached[1])
            doppler = 0.0
            model = "radio_map_cache"
        elif self.allow_analytic_fallback and not self.require_sionna:
            gain_db = self._analytic_gain(distance)
            los = True
            doppler = 0.0
            model = "analytic_fallback"
        else:
            return {
                "distance": distance,
                "gain_db": -300.0,
                "rss_dbm": -300.0,
                "snr_db": -300.0,
                "doppler": 0.0,
                "los": False,
                "model": "unavailable",
            }
        sender_is_ap = self.ap_enabled and sender == self.ap_node_id
        receiver_is_ap = self.ap_enabled and receiver == self.ap_node_id
        tx_power_dbm = self.ap_tx_power_dbm if sender_is_ap else self.tx_power_dbm
        tx_gain_dbi = (
            self.ap_antenna_gain_dbi if sender_is_ap else self.uav_antenna_gain_dbi
        )
        rx_gain_dbi = (
            self.ap_antenna_gain_dbi if receiver_is_ap else self.uav_antenna_gain_dbi
        )
        if self.directional_beamforming:
            tx_elements = (
                self.ap_array_rows * self.ap_array_cols
                if sender_is_ap
                else self.uav_array_rows * self.uav_array_cols
            )
            rx_elements = (
                self.ap_array_rows * self.ap_array_cols
                if receiver_is_ap
                else self.uav_array_rows * self.uav_array_cols
            )
            # Per-link conjugate beam steering supplies the coherent UPA gain.
            # Geometry, occlusion, reflection, diffraction settings, and path
            # gain still come exclusively from Sionna RT. No random fading is
            # superimposed on those deterministic paths.
            tx_gain_dbi += 10.0 * math.log10(float(tx_elements))
            rx_gain_dbi += 10.0 * math.log10(float(rx_elements))
        rss_dbm = tx_power_dbm + tx_gain_dbi + rx_gain_dbi + gain_db
        return {
            "distance": distance,
            "gain_db": gain_db,
            "rss_dbm": rss_dbm,
            "snr_db": rss_dbm - self.noise_power_dbm,
            "doppler": doppler,
            "los": los,
            "model": model,
        }

    def _publish(self) -> None:
        with self.pose_lock:
            if len(self.poses) != self.node_count:
                return
            positions = {
                index: self.poses[index]["position"].copy()
                for index in range(self.node_count)
            }
        now = self.get_clock().now()
        stamp = now.nanoseconds * 1.0e-9
        valid_until = now + Duration(seconds=self.validity_s)
        message = LinkQualityArray()
        message.stamp = now.to_msg()
        for sender in range(self.node_count):
            for receiver in range(self.node_count):
                if sender == receiver:
                    continue
                metric = self._metric(sender, receiver, positions, stamp)
                link = LinkQuality()
                link.stamp = message.stamp
                link.valid_until = valid_until.to_msg()
                link.sender_id = sender
                link.receiver_id = receiver
                link.distance_m = float(metric["distance"])
                link.line_of_sight = bool(metric["los"])
                link.path_gain_db = float(metric["gain_db"])
                link.rss_dbm = float(metric["rss_dbm"])
                link.snr_db = float(metric["snr_db"])
                link.doppler_hz = float(metric["doppler"])
                link.model = str(metric["model"])
                message.links.append(link)
        self.link_publisher.publish(message)

    def destroy_node(self):
        self.stop_requested.set()
        self.solve_requested.set()
        if hasattr(self, "worker"):
            self.worker.join(timeout=2.0)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = SionnaChannelNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
