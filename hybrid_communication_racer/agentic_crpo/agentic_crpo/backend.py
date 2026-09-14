"""Backend contract, deterministic mock, and filesystem ROS/Isaac bridge."""

from __future__ import annotations

import json
import math
import os
import time
from abc import ABC, abstractmethod
from pathlib import Path
from typing import Any

import numpy as np

from .schemas import BackendStep, RacerSnapshot, TaskMetrics
from .reference_metrics import ReferenceTaskMetricEvaluator


class MissionEndedError(TimeoutError):
    """The final synchronized state was consumed and no next interval exists."""


class RacerBackend(ABC):
    n_uavs: int

    @abstractmethod
    def reset(self, seed: int | None = None) -> RacerSnapshot:
        raise NotImplementedError

    @abstractmethod
    def step(self, action_matrix: np.ndarray) -> BackendStep:
        raise NotImplementedError

    def close(self) -> None:
        return None

    def synchronization_status(self) -> dict[str, Any]:
        return {"enabled": False}


class MockRacerBackend(RacerBackend):
    """Fast deterministic dynamics for CRPO and objective-isolation tests."""

    def __init__(
        self,
        n_uavs: int,
        horizon: int = 128,
        seed: int = 42,
        direct_u2u_resource: float = 11.0,
    ) -> None:
        self.n_uavs = n_uavs
        self.horizon = horizon
        self.default_seed = seed
        self.fixed_direct_resource = direct_u2u_resource
        self.rng = np.random.default_rng(seed)
        self.slot = 0
        self.decision_index = 0
        self.positions = np.zeros((n_uavs, 3), np.float32)
        self.velocities = np.zeros_like(self.positions)
        self.yaws = np.zeros(n_uavs, np.float32)
        self.channel = np.zeros((n_uavs + 1, n_uavs + 1), np.float32)
        self.local_bytes = np.zeros(n_uavs, np.float32)
        self.bs_bytes = np.zeros(n_uavs, np.float32)
        self.receiver_bytes = np.zeros((n_uavs, n_uavs), np.float32)
        self.pair_aoi = np.zeros((n_uavs, n_uavs), np.float32)
        self.bs_aoi = np.zeros(n_uavs, np.float32)
        self.coverage = 0.0

    def reset(self, seed: int | None = None) -> RacerSnapshot:
        self.rng = np.random.default_rng(self.default_seed if seed is None else seed)
        self.slot = 0
        self.decision_index = 0
        self.positions = self.rng.uniform(
            low=(-22.0, 3.0, 1.0), high=(2.0, 27.0, 3.0), size=(self.n_uavs, 3)
        ).astype(np.float32)
        self.velocities.fill(0.0)
        self.yaws = self.rng.uniform(-math.pi, math.pi, self.n_uavs).astype(
            np.float32
        )
        self.local_bytes.fill(120_000.0)
        self.bs_bytes.fill(0.0)
        self.receiver_bytes.fill(0.0)
        self.pair_aoi.fill(0.0)
        self.bs_aoi.fill(0.0)
        self.coverage = 0.02
        self._update_channel()
        return self._snapshot(coverage_delta=0.0)

    def _update_channel(self) -> None:
        n = self.n_uavs
        bs = np.asarray((-10.03, 14.89, 7.55), np.float32)
        self.channel.fill(-120.0)
        for sender in range(n):
            for receiver in range(n):
                if sender != receiver:
                    distance = np.linalg.norm(
                        self.positions[sender] - self.positions[receiver]
                    )
                    self.channel[sender, receiver] = 24.0 - 1.5 * distance
            distance_bs = np.linalg.norm(self.positions[sender] - bs)
            self.channel[sender, n] = 30.0 - 1.25 * distance_bs
            self.channel[n, sender] = 35.0 - 1.15 * distance_bs

    def _missing(self) -> tuple[np.ndarray, np.ndarray]:
        pair = np.maximum(self.local_bytes[:, None] - self.receiver_bytes, 0.0)
        np.fill_diagonal(pair, 0.0)
        bs = np.maximum(self.local_bytes - self.bs_bytes, 0.0)
        return pair.astype(np.float32), bs.astype(np.float32)

    def _task_metrics(self) -> TaskMetrics:
        pair, bs = self._missing()
        scale = max(1.0, float(np.sum(self.local_bytes) * self.n_uavs))
        d_map = float(np.sum(bs) / max(1.0, np.sum(self.local_bytes)))
        d_red = float(np.sum(pair) / scale)
        progress = self.decision_index / max(1, self.horizon)
        coverage_pc = min(1.0, 0.02 + progress)
        d_cov = float(max(0.0, coverage_pc - self.coverage))
        target = np.stack(
            (
                np.linspace(-20.0, 0.0, self.n_uavs),
                np.linspace(5.0, 25.0, self.n_uavs),
                np.full(self.n_uavs, 2.0),
            ),
            axis=1,
        )
        d_traj = float(np.mean(np.linalg.norm(self.positions - target, axis=1)) / 50.0)
        return TaskMetrics(
            d_traj=d_traj,
            d_cov=d_cov,
            d_red=d_red,
            d_map=d_map,
            coverage=float(self.coverage),
            coverage_pc=float(coverage_pc),
            redundancy=float(d_red),
            redundancy_pc=0.0,
            map_iou=float(1.0 - d_map),
            map_iou_pc=1.0,
            trajectory_deviation_m=float(d_traj * 50.0),
        )

    def _snapshot(self, coverage_delta: float) -> RacerSnapshot:
        pair_missing, bs_missing = self._missing()
        relay_queues = np.maximum(self.bs_bytes[:, None] - self.receiver_bytes, 0.0)
        np.fill_diagonal(relay_queues, 0.0)
        version_gap = np.concatenate(
            (pair_missing / 1200.0, (bs_missing / 1200.0)[:, None]), axis=1
        )
        return RacerSnapshot(
            n_uavs=self.n_uavs,
            slot=self.slot,
            sim_time_s=0.02 * self.slot,
            positions=self.positions.copy(),
            velocities=self.velocities.copy(),
            yaws=self.yaws.copy(),
            fsm_states=[
                "EXPLORE" if self.decision_index < self.horizon else "FINISH"
            ]
            * self.n_uavs,
            channel_snr_db=self.channel.copy(),
            pair_aoi_s=self.pair_aoi.copy(),
            bs_aoi_s=self.bs_aoi.copy(),
            pair_missing_bytes=pair_missing,
            bs_missing_bytes=bs_missing,
            uplink_queue_bytes=bs_missing.copy(),
            relay_queue_bytes=relay_queues.astype(np.float32),
            map_summary={
                "representation": "coarse_top_down_statistics",
                "known_ratio": self.coverage,
                "unknown_ratio": 1.0 - self.coverage,
            },
            information_version_gap=version_gap.astype(np.float32),
            coverage=float(self.coverage),
            coverage_delta=float(coverage_delta),
            task_metrics=self._task_metrics(),
            terminated=self.decision_index >= self.horizon,
            communication_slot_index=self.slot,
            rl_decision_index=self.decision_index,
            action_held_slots=(0 if self.decision_index == 0 else 5),
            communication_slot_duration_s=0.02,
            rl_decision_interval_s=0.1,
            state_sequence=self.decision_index + 1,
        ).validate()

    def _step_communication_slot(
        self, action: np.ndarray
    ) -> tuple[float, float]:
        n = self.n_uavs
        self.slot += 1
        self.pair_aoi += 0.02
        self.bs_aoi += 0.02
        ul_resource = 0.0
        dl_resource = 0.0
        quantum = 12_000.0
        for owner in range(n):
            if action[n, owner] and self.channel[owner, n] > -5.0:
                amount = min(quantum, self.local_bytes[owner] - self.bs_bytes[owner])
                if amount > 0.0:
                    self.bs_bytes[owner] += amount
                    self.bs_aoi[owner] = 0.0
                    ul_resource += 66.0 * math.ceil(amount / 1200.0)
        for owner in range(n):
            for receiver in range(n):
                if owner == receiver or not action[owner, receiver]:
                    continue
                if self.channel[n, receiver] <= -5.0:
                    continue
                available = self.bs_bytes[owner] - self.receiver_bytes[owner, receiver]
                amount = min(quantum, available)
                if amount > 0.0:
                    self.receiver_bytes[owner, receiver] += amount
                    self.pair_aoi[owner, receiver] = 0.0
                    dl_resource += 66.0 * math.ceil(amount / 1200.0)

        # Original direct U2U proceeds independently of the RL action.
        for owner in range(n):
            for receiver in range(n):
                if owner != receiver and self.channel[owner, receiver] > 8.0:
                    available = self.local_bytes[owner] - self.receiver_bytes[owner, receiver]
                    if available > 0.0:
                        self.receiver_bytes[owner, receiver] += min(2400.0, available)
                        self.pair_aoi[owner, receiver] = 0.0

        if self.slot % 4 == 0:
            self.local_bytes += self.rng.integers(1200, 4801, self.n_uavs)
        self.coverage = min(
            1.0, self.coverage + 0.0005 + 1.0e-9 * float(np.sum(self.bs_bytes))
        )
        phase = 0.04 * self.slot + np.arange(n)
        self.velocities[:, 0] = 0.25 * np.cos(phase)
        self.velocities[:, 1] = 0.25 * np.sin(phase)
        self.positions += 0.02 * self.velocities
        self.yaws = np.arctan2(self.velocities[:, 1], self.velocities[:, 0])
        self._update_channel()
        return ul_resource, dl_resource

    def step(self, action_matrix: np.ndarray) -> BackendStep:
        n = self.n_uavs
        action = np.asarray(action_matrix, dtype=np.int8)
        if action.shape != (n + 1, n) or np.any(np.diag(action[:n])):
            raise ValueError("invalid CRPO action matrix")
        old_coverage = self.coverage
        ul_resource = 0.0
        dl_resource = 0.0
        for _ in range(5):
            slot_ul, slot_dl = self._step_communication_slot(action)
            ul_resource += slot_ul
            dl_resource += slot_dl
        self.decision_index += 1
        return BackendStep(
            snapshot=self._snapshot(self.coverage - old_coverage),
            bs_ul_resource=ul_resource,
            bs_dl_resource=dl_resource,
            direct_u2u_resource=5.0 * self.fixed_direct_resource,
        )


class FileBridgeBackend(RacerBackend):
    """Exchange actions/state with the optional C++ scheduler file bridge."""

    def __init__(
        self,
        n_uavs: int,
        action_path: str,
        telemetry_path: str,
        mission_state_path: str | None = None,
        perfect_reference_result: str | None = None,
        auxiliary_reference_result: str | None = None,
        workspace_min: tuple[float, float, float] = (-22.0, 3.0, 1.0),
        workspace_max: tuple[float, float, float] = (2.0, 27.0, 3.0),
        redundancy_cell_m: float = 1.0,
        fixed_exploration_time_s: float = 300.0,
        require_auxiliary_reference_metrics: bool = True,
        require_mission_state: bool = True,
        require_perfect_reference: bool = True,
        max_relay_recipients_per_uav: int | None = None,
        timeout_s: float = 5.0,
        poll_interval_s: float = 0.002,
        synchronous_online: bool = False,
        sync_acknowledgement_path: str | None = None,
        sync_release_path: str | None = None,
        communication_slot_s: float = 0.02,
        decision_interval_s: float = 0.1,
        slots_per_decision: int = 5,
    ) -> None:
        self.n_uavs = n_uavs
        self.action_path = Path(action_path)
        self.telemetry_path = Path(telemetry_path)
        self.mission_state_path = (
            None if mission_state_path is None else Path(mission_state_path)
        )
        self.timeout_s = timeout_s
        self.poll_interval_s = poll_interval_s
        self.require_mission_state = require_mission_state
        self.require_perfect_reference = require_perfect_reference
        self.synchronous_online = bool(synchronous_online)
        self.sync_acknowledgement_path = (
            Path(sync_acknowledgement_path)
            if sync_acknowledgement_path is not None
            else None
        )
        self.sync_release_path = (
            Path(sync_release_path) if sync_release_path is not None else None
        )
        self.communication_slot_s = float(communication_slot_s)
        self.decision_interval_s = float(decision_interval_s)
        self.slots_per_decision = int(slots_per_decision)
        if self.synchronous_online:
            if (
                self.sync_acknowledgement_path is None
                or self.sync_release_path is None
            ):
                raise ValueError(
                    "synchronous file bridge requires acknowledgement and release paths"
                )
            if (
                self.communication_slot_s <= 0.0
                or self.decision_interval_s <= 0.0
                or self.slots_per_decision != 5
                or not math.isclose(
                    self.decision_interval_s,
                    self.communication_slot_s * self.slots_per_decision,
                    rel_tol=0.0,
                    abs_tol=1.0e-12,
                )
            ):
                raise ValueError(
                    "synchronous bridge requires T_RL=5*T_comm"
                )
        if max_relay_recipients_per_uav is not None and not (
            0 <= max_relay_recipients_per_uav <= max(0, n_uavs - 1)
        ):
            raise ValueError("invalid file-bridge relay fanout limit")
        self.max_relay_recipients_per_uav = (
            max_relay_recipients_per_uav
        )
        self.metric_evaluator = ReferenceTaskMetricEvaluator(
            perfect_reference_result,
            n_uavs,
            workspace_min,
            workspace_max,
            redundancy_cell_m=redundancy_cell_m,
            fixed_exploration_time_s=fixed_exploration_time_s,
            require_auxiliary_reference_metrics=require_auxiliary_reference_metrics,
            auxiliary_reference_result=auxiliary_reference_result,
        )
        self.action_epoch = 0
        self.last_sequence = -1
        self.last_decision_index = -1
        self.last_communication_slot_index = -1
        self.last_sim_time_s = 0.0
        self.last_snapshot_terminal = False
        self.last_resources = np.zeros(3, dtype=np.float64)

    @staticmethod
    def _read_json(path: Path) -> dict[str, Any]:
        with path.open("r", encoding="utf-8") as stream:
            value = json.load(stream)
        if not isinstance(value, dict):
            raise ValueError(f"bridge payload is not an object: {path}")
        return value

    @staticmethod
    def _atomic_json(path: Path, payload: dict[str, Any]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + ".tmp")
        temporary.write_text(
            json.dumps(payload, separators=(",", ":"), allow_nan=False),
            encoding="utf-8",
        )
        os.replace(temporary, path)

    def synchronization_status(self) -> dict[str, Any]:
        if not self.synchronous_online:
            return {"enabled": False}
        assert self.sync_acknowledgement_path is not None
        status: dict[str, Any] = {"enabled": True}
        try:
            status.update(self._read_json(self.sync_acknowledgement_path))
        except (OSError, ValueError, json.JSONDecodeError):
            status["status"] = "unavailable"
        try:
            payload = self._read_json(self.telemetry_path)
            status["telemetry_sim_time_s"] = float(payload["sim_time_s"])
            status["telemetry_sequence"] = int(payload["sequence"])
            status["telemetry_communication_slot_index"] = int(
                payload["communication_slot_index"]
            )
            status["telemetry_rl_decision_index"] = int(
                payload["rl_decision_index"]
            )
        except (OSError, ValueError, KeyError, json.JSONDecodeError):
            pass
        return status

    def _wait_synchronous_payload(
        self, expected_decision_index: int | None
    ) -> dict[str, Any]:
        assert self.sync_acknowledgement_path is not None
        deadline = time.monotonic() + self.timeout_s
        last_error: Exception | str | None = None
        while time.monotonic() < deadline:
            try:
                payload = self._read_json(self.telemetry_path)
                if not bool(payload.get("synchronous_online", False)):
                    raise ValueError(
                        "communication state is not in synchronous-online mode"
                    )
                decision_index = int(payload["rl_decision_index"])
                if (
                    expected_decision_index is not None
                    and decision_index > expected_decision_index
                ):
                    raise RuntimeError(
                        "synchronous communication state skipped a decision: "
                        f"expected={expected_decision_index} observed={decision_index}"
                    )
                acknowledgement = self._read_json(
                    self.sync_acknowledgement_path
                )
                matches_expected = (
                    expected_decision_index is None
                    or decision_index == expected_decision_index
                )
                if (
                    matches_expected
                    and int(acknowledgement.get("rl_decision_index", -1))
                    == decision_index
                    and int(acknowledgement.get("state_sequence", -1))
                    == int(payload["sequence"])
                    and acknowledgement.get("status") in {"paused", "terminal"}
                ):
                    state_time = float(payload["sim_time_s"])
                    ack_time = float(acknowledgement["sim_time_s"])
                    if not math.isclose(
                        state_time, ack_time, rel_tol=0.0, abs_tol=1.0e-6
                    ):
                        raise RuntimeError(
                            "Isaac acknowledgement and communication state use "
                            "different simulation times"
                        )
                    return payload
            except RuntimeError:
                raise
            except (
                OSError,
                ValueError,
                KeyError,
                json.JSONDecodeError,
            ) as error:
                last_error = error
            time.sleep(self.poll_interval_s)
        raise TimeoutError(
            "no acknowledged synchronous RL boundary state from "
            f"{self.telemetry_path}; expected_decision="
            f"{expected_decision_index} last_error={last_error}"
        )

    def _wait_payload(self, after_sequence: int | None = None) -> dict[str, Any]:
        deadline = time.monotonic() + self.timeout_s
        last_error: Exception | None = None
        while time.monotonic() < deadline:
            try:
                payload = self._read_json(self.telemetry_path)
                sequence = int(payload["sequence"])
                if after_sequence is None or sequence > after_sequence:
                    return payload
            except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
                last_error = error
            if after_sequence is not None and self.mission_state_path is not None:
                mission_ended = False
                try:
                    mission = self._read_json(self.mission_state_path)
                    mission_ended = bool(
                        mission.get("terminated", False)
                        or mission.get("truncated", False)
                    )
                except (OSError, ValueError, KeyError, json.JSONDecodeError) as error:
                    last_error = error
                # TimeoutError is an OSError subclass.  Raise outside the file
                # parsing try/except so the terminal boundary is not mistaken
                # for a transient read failure and delayed until timeout_s.
                if mission_ended:
                    raise MissionEndedError(
                        "Isaac mission ended before another RL telemetry "
                        "sequence was published"
                    )
            time.sleep(self.poll_interval_s)
        raise TimeoutError(
            f"no fresh RL telemetry from {self.telemetry_path}; last_error={last_error}"
        )

    def _snapshot(self, payload: dict[str, Any]) -> RacerSnapshot:
        n = self.n_uavs
        mission: dict[str, Any] = {}
        if self.mission_state_path is not None and self.mission_state_path.exists():
            mission = self._read_json(self.mission_state_path)
        if self.synchronous_online:
            mission_time = float(mission.get("sim_time_s", math.nan))
            payload_time = float(payload["sim_time_s"])
            if not math.isclose(
                mission_time, payload_time, rel_tol=0.0, abs_tol=1.0e-6
            ):
                raise RuntimeError(
                    "mission and communication states are not from the same "
                    f"RL boundary: mission={mission_time} communication={payload_time}"
                )
        metrics_payload = mission.get("task_metrics", payload.get("task_metrics"))
        metrics = (
            TaskMetrics(
                d_traj=float(metrics_payload["D_traj"]),
                d_cov=float(metrics_payload["D_cov"]),
                d_red=float(metrics_payload["D_red"]),
                d_map=float(metrics_payload["D_map"]),
                coverage=float(metrics_payload.get("coverage", 0.0)),
                coverage_pc=float(metrics_payload.get("coverage_pc", 0.0)),
                redundancy=float(metrics_payload.get("redundancy", 0.0)),
                redundancy_pc=float(metrics_payload.get("redundancy_pc", 0.0)),
                map_iou=float(metrics_payload.get("map_iou", 0.0)),
                map_iou_pc=float(metrics_payload.get("map_iou_pc", 0.0)),
                trajectory_deviation_m=float(
                    metrics_payload.get("trajectory_deviation_m", 0.0)
                ),
            )
            if metrics_payload is not None
            else self.metric_evaluator.evaluate(payload, mission)
        )
        positions = payload.get("positions", mission.get("positions"))
        velocities = payload.get("velocities", mission.get("velocities"))
        yaws = payload.get("yaws", mission.get("yaws"))
        snapshot = RacerSnapshot(
            n_uavs=n,
            slot=int(payload.get("communication_slot_index", payload["sequence"])),
            sim_time_s=float(payload.get("sim_time_s", mission.get("sim_time_s", 0.0))),
            positions=positions,
            velocities=velocities,
            yaws=yaws,
            fsm_states=list(mission.get("fsm_states", payload.get("fsm_states", ["UNKNOWN"] * n))),
            channel_snr_db=payload["channel_snr_db"],
            pair_aoi_s=payload["pair_aoi_s"],
            bs_aoi_s=payload["bs_aoi_s"],
            pair_missing_bytes=payload["pair_missing_bytes"],
            bs_missing_bytes=payload["bs_missing_bytes"],
            uplink_queue_bytes=payload["uplink_queue_bytes"],
            relay_queue_bytes=payload["relay_queue_bytes"],
            map_summary=mission.get("map_summary", payload.get("map_summary", {})),
            trajectory_summary=mission.get(
                "trajectory_summary", payload.get("trajectory_summary", [])
            ),
            region_summary=mission.get(
                "region_summary", payload.get("region_summary", [])
            ),
            information_version_gap=payload.get("information_version_gap"),
            coverage=float(mission.get("coverage", payload.get("coverage", 0.0))),
            coverage_delta=float(mission.get("coverage_delta", 0.0)),
            task_metrics=metrics,
            terminated=bool(mission.get("terminated", False)),
            truncated=bool(mission.get("truncated", False)),
            communication_slot_index=int(
                payload.get("communication_slot_index", payload["sequence"])
            ),
            rl_decision_index=int(
                payload.get("rl_decision_index", payload["sequence"])
            ),
            action_held_slots=int(payload.get("action_held_slots", 1)),
            communication_slot_duration_s=float(
                payload.get("communication_slot_duration_s", 0.02)
            ),
            rl_decision_interval_s=float(
                payload.get("rl_decision_interval_s", 0.02)
            ),
            state_sequence=int(payload["sequence"]),
        )
        return snapshot.validate()

    def reset(self, seed: int | None = None) -> RacerSnapshot:
        del seed
        self.metric_evaluator.reset()
        if self.require_mission_state and (
            self.mission_state_path is None or not self.mission_state_path.exists()
        ):
            raise FileNotFoundError(
                "real CRPO training requires a fresh Isaac mission_state_path; "
                "the perfect reference/ground-truth metrics are loss-only and "
                "must not be silently replaced with zeros"
            )
        if self.require_perfect_reference:
            self.metric_evaluator.validate_reference()
        payload = (
            self._wait_synchronous_payload(None)
            if self.synchronous_online
            else self._wait_payload()
        )
        self.last_sequence = int(payload["sequence"])
        self.last_decision_index = int(
            payload.get("rl_decision_index", self.last_sequence)
        )
        self.last_communication_slot_index = int(
            payload.get("communication_slot_index", self.last_sequence)
        )
        self.last_sim_time_s = float(payload.get("sim_time_s", 0.0))
        # Continue above any epoch already consumed by a still-running proxy
        # (or a stale action file it read during startup). This makes resumed
        # training sessions immediately acceptable to the monotonic C++ reader.
        self.action_epoch = max(
            self.action_epoch, int(payload.get("action_epoch", 0))
        )
        self.last_resources = np.asarray(
            (
                payload.get("bs_uplink_prb_slots", 0.0),
                payload.get("bs_downlink_prb_slots", 0.0),
                payload.get("direct_u2u_prb_slots", 0.0),
            ),
            dtype=np.float64,
        )
        snapshot = self._snapshot(payload)
        self.last_snapshot_terminal = bool(
            snapshot.terminated or snapshot.truncated
        )
        return snapshot

    def _write_action(
        self, action_matrix: np.ndarray, decision_index: int | None = None
    ) -> int:
        n = self.n_uavs
        matrix = np.asarray(action_matrix, dtype=np.int8)
        if matrix.shape != (n + 1, n):
            raise ValueError("invalid file-bridge action matrix")
        if self.max_relay_recipients_per_uav is not None:
            fanout = np.sum(matrix[:n], axis=1)
            if np.any(fanout > self.max_relay_recipients_per_uav):
                raise ValueError(
                    "file bridge refused action above the configured relay "
                    f"fanout limit {self.max_relay_recipients_per_uav}: "
                    f"counts={fanout.tolist()}"
                )
        # The C++ side deliberately uses a dependency-free whitespace parser.
        flat: list[str] = []
        for owner in range(n):
            for receiver in range(n):
                if owner != receiver:
                    flat.append(str(int(matrix[owner, receiver])))
        flat.extend(str(int(item)) for item in matrix[n])
        self.action_epoch += 1
        if self.synchronous_online:
            if decision_index is None or decision_index < 0:
                raise ValueError("synchronous action requires a decision index")
            header = f"{self.action_epoch} {n} {decision_index} "
        else:
            header = f"{self.action_epoch} {n} "
        content = header + " ".join(flat) + "\n"
        self.action_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.action_path.with_suffix(self.action_path.suffix + ".tmp")
        temporary.write_text(content, encoding="ascii")
        os.replace(temporary, self.action_path)
        return self.action_epoch

    def _release_synchronous_boundary(
        self, decision_index: int, action_epoch: int
    ) -> None:
        assert self.sync_release_path is not None
        payload = {
            "rl_decision_index": int(decision_index),
            "action_epoch": int(action_epoch),
            "trainer_pid": os.getpid(),
            "simulation_time_s": self.last_sim_time_s,
            "communication_slot_index": self.last_communication_slot_index,
            "released_at_unix_s": time.time(),
        }
        self._atomic_json(self.sync_release_path, payload)
        print(
            "RACER_RL_SYNC_ACTION "
            + json.dumps(payload, separators=(",", ":"), allow_nan=False),
            flush=True,
        )

    def step(self, action_matrix: np.ndarray) -> BackendStep:
        if self.synchronous_online and self.last_snapshot_terminal:
            raise MissionEndedError(
                "final synchronized RL state was already consumed"
            )
        previous_decision = self.last_decision_index
        previous_slot = self.last_communication_slot_index
        previous_sim_time = self.last_sim_time_s
        action_epoch = self._write_action(
            action_matrix,
            decision_index=(previous_decision if self.synchronous_online else None),
        )
        if self.synchronous_online:
            self._release_synchronous_boundary(previous_decision, action_epoch)
            payload = self._wait_synchronous_payload(previous_decision + 1)
        else:
            payload = self._wait_payload(after_sequence=self.last_sequence)
        self.last_sequence = int(payload["sequence"])
        self.last_decision_index = int(
            payload.get("rl_decision_index", self.last_sequence)
        )
        self.last_communication_slot_index = int(
            payload.get("communication_slot_index", self.last_sequence)
        )
        self.last_sim_time_s = float(payload.get("sim_time_s", previous_sim_time))
        if self.synchronous_online:
            slot_delta = self.last_communication_slot_index - previous_slot
            time_delta = self.last_sim_time_s - previous_sim_time
            if slot_delta != self.slots_per_decision:
                raise RuntimeError(
                    "synchronous transition did not contain exactly five "
                    f"communication slots: delta={slot_delta}"
                )
            if not math.isclose(
                time_delta,
                self.decision_interval_s,
                rel_tol=0.0,
                abs_tol=1.0e-6,
            ):
                raise RuntimeError(
                    "synchronous transition simulation-time delta is not "
                    f"{self.decision_interval_s}: delta={time_delta}"
                )
            if int(payload.get("action_held_slots", -1)) != self.slots_per_decision:
                raise RuntimeError(
                    "communication proxy did not hold the CRPO action for "
                    "exactly five slots"
                )
        cumulative = np.asarray(
            (
                payload.get("bs_uplink_prb_slots", 0.0),
                payload.get("bs_downlink_prb_slots", 0.0),
                payload.get("direct_u2u_prb_slots", 0.0),
            ),
            dtype=np.float64,
        )
        delta = np.maximum(cumulative - self.last_resources, 0.0)
        if self.synchronous_online:
            interval_delta = np.asarray(
                (
                    payload["interval_bs_uplink_prb_slots"],
                    payload["interval_bs_downlink_prb_slots"],
                    payload["interval_direct_u2u_prb_slots"],
                ),
                dtype=np.float64,
            )
            if not np.allclose(delta, interval_delta, rtol=0.0, atol=1.0e-9):
                raise RuntimeError(
                    "100 ms interval resource counters do not match cumulative deltas"
                )
        self.last_resources = cumulative
        snapshot = self._snapshot(payload)
        self.last_snapshot_terminal = bool(
            snapshot.terminated or snapshot.truncated
        )
        return BackendStep(snapshot, *delta.tolist())
