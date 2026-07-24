"""Probabilistic occupancy and ESDF map port of ``plan_env/sdf_map``."""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import IntEnum
from typing import Iterable

import numpy as np
from numpy.typing import NDArray
from scipy.ndimage import binary_dilation, distance_transform_edt

from .raycast import RayCaster


Vector = NDArray[np.float64]
Index = NDArray[np.int64]


class Occupancy(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OCCUPIED = 2


def _logit(probability: float) -> float:
    return math.log(probability / (1.0 - probability))


@dataclass(slots=True)
class VoxelMapConfig:
    resolution: float = 0.1
    map_size: tuple[float, float, float] = (35.0, 35.0, 3.5)
    ground_height: float = -1.0
    obstacles_inflation: float = 0.199
    local_bound_inflate: float = 0.5
    local_map_margin: int = 50
    default_distance: float = 0.5
    optimistic: bool = True
    signed_distance: bool = False
    p_hit: float = 0.65
    p_miss: float = 0.35
    p_min: float = 0.12
    p_max: float = 0.90
    p_occupied: float = 0.80
    max_ray_length: float = 4.5
    virtual_ceiling_height: float = -10.0
    box_min: tuple[float, float, float] = (-7.0, -15.0, 0.0)
    box_max: tuple[float, float, float] = (7.0, 15.0, 1.7)


class VoxelMap:
    """Dense map retaining RACER's x/y/z indexing and log-odds update rules."""

    def __init__(self, config: VoxelMapConfig) -> None:
        self.config = config
        self.resolution = float(config.resolution)
        self.inverse_resolution = 1.0 / self.resolution
        self.map_origin = np.asarray(
            (
                -config.map_size[0] / 2.0,
                -config.map_size[1] / 2.0,
                config.ground_height,
            ),
            dtype=np.float64,
        )
        self.map_size = np.asarray(config.map_size, dtype=np.float64)
        self.voxel_count = np.ceil(self.map_size / self.resolution).astype(np.int64)
        self.map_min = self.map_origin.copy()
        self.map_max = self.map_origin + self.map_size
        self.box_min = np.asarray(config.box_min, dtype=np.float64)
        self.box_max = np.asarray(config.box_max, dtype=np.float64)
        self.box_min_index = self.position_to_index(self.box_min)
        self.box_max_index = self.position_to_index(self.box_max)

        self.prob_hit_log = _logit(config.p_hit)
        self.prob_miss_log = _logit(config.p_miss)
        self.clamp_min_log = _logit(config.p_min)
        self.clamp_max_log = _logit(config.p_max)
        self.occupied_log = _logit(config.p_occupied)
        self.unknown_flag = 0.01
        shape = tuple(int(value) for value in self.voxel_count)
        self.occupancy = np.full(
            shape, self.clamp_min_log - self.unknown_flag, dtype=np.float64
        )
        self.inflated = np.zeros(shape, dtype=np.uint8)
        self.distance = np.full(shape, config.default_distance, dtype=np.float64)
        self.negative_distance = np.full(shape, config.default_distance, dtype=np.float64)
        self.local_bound_min = np.zeros(3, dtype=np.int64)
        self.local_bound_max = self.voxel_count - 1
        self.update_min = np.full(3, math.inf, dtype=np.float64)
        self.update_max = np.full(3, -math.inf, dtype=np.float64)
        self.all_min = np.full(3, math.inf, dtype=np.float64)
        self.all_max = np.full(3, -math.inf, dtype=np.float64)
        self.ray_caster = RayCaster(self.resolution, self.map_origin)
        self.swarm_transforms: dict[int, Vector] = {}

    def position_to_index(self, position: Vector) -> Index:
        return np.floor(
            (np.asarray(position, dtype=np.float64) - self.map_origin)
            * self.inverse_resolution
        ).astype(np.int64)

    def index_to_position(self, index: Index) -> Vector:
        return (
            (np.asarray(index, dtype=np.float64) + 0.5) * self.resolution
            + self.map_origin
        )

    def bound_index(self, index: Index) -> Index:
        return np.minimum(np.maximum(index, 0), self.voxel_count - 1).astype(np.int64)

    def to_address(self, index: Index) -> int:
        x, y, z = (int(value) for value in index)
        return x * int(self.voxel_count[1] * self.voxel_count[2]) + y * int(
            self.voxel_count[2]
        ) + z

    def address_to_index(self, address: int) -> Index:
        yz = int(self.voxel_count[1] * self.voxel_count[2])
        x, remainder = divmod(int(address), yz)
        y, z = divmod(remainder, int(self.voxel_count[2]))
        return np.asarray((x, y, z), dtype=np.int64)

    def is_in_map(self, value: Vector | Index) -> bool:
        array = np.asarray(value)
        if np.issubdtype(array.dtype, np.integer):
            return bool(np.all(array >= 0) and np.all(array < self.voxel_count))
        return bool(
            np.all(array >= self.map_min + 1.0e-4)
            and np.all(array <= self.map_max - 1.0e-4)
        )

    def is_in_box(self, value: Vector | Index) -> bool:
        array = np.asarray(value)
        if np.issubdtype(array.dtype, np.integer):
            return bool(np.all(array >= self.box_min_index) and np.all(array < self.box_max_index))
        return bool(np.all(array > self.box_min) and np.all(array < self.box_max))

    def bound_box(self, lower: Vector, upper: Vector) -> tuple[Vector, Vector]:
        return np.maximum(lower, self.box_min), np.minimum(upper, self.box_max)

    def get_occupancy(self, value: Vector | Index) -> Occupancy | int:
        index = (
            np.asarray(value, dtype=np.int64)
            if np.issubdtype(np.asarray(value).dtype, np.integer)
            else self.position_to_index(np.asarray(value, dtype=np.float64))
        )
        if not self.is_in_map(index):
            return -1
        log_odds = float(self.occupancy[tuple(index)])
        if log_odds < self.clamp_min_log - 1.0e-3:
            return Occupancy.UNKNOWN
        if log_odds > self.occupied_log:
            return Occupancy.OCCUPIED
        return Occupancy.FREE

    def get_inflated_occupancy(self, value: Vector | Index) -> int:
        index = (
            np.asarray(value, dtype=np.int64)
            if np.issubdtype(np.asarray(value).dtype, np.integer)
            else self.position_to_index(np.asarray(value, dtype=np.float64))
        )
        if not self.is_in_map(index):
            return -1
        return int(self.inflated[tuple(index)])

    def set_occupied(self, position: Vector, occupied: int = 1) -> None:
        if self.is_in_map(position):
            self.inflated[tuple(self.position_to_index(position))] = int(occupied)

    def get_distance(self, value: Vector | Index) -> float:
        index = (
            np.asarray(value, dtype=np.int64)
            if np.issubdtype(np.asarray(value).dtype, np.integer)
            else self.position_to_index(np.asarray(value, dtype=np.float64))
        )
        if not self.is_in_map(index):
            return -1.0
        return float(self.distance[tuple(index)])

    def closest_point_in_map(self, point: Vector, camera: Vector) -> Vector:
        difference = np.asarray(point) - np.asarray(camera)
        maximum = self.map_max - camera
        minimum = self.map_min - camera
        factor = math.inf
        for axis in range(3):
            if abs(difference[axis]) > 0.0:
                for boundary in (maximum[axis], minimum[axis]):
                    candidate = boundary / difference[axis]
                    if 0.0 < candidate < factor:
                        factor = candidate
        return camera + (factor - 1.0e-3) * difference

    def input_point_cloud(self, points: Iterable[Iterable[float]] | Vector, camera: Vector) -> list[int]:
        cloud = np.asarray(points, dtype=np.float64).reshape((-1, 3))
        camera_position = np.asarray(camera, dtype=np.float64)
        if cloud.size == 0:
            return []
        hit_count: dict[tuple[int, int, int], int] = {}
        miss: set[tuple[int, int, int]] = set()
        update_min = camera_position.copy()
        update_max = camera_position.copy()
        processed_endpoints: set[tuple[int, int, int]] = set()

        for raw_point in cloud:
            point = raw_point.copy()
            endpoint_hit = True
            if not self.is_in_map(point):
                point = self.closest_point_in_map(point, camera_position)
                endpoint_hit = False
            length = float(np.linalg.norm(point - camera_position))
            if length > self.config.max_ray_length:
                point = (
                    (point - camera_position) / length * self.config.max_ray_length
                    + camera_position
                )
                endpoint_hit = False
            if point[2] < 0.2 or not self.is_in_map(point):
                continue
            endpoint_index = self.position_to_index(point)
            endpoint_key = tuple(int(item) for item in endpoint_index)
            if endpoint_hit:
                hit_count[endpoint_key] = hit_count.get(endpoint_key, 0) + 1
            else:
                miss.add(endpoint_key)
            update_min = np.minimum(update_min, point)
            update_max = np.maximum(update_max, point)
            if endpoint_key in processed_endpoints:
                continue
            processed_endpoints.add(endpoint_key)
            ray = list(self.ray_caster.indices(point, camera_position))
            for index in ray[1:]:
                if self.is_in_map(index):
                    miss.add(tuple(int(item) for item in index))

        inflation = np.asarray(
            (self.config.local_bound_inflate, self.config.local_bound_inflate, 0.0)
        )
        self.local_bound_min = self.bound_index(self.position_to_index(update_min - inflation))
        self.local_bound_max = self.bound_index(self.position_to_index(update_max + inflation))
        self.update_min = np.minimum(self.update_min, update_min)
        self.update_max = np.maximum(self.update_max, update_max)
        self.all_min = np.minimum(self.all_min, update_min)
        self.all_max = np.maximum(self.all_max, update_max)

        changed: list[int] = []
        for key in set(hit_count).union(miss):
            index = np.asarray(key, dtype=np.int64)
            old = float(self.occupancy[key])
            update = (
                self.prob_hit_log
                if hit_count.get(key, 0) >= (1 if key in miss else 0)
                else self.prob_miss_log
            )
            if old < self.clamp_min_log - 1.0e-3:
                old = self.occupied_log
                changed.append(self.to_address(index))
            self.occupancy[key] = min(
                max(old + update, self.clamp_min_log), self.clamp_max_log
            )
        return changed

    def inflate_local_map(self) -> None:
        step = int(math.ceil(self.config.obstacles_inflation / self.resolution))
        occupied = self.occupancy > self.occupied_log
        if step > 0:
            self.inflated[:] = binary_dilation(
                occupied, structure=np.ones((2 * step + 1,) * 3, dtype=bool)
            )
        else:
            self.inflated[:] = occupied

    def update_esdf(self) -> None:
        obstacle = self.inflated.astype(bool)
        if not self.config.optimistic:
            unknown = self.occupancy < self.clamp_min_log - 1.0e-3
            obstacle = np.logical_or(obstacle, unknown)
        positive = distance_transform_edt(~obstacle) * self.resolution
        if self.config.signed_distance:
            negative = distance_transform_edt(obstacle) * self.resolution
            positive = positive + np.where(
                negative > 0.0, -negative + self.resolution, 0.0
            )
            self.negative_distance[:] = negative
        self.distance[:] = positive

    def get_distance_with_gradient(self, position: Vector) -> tuple[float, Vector]:
        point = np.asarray(position, dtype=np.float64)
        if not self.is_in_map(point):
            return 0.0, np.zeros(3, dtype=np.float64)
        shifted = point - 0.5 * self.resolution
        base = self.bound_index(self.position_to_index(shifted))
        base = np.minimum(base, self.voxel_count - 2)
        base_position = self.index_to_position(base)
        difference = (point - base_position) * self.inverse_resolution
        values = self.distance[
            base[0] : base[0] + 2, base[1] : base[1] + 2, base[2] : base[2] + 2
        ]
        dx, dy, dz = (float(item) for item in difference)
        v00 = (1.0 - dx) * values[0, 0, 0] + dx * values[1, 0, 0]
        v01 = (1.0 - dx) * values[0, 0, 1] + dx * values[1, 0, 1]
        v10 = (1.0 - dx) * values[0, 1, 0] + dx * values[1, 1, 0]
        v11 = (1.0 - dx) * values[0, 1, 1] + dx * values[1, 1, 1]
        v0 = (1.0 - dy) * v00 + dy * v10
        v1 = (1.0 - dy) * v01 + dy * v11
        distance = (1.0 - dz) * v0 + dz * v1
        gradient = np.zeros(3, dtype=np.float64)
        gradient[2] = (v1 - v0) * self.inverse_resolution
        gradient[1] = (
            (1.0 - dz) * (v10 - v00) + dz * (v11 - v01)
        ) * self.inverse_resolution
        gradient[0] = (
            (1.0 - dz) * (1.0 - dy) * (values[1, 0, 0] - values[0, 0, 0])
            + (1.0 - dz) * dy * (values[1, 1, 0] - values[0, 1, 0])
            + dz * (1.0 - dy) * (values[1, 0, 1] - values[0, 0, 1])
            + dz * dy * (values[1, 1, 1] - values[0, 1, 1])
        ) * self.inverse_resolution
        return float(distance), gradient

    def region(self) -> tuple[Vector, Vector]:
        return self.map_origin.copy(), self.map_size.copy()

    def box(self) -> tuple[Vector, Vector]:
        return self.box_min.copy(), self.box_max.copy()

    def updated_box(self, reset: bool = False) -> tuple[Vector, Vector]:
        result = (self.update_min.copy(), self.update_max.copy())
        if reset:
            self.update_min[:] = math.inf
            self.update_max[:] = -math.inf
        return result
