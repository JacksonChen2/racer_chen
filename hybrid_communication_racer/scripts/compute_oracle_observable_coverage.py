#!/usr/bin/env python3
"""Build a ground-truth observability mask and evaluate exploration coverage.

The oracle denominator contains free voxels and first-hit surface voxels that
have line of sight to at least one collision-free pose connected to a supplied
start position. Solid interiors and sealed/unreachable cavities are excluded.

Run this with Isaac Sim's Python, which already provides trimesh, scipy and
numba in this repository's portable environment.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
from typing import Any

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import trimesh
from numba import njit, prange
from scipy import ndimage


DEFAULT_BOX_MIN = (-27.0, 0.6, 0.4)
DEFAULT_BOX_MAX = (6.0, 30.6, 8.4)
DEFAULT_RESOLUTION = 0.1
DEFAULT_VEHICLE_RADIUS = 0.332
DEFAULT_SENSOR_RANGE = 4.5


def _sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _parse_vec3(value: str) -> np.ndarray:
    fields = value.split(",")
    if len(fields) != 3:
        raise argparse.ArgumentTypeError("expected x,y,z")
    return np.asarray([float(field) for field in fields], dtype=np.float64)


def _voxelize_truth(
    mesh_dir: Path,
    box_min: np.ndarray,
    shape: np.ndarray,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray, list[dict[str, Any]]]:
    solid = np.zeros(tuple(shape), dtype=np.bool_)
    surface = np.zeros(tuple(shape), dtype=np.bool_)
    mesh_records: list[dict[str, Any]] = []

    mesh_paths = sorted(mesh_dir.glob("*.ply"))
    if not mesh_paths:
        raise FileNotFoundError(f"no PLY meshes found in {mesh_dir}")

    for path in mesh_paths:
        mesh = trimesh.load(path, force="mesh", process=False)
        surface_grid = mesh.voxelized(resolution)
        surface_points = surface_grid.points
        surface_indices = np.floor(
            (surface_points - box_min) / resolution + 1.0e-7
        ).astype(np.int32)
        surface_valid = np.all(
            (surface_indices >= 0) & (surface_indices < shape), axis=1
        )
        surface_indices = surface_indices[surface_valid]
        surface[
            surface_indices[:, 0],
            surface_indices[:, 1],
            surface_indices[:, 2],
        ] = True

        # ``VoxelGrid.fill`` alone is incorrect for a warehouse enclosure: it
        # cannot distinguish material volume from an enclosed air cavity. Use
        # it only to bound candidate points, then apply the mesh parity test.
        # Surface cells are restored explicitly because points exactly on a
        # triangle have numerically ambiguous inside/outside parity.
        surface_grid.fill()
        filled_points = surface_grid.points
        inside_parts = []
        for begin in range(0, len(filled_points), 50_000):
            inside_parts.append(mesh.contains(filled_points[begin : begin + 50_000]))
        inside = (
            np.concatenate(inside_parts)
            if inside_parts
            else np.empty(0, dtype=np.bool_)
        )
        solid_points = filled_points[inside]
        solid_indices = np.floor(
            (solid_points - box_min) / resolution + 1.0e-7
        ).astype(np.int32)
        solid_valid = np.all(
            (solid_indices >= 0) & (solid_indices < shape), axis=1
        )
        solid_indices = solid_indices[solid_valid]
        solid[
            solid_indices[:, 0], solid_indices[:, 1], solid_indices[:, 2]
        ] = True

        mesh_records.append(
            {
                "file": str(path.resolve()),
                "sha256": _sha256(path),
                "vertices": int(len(mesh.vertices)),
                "faces": int(len(mesh.faces)),
                "watertight": bool(mesh.is_watertight),
                "surface_voxels_in_box_before_union": int(
                    len(surface_indices)
                ),
                "inside_voxels_in_box_before_union": int(len(solid_indices)),
            }
        )

    solid |= surface
    return solid, surface, mesh_records


def _reachable_poses(
    solid: np.ndarray,
    starts: np.ndarray,
    box_min: np.ndarray,
    resolution: float,
    vehicle_radius: float,
    maximum_seed_snap: float,
) -> tuple[np.ndarray, list[dict[str, Any]], int, int]:
    clearance_m = ndimage.distance_transform_edt(~solid) * resolution
    legal = (~solid) & (clearance_m + 1.0e-9 >= vehicle_radius)
    labels, component_count = ndimage.label(
        legal, structure=np.ones((3, 3, 3), dtype=np.uint8)
    )

    # Return the closest legal voxel for starts that fall on a discretization
    # boundary. This is a voxelization correction, not a new reachable seed.
    _, closest_legal = ndimage.distance_transform_edt(
        ~legal, return_indices=True
    )
    seed_labels: set[int] = set()
    start_records: list[dict[str, Any]] = []
    shape = np.asarray(solid.shape, dtype=np.int32)
    for start in starts:
        raw_index = np.floor((start - box_min) / resolution).astype(np.int32)
        clipped = np.minimum(np.maximum(raw_index, 0), shape - 1)
        snapped = clipped.copy()
        if not legal[tuple(snapped)]:
            snapped = closest_legal[(slice(None),) + tuple(snapped)]
        snapped = np.asarray(snapped, dtype=np.int32)
        snapped_position = box_min + (snapped.astype(float) + 0.5) * resolution
        snap_distance = float(np.linalg.norm(snapped_position - start))
        label = int(labels[tuple(snapped)])
        seed_valid = label > 0 and snap_distance <= maximum_seed_snap
        if seed_valid:
            seed_labels.add(label)
        start_records.append(
            {
                "position_m": start.tolist(),
                "raw_voxel_index": raw_index.tolist(),
                "seed_voxel_index": snapped.tolist(),
                "seed_position_m": snapped_position.tolist(),
                "snap_distance_m": snap_distance,
                "component_label": label if seed_valid else 0,
                "valid_reachability_seed": seed_valid,
            }
        )

    if not seed_labels:
        raise RuntimeError("none of the start positions maps to a legal pose")
    reachable = np.isin(labels, np.asarray(sorted(seed_labels), dtype=labels.dtype))
    return reachable, start_records, int(component_count), len(seed_labels)


@njit(parallel=True, cache=True)
def _line_of_sight_to_nearest_pose(
    solid: np.ndarray,
    surface: np.ndarray,
    reachable: np.ndarray,
    nearest_x: np.ndarray,
    nearest_y: np.ndarray,
    nearest_z: np.ndarray,
    distance_voxels: np.ndarray,
    max_range_voxels: float,
) -> np.ndarray:
    nx, ny, nz = solid.shape
    output = np.zeros(solid.shape, dtype=np.uint8)
    plane = ny * nz
    total = nx * plane
    for linear in prange(total):
        x = linear // plane
        rem = linear - x * plane
        y = rem // nz
        z = rem - y * nz

        # Interior solid voxels can never be in the denominator. Surface
        # voxels remain candidates, because they are valid first ray hits.
        if solid[x, y, z] and not surface[x, y, z]:
            continue
        if reachable[x, y, z]:
            output[x, y, z] = 1
            continue
        if distance_voxels[x, y, z] > max_range_voxels + 1.0e-6:
            continue

        sx = int(nearest_x[x, y, z])
        sy = int(nearest_y[x, y, z])
        sz = int(nearest_z[x, y, z])
        dx = x - sx
        dy = y - sy
        dz = z - sz
        steps = max(abs(dx), abs(dy), abs(dz))
        if steps == 0:
            output[x, y, z] = 1
            continue

        blocked = False
        last_x = sx
        last_y = sy
        last_z = sz
        # Supercover-like dense sampling at half a voxel prevents diagonal
        # rays from slipping through a one-voxel wall.
        samples = 2 * steps
        for sample in range(1, samples):
            fraction = sample / samples
            qx = int(math.floor(sx + fraction * dx + 0.5))
            qy = int(math.floor(sy + fraction * dy + 0.5))
            qz = int(math.floor(sz + fraction * dz + 0.5))
            if qx == last_x and qy == last_y and qz == last_z:
                continue
            last_x, last_y, last_z = qx, qy, qz
            if solid[qx, qy, qz]:
                blocked = True
                break
        if not blocked:
            output[x, y, z] = 1
    return output


def _build_observable_mask(
    solid: np.ndarray,
    surface: np.ndarray,
    reachable: np.ndarray,
    resolution: float,
    sensor_range: float,
) -> np.ndarray:
    distance, nearest = ndimage.distance_transform_edt(
        ~reachable, return_indices=True
    )
    return _line_of_sight_to_nearest_pose(
        solid,
        surface,
        reachable,
        nearest[0],
        nearest[1],
        nearest[2],
        distance.astype(np.float32),
        sensor_range / resolution,
    ).astype(np.bool_)


def _load_known_mask(path: Path, shape: tuple[int, ...]) -> np.ndarray:
    payload = np.load(path)
    if "known_mask" in payload:
        known = np.asarray(payload["known_mask"], dtype=np.bool_)
    elif "known_mask_packed" in payload and "known_mask_shape" in payload:
        packed = np.asarray(payload["known_mask_packed"], dtype=np.uint8)
        mask_shape = tuple(int(value) for value in payload["known_mask_shape"])
        bitorder = str(payload["bitorder"]) if "bitorder" in payload else "big"
        known = np.unpackbits(
            packed, count=int(np.prod(mask_shape)), bitorder=bitorder
        ).reshape(mask_shape).astype(np.bool_)
    else:
        raise KeyError(
            f"{path} must contain known_mask or known_mask_packed/known_mask_shape"
        )
    if known.shape != shape:
        raise ValueError(f"known mask shape {known.shape} != oracle shape {shape}")
    return known


def _save_packed_masks(
    path: Path,
    solid: np.ndarray,
    surface: np.ndarray,
    reachable: np.ndarray,
    observable: np.ndarray,
    box_min: np.ndarray,
    box_max: np.ndarray,
    resolution: float,
) -> None:
    np.savez_compressed(
        path,
        shape=np.asarray(solid.shape, dtype=np.int32),
        box_min=box_min,
        box_max=box_max,
        resolution=np.asarray(resolution),
        bitorder=np.asarray("little"),
        solid_mask_packed=np.packbits(solid.reshape(-1), bitorder="little"),
        surface_mask_packed=np.packbits(surface.reshape(-1), bitorder="little"),
        reachable_pose_mask_packed=np.packbits(
            reachable.reshape(-1), bitorder="little"
        ),
        observable_mask_packed=np.packbits(
            observable.reshape(-1), bitorder="little"
        ),
    )


def _plot_summary(
    path: Path,
    solid: np.ndarray,
    reachable: np.ndarray,
    observable: np.ndarray,
    box_min: np.ndarray,
    box_max: np.ndarray,
    starts: np.ndarray,
) -> None:
    extent = [box_min[0], box_max[0], box_min[1], box_max[1]]
    eligible = ~solid
    observable_free = observable & eligible
    column_denominator = eligible.sum(axis=2)
    column_fraction = np.divide(
        observable_free.sum(axis=2),
        column_denominator,
        out=np.zeros_like(column_denominator, dtype=float),
        where=column_denominator > 0,
    )
    middle = solid.shape[2] // 2

    fig, axes = plt.subplots(2, 2, figsize=(12, 10), constrained_layout=True)
    panels = (
        (solid.max(axis=2), "Truth solid projection", "gray", 0, 1),
        (reachable.max(axis=2), "Reachable UAV-centre projection", "Blues", 0, 1),
        (column_fraction, "Observable free fraction by vertical column", "viridis", 0, 1),
        (observable[:, :, middle], "Observable mask at middle height", "magma", 0, 1),
    )
    for axis, (data, title, cmap, vmin, vmax) in zip(axes.flat, panels):
        image = axis.imshow(
            data.T,
            origin="lower",
            extent=extent,
            cmap=cmap,
            vmin=vmin,
            vmax=vmax,
            interpolation="nearest",
            aspect="equal",
        )
        axis.scatter(starts[:, 0], starts[:, 1], c="red", s=12, label="starts")
        axis.set_title(title)
        axis.set_xlabel("x [m]")
        axis.set_ylabel("y [m]")
        fig.colorbar(image, ax=axis, shrink=0.8)
    fig.savefig(path, dpi=180)
    plt.close(fig)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--layout-json", type=Path, required=True)
    parser.add_argument("--result-json", type=Path)
    parser.add_argument("--known-mask-npz", type=Path)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--box-min", type=_parse_vec3, default=np.asarray(DEFAULT_BOX_MIN))
    parser.add_argument("--box-max", type=_parse_vec3, default=np.asarray(DEFAULT_BOX_MAX))
    parser.add_argument("--resolution", type=float, default=DEFAULT_RESOLUTION)
    parser.add_argument("--vehicle-radius", type=float, default=DEFAULT_VEHICLE_RADIUS)
    parser.add_argument("--sensor-range", type=float, default=DEFAULT_SENSOR_RANGE)
    parser.add_argument(
        "--maximum-seed-snap",
        type=float,
        help="maximum metric correction from a start to a legal voxel",
    )
    args = parser.parse_args()

    box_min = np.asarray(args.box_min, dtype=np.float64)
    box_max = np.asarray(args.box_max, dtype=np.float64)
    shape_float = (box_max - box_min) / args.resolution
    shape = np.rint(shape_float).astype(np.int32)
    if not np.allclose(shape_float, shape, atol=1.0e-8):
        raise ValueError("box dimensions must be an integer number of voxels")

    layout = json.loads(args.layout_json.read_text())
    starts = np.asarray(layout["start_positions"], dtype=np.float64)
    if starts.ndim != 2 or starts.shape[1] != 3:
        raise ValueError("layout start_positions must be an Nx3 array")
    maximum_seed_snap = (
        args.maximum_seed_snap
        if args.maximum_seed_snap is not None
        else math.sqrt(3.0) * args.resolution * 1.01
    )

    args.output_dir.mkdir(parents=True, exist_ok=True)
    solid, surface, mesh_records = _voxelize_truth(
        args.mesh_dir, box_min, shape, args.resolution
    )
    reachable, start_records, component_count, seeded_component_count = (
        _reachable_poses(
            solid,
            starts,
            box_min,
            args.resolution,
            args.vehicle_radius,
            maximum_seed_snap,
        )
    )
    observable = _build_observable_mask(
        solid, surface, reachable, args.resolution, args.sensor_range
    )

    masks_path = args.output_dir / "warehouse_full_oracle_masks.npz"
    report_path = args.output_dir / "oracle_observable_coverage.json"
    plot_path = args.output_dir / "warehouse_full_oracle_summary.png"
    _save_packed_masks(
        masks_path,
        solid,
        surface,
        reachable,
        observable,
        box_min,
        box_max,
        args.resolution,
    )
    _plot_summary(
        plot_path, solid, reachable, observable, box_min, box_max, starts
    )

    total_count = int(solid.size)
    observable_count = int(observable.sum())
    report: dict[str, Any] = {
        "method": "ground_truth_voxel_oracle_nearest_reachable_line_of_sight",
        "strict_known_mask_intersection": False,
        "grid": {
            "box_min_m": box_min.tolist(),
            "box_max_m": box_max.tolist(),
            "resolution_m": args.resolution,
            "shape": shape.tolist(),
            "total_voxels": total_count,
        },
        "oracle_parameters": {
            "vehicle_collision_radius_m": args.vehicle_radius,
            "maximum_virtual_ray_range_m": args.sensor_range,
            "orientation_model": "omnidirectional; a legal pose may aim at any target",
            "reachability_connectivity": 26,
            "reachability_seed_count": int(len(starts)),
            "maximum_seed_snap_m": maximum_seed_snap,
            "seeded_legal_component_count": seeded_component_count,
            "all_legal_component_count": component_count,
            "minimum_sensor_range_treatment": (
                "ignored because a legal pose can back away within the same dense "
                "reachable component"
            ),
            "visibility_approximation": (
                "line of sight to the nearest reachable pose; conservative in "
                "rare concave cells visible only from a farther pose"
            ),
        },
        "truth_counts": {
            "solid_voxels": int(solid.sum()),
            "surface_voxels": int(surface.sum()),
            "free_voxels": int((~solid).sum()),
            "legal_pose_voxels_all_components": int(
                ((~solid) & (ndimage.distance_transform_edt(~solid) * args.resolution >= args.vehicle_radius)).sum()
            ),
            "reachable_legal_pose_voxels": int(reachable.sum()),
            "oracle_observable_voxels": observable_count,
            "oracle_excluded_voxels": total_count - observable_count,
            "oracle_denominator_fraction_of_raw_box": observable_count / total_count,
        },
        "starts": start_records,
        "meshes": mesh_records,
        "artifacts": {
            "packed_masks_npz": str(masks_path.resolve()),
            "summary_png": str(plot_path.resolve()),
        },
    }

    if args.result_json is not None:
        result = json.loads(args.result_json.read_text())
        counts = result["metrics"]["mapping_coverage_counts_per_agent"]
        best = max(counts, key=lambda item: int(item["known_voxels"]))
        known_count = int(best["known_voxels"])
        evaluation: dict[str, Any] = {
            "source_result_json": str(args.result_json.resolve()),
            "source_result_sha256": _sha256(args.result_json),
            "best_peer_fused_known_voxels": known_count,
            "raw_planning_box_coverage": known_count / total_count,
        }
        if args.known_mask_npz is not None:
            known = _load_known_mask(args.known_mask_npz, tuple(shape))
            numerator = int((known & observable).sum())
            evaluation.update(
                {
                    "strict_known_mask_intersection": True,
                    "observable_known_voxels": numerator,
                    "observable_coverage": numerator / observable_count,
                    "known_voxels_outside_oracle": int((known & ~observable).sum()),
                    "known_mask_npz": str(args.known_mask_npz.resolve()),
                }
            )
            report["strict_known_mask_intersection"] = True
        else:
            # Every real camera ray begins at a physically legal executed pose,
            # so a comprehensive oracle should contain its free cells and first
            # hit. The old result stores counts only, hence this cannot audit the
            # membership assumption voxel by voxel.
            evaluation.update(
                {
                    "strict_known_mask_intersection": False,
                    "observable_known_voxels_assuming_subset": known_count,
                    "observable_coverage_assuming_known_subset": known_count
                    / observable_count,
                    "limitation": (
                        "the historical result has no final known-voxel mask; "
                        "the numerator assumes its known voxels are a subset of "
                        "the oracle-observable mask"
                    ),
                }
            )
        report["experiment_evaluation"] = evaluation

    report_path.write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
