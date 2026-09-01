#!/usr/bin/env python3
"""Render an XY-orthographic overview directly from Sionna PLY geometry."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import struct

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection, PolyCollection
from matplotlib.lines import Line2D
import numpy as np


MATERIAL_COLORS = {
    "concrete": "#777777",
    "metal": "#477a9f",
    "wood": "#9a6338",
    "chipboard": "#c49355",
    "ceiling_board": "#b8b8b8",
    "plywood": "#d6ae73",
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mesh-dir", type=Path, required=True)
    parser.add_argument("--points", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--z-min", type=float, default=0.25)
    parser.add_argument("--z-max", type=float, default=8.5)
    parser.add_argument(
        "--ap-position", type=float, nargs=3,
        default=(-10.02891489217081, 14.888611215255622, 7.55),
    )
    parser.add_argument("--site-label", default="Industrial AP")
    parser.add_argument("--scene-label", default="Warehouse Full with Industrial AP")
    return parser.parse_args()


def read_binary_ply(path: Path) -> tuple[np.ndarray, np.ndarray]:
    """Read the binary little-endian triangle PLY files made by the converter."""
    with path.open("rb") as stream:
        header_lines = []
        while True:
            line = stream.readline()
            if not line:
                raise ValueError(f"incomplete PLY header: {path}")
            decoded = line.decode("ascii").strip()
            header_lines.append(decoded)
            if decoded == "end_header":
                break
        if "format binary_little_endian 1.0" not in header_lines:
            raise ValueError(f"unsupported PLY encoding: {path}")
        vertex_count = int(
            next(line.split()[2] for line in header_lines if line.startswith("element vertex "))
        )
        face_count = int(
            next(line.split()[2] for line in header_lines if line.startswith("element face "))
        )
        vertex_data = stream.read(vertex_count * 12)
        vertices = np.frombuffer(vertex_data, dtype="<f4").reshape(vertex_count, 3).copy()
        faces = []
        for _ in range(face_count):
            count_raw = stream.read(1)
            if not count_raw:
                raise ValueError(f"truncated PLY face data: {path}")
            count = count_raw[0]
            indices = struct.unpack(f"<{count}I", stream.read(4 * count))
            if count == 3:
                faces.append(indices)
            elif count > 3:
                faces.extend((indices[0], indices[i], indices[i + 1]) for i in range(1, count - 1))
    return vertices, np.asarray(faces, dtype=np.int64)


def material_name(path: Path) -> str:
    return path.stem.removeprefix("warehouse_")


def load_projected_geometry(
    mesh_dir: Path, z_min: float, z_max: float
) -> list[tuple[str, np.ndarray]]:
    projected = []
    for path in sorted(mesh_dir.glob("*.ply")):
        vertices, faces = read_binary_ply(path)
        triangles = vertices[faces]
        intersects = (triangles[:, :, 2].max(axis=1) >= z_min) & (
            triangles[:, :, 2].min(axis=1) <= z_max
        )
        projected.append((material_name(path), triangles[intersects, :, :2]))
    if not projected:
        raise RuntimeError(f"no PLY meshes found in {mesh_dir}")
    return projected


def draw_geometry(axis, geometry, bounds) -> None:
    x_min, x_max, y_min, y_max = bounds
    for name, triangles in geometry:
        color = MATERIAL_COLORS.get(name, "#777777")
        # Filled horizontal/oblique surfaces communicate obstacle footprints.
        twice_area = np.abs(
            (triangles[:, 1, 0] - triangles[:, 0, 0])
            * (triangles[:, 2, 1] - triangles[:, 0, 1])
            - (triangles[:, 2, 0] - triangles[:, 0, 0])
            * (triangles[:, 1, 1] - triangles[:, 0, 1])
        )
        filled = triangles[twice_area > 1.0e-5]
        if len(filled):
            axis.add_collection(
                PolyCollection(filled, facecolors=color, edgecolors="none", alpha=0.72)
            )
        # Edges retain thin walls whose XY projection has near-zero area.
        edges = np.concatenate(
            [triangles[:, [0, 1]], triangles[:, [1, 2]], triangles[:, [2, 0]]], axis=0
        )
        axis.add_collection(
            LineCollection(edges, colors=color, linewidths=0.22, alpha=0.52)
        )
    axis.set_xlim(x_min, x_max)
    axis.set_ylim(y_min, y_max)
    axis.set_aspect("equal", adjustable="box")
    axis.set_xlabel("x (m)")
    axis.set_ylabel("y (m)")
    axis.set_xticks(np.arange(np.ceil(x_min / 5) * 5, x_max + 0.1, 5))
    axis.set_yticks(np.arange(np.ceil(y_min / 5) * 5, y_max + 0.1, 5))
    axis.grid(color="white", linewidth=0.45, alpha=0.42)
    axis.set_facecolor("#f4f1ea")


def draw_ap(axis, ap_position, site_label: str) -> None:
    axis.scatter(
        [ap_position[0]], [ap_position[1]], marker="D", s=85,
        c="#e31a1c", edgecolors="white", linewidths=1.0, zorder=9,
    )
    axis.annotate(
        f"{site_label}\n({ap_position[0]:.1f}, {ap_position[1]:.1f})",
        (ap_position[0], ap_position[1]), xytext=(7, 7),
        textcoords="offset points", fontsize=8, color="#9d1113", zorder=10,
    )


def draw_points(axis, points) -> None:
    positions = np.asarray([point["position"] for point in points])
    axis.scatter(
        positions[:, 0], positions[:, 1], marker="*", s=62,
        c="white", edgecolors="black", linewidths=0.65, zorder=8,
    )
    for index, position in enumerate(positions, start=1):
        axis.annotate(
            f"P{index:02d}", position[:2], xytext=(3, 3),
            textcoords="offset points", fontsize=6.5, weight="bold", zorder=10,
        )


def legend(axis, site_label: str) -> None:
    handles = [
        Line2D([0], [0], marker="s", linestyle="", markersize=8,
               markerfacecolor=color, markeredgecolor="none", label=name.replace("_", " "))
        for name, color in MATERIAL_COLORS.items()
    ]
    handles.extend(
        [
            Line2D([0], [0], marker="D", linestyle="", markersize=7,
                   markerfacecolor="#e31a1c", markeredgecolor="white", label=site_label),
            Line2D([0], [0], marker="*", linestyle="", markersize=10,
                   markerfacecolor="white", markeredgecolor="black", label="UAV TX sample"),
        ]
    )
    axis.legend(handles=handles, loc="upper right", fontsize=7, framealpha=0.9, ncol=2)


def main() -> None:
    args = parse_args()
    selection = json.loads(args.points.read_text(encoding="utf-8"))
    bounds = [float(value) for value in selection["bounds_m"]]
    geometry = load_projected_geometry(args.mesh_dir, args.z_min, args.z_max)
    args.output.mkdir(parents=True, exist_ok=True)

    figure, axis = plt.subplots(figsize=(10.5, 10), constrained_layout=True)
    draw_geometry(axis, geometry, bounds)
    draw_ap(axis, args.ap_position, args.site_label)
    axis.set_title(
        f"{args.scene_label} — orthographic scene top view\n"
        f"Radio-map bounds: x=[{bounds[0]:g}, {bounds[1]:g}] m, "
        f"y=[{bounds[2]:g}, {bounds[3]:g}] m"
    )
    legend(axis, args.site_label)
    figure.savefig(args.output / "scene_top_view_radio_map_bounds.png", dpi=220)
    plt.close(figure)

    figure, axes = plt.subplots(2, 3, figsize=(16, 11.5), constrained_layout=True)
    clean_axis = axes.flat[0]
    draw_geometry(clean_axis, geometry, bounds)
    draw_ap(clean_axis, args.ap_position, args.site_label)
    clean_axis.set_title("Scene geometry and radio-map bounds")
    legend(clean_axis, args.site_label)
    for axis, height_entry in zip(axes.flat[1:], selection["heights"]):
        draw_geometry(axis, geometry, bounds)
        draw_ap(axis, args.ap_position, args.site_label)
        draw_points(axis, height_entry["points"])
        axis.set_title(
            f"Actual P01–P{len(height_entry['points']):02d} UAV TX positions | "
            f"z={height_entry['height_m']:g} m"
        )
    figure.suptitle(
        f"{args.scene_label} — scene/radio-map correspondence\n"
        f"{args.site_label} is active in BS-UAV maps and inactive in UAV-UAV maps",
        fontsize=15,
    )
    figure.savefig(args.output / "scene_top_view_sampling_points_all_heights.png", dpi=220)
    plt.close(figure)

    print(f"saved scene top views to {args.output}")


if __name__ == "__main__":
    main()
