#!/usr/bin/env python3
"""Render an orthographic audit of gripper visual and collision geometry."""

from __future__ import annotations

import argparse
import xml.etree.ElementTree as ET
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import trimesh
from matplotlib.patches import Rectangle


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_URDF = PACKAGE_ROOT / "urdf/alohamini2pro_moveit.urdf"


def _vector(value: str) -> np.ndarray:
    return np.fromstring(value, sep=" ", dtype=float)


def _mesh_path(uri: str) -> Path:
    prefix = "package://alohamini_description/"
    if not uri.startswith(prefix):
        raise ValueError(f"unsupported mesh URI: {uri}")
    return PACKAGE_ROOT / uri.removeprefix(prefix)


def _box_bounds(collision: ET.Element) -> tuple[str, np.ndarray, np.ndarray]:
    origin = collision.find("origin")
    xyz = _vector(origin.get("xyz", "0 0 0")) if origin is not None else np.zeros(3)
    rpy = _vector(origin.get("rpy", "0 0 0")) if origin is not None else np.zeros(3)
    if not np.allclose(rpy, 0.0):
        raise ValueError("this audit currently requires axis-aligned box collisions")
    size = _vector(collision.find("geometry/box").get("size"))
    return collision.get("name", "unnamed"), xyz - size / 2.0, xyz + size / 2.0


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--side", choices=("left", "right"), default="right")
    parser.add_argument("--urdf", type=Path, default=DEFAULT_URDF)
    parser.add_argument("--output", type=Path, default=Path("/tmp/alohamini_gripper_collision_audit.png"))
    args = parser.parse_args()

    root = ET.parse(args.urdf).getroot()
    link = root.find(f"./link[@name='{args.side}_Fixed_Jaw']")
    if link is None:
        raise ValueError(f"missing {args.side}_Fixed_Jaw")
    visual = link.find("visual/geometry/mesh")
    mesh = trimesh.load_mesh(_mesh_path(visual.get("filename")), process=False)
    vertices = np.asarray(mesh.vertices)
    # STL repeats triangle vertices. This is only a projection audit, so a
    # deterministic stride keeps the plot responsive without changing bounds.
    sample = vertices[:: max(1, len(vertices) // 40000)]
    boxes = [
        _box_bounds(collision)
        for collision in link.findall("collision")
        if collision.find("geometry/box") is not None
    ]
    collision_vertices = []
    for collision in link.findall("collision"):
        mesh_element = collision.find("geometry/mesh")
        if mesh_element is None:
            continue
        collision_mesh = trimesh.load_mesh(
            _mesh_path(mesh_element.get("filename")), process=False
        )
        collision_vertices.append(np.asarray(collision_mesh.vertices))
    collision_sample = (
        np.concatenate(collision_vertices)
        if collision_vertices
        else np.empty((0, 3), dtype=float)
    )

    projections = ((0, 1, "x", "y"), (0, 2, "x", "z"), (1, 2, "y", "z"))
    figure, axes = plt.subplots(1, 3, figsize=(18, 6))
    colors = ("tab:red", "tab:blue", "tab:green", "tab:orange")
    for axis, (u, v, u_name, v_name) in zip(axes, projections, strict=True):
        axis.scatter(sample[:, u] * 1000, sample[:, v] * 1000, s=0.5, alpha=0.18, label="visual STL")
        if len(collision_sample):
            axis.scatter(
                collision_sample[:, u] * 1000,
                collision_sample[:, v] * 1000,
                s=4,
                alpha=0.55,
                color="tab:orange",
                label="separate VHACD pieces",
            )
        for color, (name, lower, upper) in zip(colors, boxes, strict=False):
            axis.add_patch(
                Rectangle(
                    lower[[u, v]] * 1000,
                    *(upper[[u, v]] - lower[[u, v]]) * 1000,
                    fill=False,
                    edgecolor=color,
                    linewidth=2,
                    label=name,
                )
            )
        axis.set_xlabel(f"{u_name} (mm)")
        axis.set_ylabel(f"{v_name} (mm)")
        axis.set_aspect("equal", adjustable="box")
        axis.grid(alpha=0.2)
        axis.legend(loc="best", fontsize=8)
    figure.suptitle(f"{args.side}_Fixed_Jaw: visual CAD vs planning collision geometry")
    figure.tight_layout()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(args.output, dpi=180)
    print(args.output)


if __name__ == "__main__":
    main()
