#!/usr/bin/env python3
"""Generate deterministic VHACD pieces for the symmetric AlohaMini arm links."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path

import numpy as np
import trimesh


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
VISUAL_DIR = PACKAGE_ROOT / "meshes" / "visual"
OUTPUT_DIR = PACKAGE_ROOT / "meshes" / "collision"

LINKS = {
    "arm_base": ("left_Base.STL", 8),
    "rotation_pitch": ("left_Rotation_Pitch.STL", 6),
    "upper_arm": ("left_Upper_Arm.STL", 8),
    "lower_arm": ("left_Lower_Arm.STL", 8),
    "wrist_pitch_roll": ("left_Wrist_Pitch_Roll.STL", 4),
    "wrist_yaw": ("left_wrist_yaw.STL", 6),
    "fixed_jaw": ("left_Fixed_Jaw.STL", 8),
    "wrist_camera": ("left_camera.STL", 2),
}

COMMON_OPTIONS = {
    "resolution": 100000,
    "minimumVolumePercentErrorAllowed": 2.0,
    "maxNumVerticesPerCH": 32,
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as file:
        for chunk in iter(lambda: file.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--report-json",
        type=Path,
        default=PACKAGE_ROOT / "config" / "kinematics" / "arm_collision_report.json",
    )
    parser.add_argument(
        "--split-existing-fixed-jaw",
        action="store_true",
        help=(
            "split the reviewed combined fixed_jaw_vhacd.stl into its existing "
            "connected convex components without running VHACD"
        ),
    )
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    if args.split_existing_fixed_jaw:
        destination = OUTPUT_DIR / "fixed_jaw_vhacd.stl"
        pieces = list(
            trimesh.load_mesh(destination, process=True).split(only_watertight=False)
        )
        pieces.sort(key=lambda part: tuple(np.round(part.centroid[[1, 2, 0]], 9)))
        if len(pieces) != 8:
            raise RuntimeError(
                f"expected 8 reviewed Fixed_Jaw components, found {len(pieces)}"
            )
        for stale in OUTPUT_DIR.glob("fixed_jaw_vhacd_*.stl"):
            stale.unlink()
        piece_files = []
        for index, piece in enumerate(pieces):
            piece_path = OUTPUT_DIR / f"fixed_jaw_vhacd_{index:02d}.stl"
            piece.export(piece_path, file_type="stl")
            piece_files.append({"file": piece_path.name, "sha256": sha256(piece_path)})

        report = json.loads(args.report_json.read_text(encoding="utf-8"))
        fixed_report = report["links"]["fixed_jaw"]
        fixed_report["piece_files"] = piece_files
        args.report_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(f"split {destination.name}: {len(pieces)} existing convex components")
        print(f"wrote {args.report_json}")
        return

    report = {"schema_version": 1, "generator": "trimesh_vhacdx", "links": {}}

    for slug, (source_name, max_hulls) in LINKS.items():
        source = VISUAL_DIR / source_name
        opposite = VISUAL_DIR / source_name.replace("left_", "right_", 1)
        if not opposite.is_file() or sha256(source) != sha256(opposite):
            raise RuntimeError(f"left/right CAD mismatch for {slug}")

        mesh = trimesh.load_mesh(source, process=True)
        options = {**COMMON_OPTIONS, "maxConvexHulls": max_hulls}
        pieces = [
            trimesh.Trimesh(**item, process=True)
            for item in trimesh.decomposition.convex_decomposition(mesh, **options)
        ]
        pieces.sort(key=lambda part: tuple(np.round(part.centroid[[1, 2, 0]], 9)))

        records = []
        for piece in pieces:
            records.append(
                {
                    "faces": int(len(piece.faces)),
                    "vertices": int(len(piece.vertices)),
                    "volume_cm3": float(abs(piece.volume) * 1e6),
                    "bounds_mm": np.asarray(piece.bounds * 1000.0).tolist(),
                }
            )

        combined = trimesh.util.concatenate(pieces)
        destination = OUTPUT_DIR / f"{slug}_vhacd.stl"
        combined.export(destination, file_type="stl")
        for stale in OUTPUT_DIR.glob(f"{slug}_vhacd_*.stl"):
            stale.unlink()

        piece_files = []
        # Fixed_Jaw is deeply concave: a single collision mesh containing all
        # disconnected VHACD pieces may be re-convexified by downstream
        # consumers and bridge the open gripper aperture.  Export its convex
        # components separately so URDF/SRDF consumers retain the decomposition.
        if slug == "fixed_jaw":
            for index, piece in enumerate(pieces):
                piece_path = OUTPUT_DIR / f"{slug}_vhacd_{index:02d}.stl"
                piece.export(piece_path, file_type="stl")
                piece_files.append(
                    {
                        "file": piece_path.name,
                        "sha256": sha256(piece_path),
                    }
                )

        report["links"][slug] = {
            "source": f"meshes/visual/{source_name}",
            "source_sha256": sha256(source),
            "left_right_source_identical": True,
            "source_faces": int(len(mesh.faces)),
            "source_volume_cm3": float(abs(mesh.volume) * 1e6),
            "single_convex_hull_volume_cm3": float(abs(mesh.convex_hull.volume) * 1e6),
            "vhacd_options": options,
            "combined_file": destination.name,
            "combined_sha256": sha256(destination),
            "combined_faces": int(len(combined.faces)),
            "piece_count": len(records),
            "piece_volume_total_cm3": float(sum(item["volume_cm3"] for item in records)),
            "piece_files": piece_files,
            "pieces": records,
        }
        print(f"generated {destination.name}: {len(records)} hulls", flush=True)

    args.report_json.parent.mkdir(parents=True, exist_ok=True)
    args.report_json.write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
    print(f"wrote {args.report_json}", flush=True)


if __name__ == "__main__":
    main()
