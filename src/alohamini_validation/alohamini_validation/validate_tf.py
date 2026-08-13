from __future__ import annotations

import math
import time
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener


def translation(transform) -> tuple[float, float, float]:
    value = transform.transform.translation
    return float(value.x), float(value.y), float(value.z)


def main() -> None:
    description = Path(get_package_share_directory("alohamini_description"))
    root = ET.parse(description / "urdf/alohamini2pro_moveit.urdf").getroot()
    frames = {link.get("name") for link in root.findall("link")}

    rclpy.init()
    node = Node("alohamini_tf_validation")
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    deadline = time.monotonic() + 8.0
    missing = set(frames) - {"root"}
    while missing and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)
        missing = {
            frame
            for frame in missing
            if not buffer.can_transform("root", frame, rclpy.time.Time(), timeout=Duration(seconds=0.0))
        }
    try:
        assert not missing, f"missing TF frames: {sorted(missing)}"
        left = translation(buffer.lookup_transform("root", "left_tcp", rclpy.time.Time()))
        right = translation(buffer.lookup_transform("root", "right_tcp", rclpy.time.Time()))
        assert all(math.isfinite(value) for value in (*left, *right))
        assert math.isclose(left[1], right[1], abs_tol=1e-6)
        assert math.isclose(left[2], right[2], abs_tol=1e-6)
        print(f"[PASS] TF tree: {len(frames)} frames, root -> both TCPs available")
        print(f"       left_tcp={left}, right_tcp={right}")
    finally:
        del listener
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
