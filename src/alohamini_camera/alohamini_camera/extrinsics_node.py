from __future__ import annotations

from pathlib import Path

import rclpy
import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


class CameraExtrinsicsNode(Node):
    def __init__(self) -> None:
        super().__init__("alohamini_camera_extrinsics")
        self.declare_parameter("extrinsics_csv", "")
        self.declare_parameter("allow_candidate", False)
        paths = [
            value.strip()
            for value in str(self.get_parameter("extrinsics_csv").value).split(",")
            if value.strip()
        ]
        allow_candidate = bool(self.get_parameter("allow_candidate").value)
        calibration_share = Path(get_package_share_directory("alohamini_calibration"))
        broadcaster = StaticTransformBroadcaster(self)
        transforms = []
        for configured in paths:
            path = Path(configured).expanduser()
            if not path.is_absolute():
                path = calibration_share / "config/cameras/extrinsics" / path
            with path.open(encoding="utf-8") as stream:
                document = yaml.safe_load(stream)
            status = str(document.get("status", ""))
            if not status.startswith("accepted_") and not allow_candidate:
                raise ValueError(f"refusing non-accepted camera extrinsic {path}: {status}")
            transform_data = document.get("T_mount_link_from_camera_optical")
            if not isinstance(transform_data, dict):
                raise ValueError(f"{path} lacks T_mount_link_from_camera_optical")
            xyz = transform_data.get("xyz_m")
            quaternion = transform_data.get("quaternion_xyzw")
            if not isinstance(xyz, list) or len(xyz) != 3:
                raise ValueError(f"{path} xyz_m must contain three values")
            if not isinstance(quaternion, list) or len(quaternion) != 4:
                raise ValueError(f"{path} quaternion_xyzw must contain four values")
            message = TransformStamped()
            message.header.stamp = self.get_clock().now().to_msg()
            message.header.frame_id = str(document["mount_link"])
            message.child_frame_id = str(document["optical_frame"])
            message.transform.translation.x = float(xyz[0])
            message.transform.translation.y = float(xyz[1])
            message.transform.translation.z = float(xyz[2])
            message.transform.rotation.x = float(quaternion[0])
            message.transform.rotation.y = float(quaternion[1])
            message.transform.rotation.z = float(quaternion[2])
            message.transform.rotation.w = float(quaternion[3])
            transforms.append(message)
        if transforms:
            broadcaster.sendTransform(transforms)
        self._broadcaster = broadcaster
        self.get_logger().info(f"Published {len(transforms)} calibrated camera transforms")


def main() -> None:
    rclpy.init()
    node = CameraExtrinsicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
