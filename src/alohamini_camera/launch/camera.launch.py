from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("alohamini_camera"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("port", default_value="5557"),
            DeclareLaunchArgument("publish_raw", default_value="true"),
            DeclareLaunchArgument("timestamp_mode", default_value="receipt"),
            DeclareLaunchArgument("enable_extrinsics", default_value="false"),
            DeclareLaunchArgument("extrinsics", default_value=""),
            DeclareLaunchArgument("allow_candidate_extrinsics", default_value="false"),
            Node(
                package="alohamini_camera",
                executable="camera_node",
                name="alohamini_camera",
                output="screen",
                parameters=[
                    str(share / "config/camera.yaml"),
                    {
                        "host": LaunchConfiguration("host"),
                        "port": LaunchConfiguration("port"),
                        "publish_raw": LaunchConfiguration("publish_raw"),
                        "timestamp_mode": LaunchConfiguration("timestamp_mode"),
                    },
                ],
            ),
            Node(
                package="alohamini_camera",
                executable="extrinsics_node",
                name="alohamini_camera_extrinsics",
                output="screen",
                condition=IfCondition(LaunchConfiguration("enable_extrinsics")),
                parameters=[
                    {
                        "extrinsics_csv": LaunchConfiguration("extrinsics"),
                        "allow_candidate": LaunchConfiguration(
                            "allow_candidate_extrinsics"
                        ),
                    }
                ],
            ),
        ]
    )
