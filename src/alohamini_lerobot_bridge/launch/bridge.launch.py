from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("alohamini_lerobot_bridge"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("arm_mapping_dir", default_value=""),
            DeclareLaunchArgument("state_timestamp_mode", default_value="receipt"),
            DeclareLaunchArgument(
                "max_state_response_age_sec", default_value="0.25"
            ),
            Node(
                package="alohamini_lerobot_bridge",
                executable="bridge_node",
                name="alohamini_lerobot_bridge",
                output="screen",
                parameters=[
                    str(share / "config" / "bridge.yaml"),
                    {
                        "host": LaunchConfiguration("host"),
                        "arm_mapping_dir": LaunchConfiguration("arm_mapping_dir"),
                        "state_timestamp_mode": LaunchConfiguration(
                            "state_timestamp_mode"
                        ),
                        "max_state_response_age_sec": LaunchConfiguration(
                            "max_state_response_age_sec"
                        ),
                    },
                ],
            ),
        ]
    )
