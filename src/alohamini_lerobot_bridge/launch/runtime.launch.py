from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    bridge_share = Path(get_package_share_directory("alohamini_lerobot_bridge"))
    description_share = Path(get_package_share_directory("alohamini_description"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(description_share / "launch" / "description.launch.py")
                )
            ),
            Node(
                package="alohamini_lerobot_bridge",
                executable="bridge_node",
                name="alohamini_lerobot_bridge",
                output="screen",
                parameters=[
                    str(bridge_share / "config" / "bridge.yaml"),
                    {
                        "host": LaunchConfiguration("host"),
                    },
                ],
            ),
        ]
    )
