"""Joy-Con hardware input component using externally owned ROS interfaces."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package = Path(get_package_share_directory("alohamini_joycon_teleop"))
    description = Path(get_package_share_directory("alohamini_description"))
    urdf = (
        description / "urdf" / "alohamini2pro_moveit.urdf"
    ).read_text(encoding="utf-8")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_native_reader", default_value="true"),
            DeclareLaunchArgument(
                "native_python",
                default_value=(
                    "/home/anncatto/miniconda3/envs/lerobot_alohamini/bin/python"
                ),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="joycon_hardware_rviz",
                output="log",
                arguments=["-d", str(package / "config" / "joycon_hardware.rviz")],
                parameters=[{"robot_description": urdf}],
                condition=IfCondition(LaunchConfiguration("use_rviz")),
            ),
            ExecuteProcess(
                cmd=[
                    LaunchConfiguration("native_python"),
                    str(package / "scripts" / "joycon_native_reader.py"),
                    "--endpoint",
                    "tcp://127.0.0.1:5567",
                ],
                output="screen",
                condition=IfCondition(LaunchConfiguration("start_native_reader")),
            ),
            Node(
                package="alohamini_joycon_teleop",
                executable="teleop_node",
                name="alohamini_joycon_teleop",
                output="screen",
                parameters=[
                    str(package / "config" / "joycon.yaml"),
                    {"hardware_mode": True},
                ],
            ),
        ]
    )
