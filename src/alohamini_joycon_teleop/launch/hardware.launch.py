from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    package = Path(get_package_share_directory("alohamini_joycon_teleop"))
    moveit = Path(get_package_share_directory("alohamini_moveit_config"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("host"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_native_reader", default_value="true"),
            DeclareLaunchArgument(
                "native_python",
                default_value="/home/anncatto/miniconda3/envs/lerobot_alohamini/bin/python",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(moveit / "launch" / "hardware_execution.launch.py")
                ),
                launch_arguments={
                    "host": LaunchConfiguration("host"),
                    "use_rviz": LaunchConfiguration("use_rviz"),
                }.items(),
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
