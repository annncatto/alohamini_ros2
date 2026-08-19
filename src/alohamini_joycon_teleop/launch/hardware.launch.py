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
    description = Path(get_package_share_directory("alohamini_description"))
    urdf = (
        description / "urdf" / "alohamini2pro_moveit.urdf"
    ).read_text(encoding="utf-8")
    return LaunchDescription(
        [
            DeclareLaunchArgument("host"),
            # Machine-specific arm mapping produced by
            # `ros2 run alohamini_calibration sync_arm_mapping` (e.g.
            # ~/.config/alohamini/192.168.3.73). Empty = the installed
            # default mapping.
            DeclareLaunchArgument("arm_mapping_dir", default_value=""),
            # Named joycon_rviz so the included hardware_execution launch's
            # use_rviz:=false (a global launch configuration) cannot shadow it.
            DeclareLaunchArgument("joycon_rviz", default_value="true"),
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
                    "arm_mapping_dir": LaunchConfiguration("arm_mapping_dir"),
                    # The MoveIt RViz panel has an interactive TCP marker that
                    # fights the Joy-Con controller; the Joy-Con launch uses
                    # its own RViz config without it.
                    "use_rviz": "false",
                }.items(),
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="joycon_hardware_rviz",
                output="log",
                arguments=["-d", str(package / "config" / "joycon_hardware.rviz")],
                parameters=[{"robot_description": urdf}],
                condition=IfCondition(LaunchConfiguration("joycon_rviz")),
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
