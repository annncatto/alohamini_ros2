from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("alohamini_description"))
    model = share / "urdf" / "lidar_three_wheel_preview.urdf.xacro"
    robot_description = Command([FindExecutable(name="xacro"), " ", str(model)])
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                parameters=[{"robot_description": robot_description}],
                condition=UnlessCondition(use_gui),
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                parameters=[{"robot_description": robot_description}],
                condition=IfCondition(use_gui),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(share / "rviz" / "description.rviz")],
                condition=IfCondition(use_rviz),
                output="log",
            ),
        ]
    )
