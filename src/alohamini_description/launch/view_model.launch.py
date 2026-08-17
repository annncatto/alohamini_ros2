from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    share = Path(get_package_share_directory("alohamini_description"))
    robot_description = (share / "urdf" / "alohamini2pro_moveit.urdf").read_text(encoding="utf-8")
    rviz_config = share / "rviz" / "description.rviz"
    use_gui = LaunchConfiguration("use_gui")
    use_rviz = LaunchConfiguration("use_rviz")

    home = {
        "zeros.left_shoulder_pan": 0.0,
        "zeros.left_shoulder_lift": 0.0,
        "zeros.left_elbow_flex": 0.0,
        "zeros.left_wrist_flex": 1.435806017460960,
        "zeros.left_wrist_yaw_joint": 0.0,
        "zeros.left_wrist_roll": 0.0,
        "zeros.left_gripper": 0.32,
        "zeros.right_shoulder_pan": 0.0,
        "zeros.right_shoulder_lift": 0.0,
        "zeros.right_elbow_flex": 0.0,
        "zeros.right_wrist_flex": 1.5,
        "zeros.right_wrist_yaw_joint": 0.0,
        "zeros.right_wrist_roll": 0.0,
        "zeros.right_gripper": 0.32,
    }
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_gui", default_value="true"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="alohamini_view_robot_state_publisher",
                parameters=[{"robot_description": robot_description}],
                remappings=[
                    ("robot_description", "/alohamini/robot_description"),
                    ("joint_states", "/alohamini_view/joint_states"),
                ],
                output="screen",
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="alohamini_view_joint_state_publisher_gui",
                parameters=[{"robot_description": robot_description}, home],
                remappings=[("joint_states", "/alohamini_view/joint_states")],
                condition=IfCondition(use_gui),
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="alohamini_view_joint_state_publisher",
                parameters=[{"robot_description": robot_description}, home],
                remappings=[("joint_states", "/alohamini_view/joint_states")],
                condition=UnlessCondition(use_gui),
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", str(rviz_config)],
                condition=IfCondition(use_rviz),
                output="log",
            ),
        ]
    )
