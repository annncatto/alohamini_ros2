"""MoveIt execution against the existing LeRobot Host through the ROS bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    description = Path(get_package_share_directory("alohamini_description"))
    moveit_share = Path(get_package_share_directory("alohamini_moveit_config"))
    bridge_share = Path(get_package_share_directory("alohamini_lerobot_bridge"))
    moveit_config = (
        MoveItConfigsBuilder(
            "alohamini2pro", package_name="alohamini_moveit_config"
        )
        .robot_description(
            file_path=str(description / "urdf" / "alohamini2pro_moveit.urdf")
        )
        .robot_description_semantic(
            file_path=str(description / "srdf" / "alohamini2pro.srdf")
        )
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(
            file_path="config/moveit_controllers.yaml",
            moveit_manage_controllers=False,
        )
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .to_moveit_configs()
    )
    move_group_parameters = [
        moveit_config.to_dict(),
        {
            "allow_trajectory_execution": True,
            "publish_robot_description_semantic": True,
            "monitor_dynamics": False,
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("host", default_value="127.0.0.1"),
            DeclareLaunchArgument("arm_mapping_dir", default_value=""),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    str(bridge_share / "launch" / "bridge.launch.py")
                ),
                launch_arguments={
                    "host": LaunchConfiguration("host"),
                    "arm_mapping_dir": LaunchConfiguration("arm_mapping_dir"),
                }.items(),
            ),
            # /joint_states comes exclusively from the bridge's measured Host state.
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="alohamini_robot_state_publisher",
                parameters=[moveit_config.robot_description],
                remappings=[
                    ("robot_description", "/alohamini/robot_description"),
                ],
                output="screen",
            ),
            Node(
                package="moveit_ros_move_group",
                executable="move_group",
                output="screen",
                parameters=move_group_parameters,
            ),
            GroupAction(
                condition=IfCondition(LaunchConfiguration("use_rviz")),
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="moveit_rviz",
                        output="log",
                        arguments=["-d", str(moveit_share / "config" / "moveit.rviz")],
                        parameters=[
                            moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits,
                        ],
                    )
                ],
            ),
        ]
    )
