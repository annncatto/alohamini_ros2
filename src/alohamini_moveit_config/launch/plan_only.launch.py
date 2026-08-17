"""AlohaMini2Pro MoveIt launch that cannot execute hardware trajectories."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    description = Path(get_package_share_directory("alohamini_description"))
    package = Path(get_package_share_directory("alohamini_moveit_config"))
    moveit_config = (
        MoveItConfigsBuilder("alohamini2pro", package_name="alohamini_moveit_config")
        .robot_description(file_path=str(description / "urdf" / "alohamini2pro_moveit.urdf"))
        .robot_description_semantic(file_path=str(description / "srdf" / "alohamini2pro.srdf"))
        .robot_description_kinematics()
        .joint_limits()
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"])
        .trajectory_execution(file_path="config/moveit_controllers.yaml", moveit_manage_controllers=False)
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

    # Installed folded Home from alohamini_calibration/config/hardware/arm_home.yaml.
    # Keep every arm joint explicit so collision checks cover the real common pose.
    home = {
        "zeros.left_shoulder_pan": 0.0,
        "zeros.left_shoulder_lift": 0.0,
        "zeros.left_elbow_flex": 0.0,
        "zeros.left_wrist_flex": 1.435806017460960,
        "zeros.left_wrist_yaw_joint": 0.0,
        "zeros.left_wrist_roll": 0.0,
        "zeros.right_shoulder_pan": 0.0,
        "zeros.right_shoulder_lift": 0.0,
        "zeros.right_elbow_flex": 0.0,
        "zeros.right_wrist_flex": 1.5,
        "zeros.right_wrist_yaw_joint": 0.0,
        "zeros.right_wrist_roll": 0.0,
        "zeros.left_gripper": 0.32,
        "zeros.right_gripper": 0.32,
    }
    move_group_parameters = [
        moveit_config.to_dict(),
        {
            "allow_trajectory_execution": False,
            "publish_robot_description_semantic": True,
            "monitor_dynamics": False,
            # Never let an offline fake state race the real bridge on
            # /joint_states when both graphs are visible on the same domain.
            "planning_scene_monitor_options.joint_state_topic": "/alohamini_plan_only/joint_states",
        },
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument(
                "joycon_preview",
                default_value="false",
                description="Disable the fixed Home joint publisher so the Joy-Con IK worker can preview /joint_states.",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                name="alohamini_fake_joint_state_publisher",
                parameters=[moveit_config.robot_description, home],
                remappings=[
                    ("joint_states", "/alohamini_plan_only/joint_states"),
                    ("robot_description", "/alohamini/robot_description"),
                ],
                output="screen",
                condition=UnlessCondition(LaunchConfiguration("joycon_preview")),
            ),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="alohamini_plan_only_robot_state_publisher",
                parameters=[moveit_config.robot_description],
                remappings=[
                    ("joint_states", "/alohamini_plan_only/joint_states"),
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
                        arguments=["-d", str(package / "config/moveit.rviz")],
                        parameters=[
                            moveit_config.robot_description,
                            moveit_config.robot_description_semantic,
                            moveit_config.robot_description_kinematics,
                            moveit_config.planning_pipelines,
                            moveit_config.joint_limits,
                        ],
                        condition=UnlessCondition(LaunchConfiguration("joycon_preview")),
                    ),
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="joycon_preview_rviz",
                        output="log",
                        arguments=["-d", str(package / "config/joycon_preview.rviz")],
                        parameters=[moveit_config.robot_description],
                        condition=IfCondition(LaunchConfiguration("joycon_preview")),
                    ),
                ],
            ),
        ]
    )
