"""Spawn AlohaMini in Gazebo Fortress without any real-hardware process."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable,
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from alohamini_gazebo.sim_description import make_sim_description


def generate_launch_description() -> LaunchDescription:
    gazebo_share = Path(get_package_share_directory("alohamini_gazebo"))
    description_share = Path(get_package_share_directory("alohamini_description"))
    ros_gz_share = Path(get_package_share_directory("ros_gz_sim"))
    world = gazebo_share / "worlds" / "lift_pick_place.sdf"
    controllers = gazebo_share / "config" / "ros2_controllers.yaml"
    robot_description = make_sim_description(
        description_share / "urdf" / "alohamini2pro_moveit.urdf",
        controllers,
    )

    headless = LaunchConfiguration("headless")
    common_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r {world}"}.items(),
        condition=UnlessCondition(headless),
    )
    headless_gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(ros_gz_share / "launch" / "gz_sim.launch.py")),
        launch_arguments={"gz_args": f"-r -s {world}"}.items(),
        condition=IfCondition(headless),
    )

    controller_names = [
        "joint_state_broadcaster",
        "base_planar_controller",
        "wheel_velocity_controller",
        "lift_controller",
        "left_arm_controller",
        "right_arm_controller",
        "left_gripper_controller",
        "right_gripper_controller",
    ]

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "headless",
                default_value="false",
                description="Run only the Gazebo server, without the GUI client.",
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", str(gazebo_share / "models")
            ),
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", str(gazebo_share / "models")
            ),
            AppendEnvironmentVariable(
                "GZ_SIM_RESOURCE_PATH", str(description_share.parent)
            ),
            AppendEnvironmentVariable(
                "IGN_GAZEBO_RESOURCE_PATH", str(description_share.parent)
            ),
            common_gz,
            headless_gz,
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="alohamini_gazebo_robot_state_publisher",
                parameters=[{"robot_description": robot_description, "use_sim_time": True}],
                output="screen",
            ),
            Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                name="alohamini_gazebo_bridge",
                parameters=[{"config_file": str(gazebo_share / "config" / "bridge.yaml")}],
                output="screen",
            ),
            TimerAction(
                period=2.0,
                actions=[
                    Node(
                        package="ros_gz_sim",
                        executable="create",
                        arguments=[
                            "-name", "alohamini2pro",
                            "-topic", "robot_description",
                            "-z", "0.01",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="controller_manager",
                        executable="spawner",
                        arguments=[
                            *controller_names,
                            "--controller-manager", "/controller_manager",
                            "--controller-manager-timeout", "60",
                        ],
                        output="screen",
                    )
                ],
            ),
            TimerAction(
                period=10.0,
                actions=[
                    Node(
                        package="alohamini_gazebo",
                        executable="omni_base_adapter",
                        parameters=[
                            str(gazebo_share / "config" / "demo.yaml"),
                            {"use_sim_time": True},
                        ],
                        output="screen",
                    )
                ],
            ),
        ]
    )
