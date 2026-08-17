from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("alohamini_description"))
    model = package_share / "urdf" / "alohamini2pro_moveit.urdf"
    robot_description = model.read_text(encoding="utf-8")

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="alohamini_robot_state_publisher",
                parameters=[
                    {
                        "robot_description": robot_description,
                        "use_sim_time": False,
                    }
                ],
                remappings=[
                    ("robot_description", "/alohamini/robot_description"),
                ],
                output="screen",
            ),
        ]
    )
