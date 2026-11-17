from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    package_share = Path(get_package_share_directory("moment_manipulation"))
    moveit_config = MoveItConfigsBuilder(
        "moment", package_name="moment_moveit_config"
    ).to_moveit_configs()

    return LaunchDescription([
        Node(
            package="moment_manipulation",
            executable="manipulation_server",
            output="screen",
            parameters=[
                str(package_share / "config" / "manipulation.yaml"),
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.joint_limits,
            ],
        )
    ])
