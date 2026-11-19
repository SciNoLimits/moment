"""Launch the MOMENT warehouse simulation and manipulation stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetParameter
from launch_ros.substitutions import FindPackageShare


def package_launch(package, filename):
    return PythonLaunchDescriptionSource(
        PathJoinSubstitution([FindPackageShare(package), "launch", filename])
    )


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value="moment_combined_warehouse1_scene.xml",
            description="Scene filename in moment_mujoco/models.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true", description="Start MoveIt RViz.",
        ),
        GroupAction(actions=[
            IncludeLaunchDescription(
                package_launch("moment_mujoco", "moment_mujoco.launch.py"),
                launch_arguments={"scene": LaunchConfiguration("scene")}.items(),
            ),
        ]),
        GroupAction(actions=[
            SetParameter(name="use_sim_time", value=True),
            IncludeLaunchDescription(
                package_launch("moment_moveit_config", "move_group.launch.py"),
            ),
            IncludeLaunchDescription(
                package_launch("moment_moveit_config", "moveit_rviz.launch.py"),
                condition=IfCondition(LaunchConfiguration("rviz")),
            ),
            IncludeLaunchDescription(
                package_launch("moment_manipulation", "manipulation.launch.py"),
            ),
        ]),
    ])
