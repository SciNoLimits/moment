"""Start the colour detector (and optionally the Gazebo camera bridge).

  ros2 launch moment_perception perception.launch.py
  ros2 launch moment_perception perception.launch.py bridge:=true continuous:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('moment_perception')
    use_sim_time = LaunchConfiguration('use_sim_time')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(pkg, 'config', 'perception.yaml')),
        DeclareLaunchArgument('bridge', default_value='false',
                              description='Start ros_gz_bridge for the camera (if the sim launch does not)'),
        DeclareLaunchArgument('continuous', default_value='false'),

        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='camera_bridge',
            parameters=[{'config_file': os.path.join(pkg, 'sim', 'camera_bridge.yaml'),
                         'use_sim_time': use_sim_time}],
            condition=IfCondition(LaunchConfiguration('bridge')), output='screen'),

        Node(
            package='moment_perception', executable='color_detector', name='color_detector',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': use_sim_time,
                         'continuous': LaunchConfiguration('continuous')}],
            output='screen'),
    ])
