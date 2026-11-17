"""Start the colour detector.

The camera topics come from the simulation (mujoco_ros2_control CameraPlugin):
/camera/image_raw, /camera/depth/image_raw, /camera/camera_info.

  ros2 launch moment_perception perception.launch.py
  ros2 launch moment_perception perception.launch.py continuous:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('moment_perception')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg, 'config', 'perception.yaml')),
        DeclareLaunchArgument('continuous', default_value='false',
                              description='Detect on every frame (debug image, pose topics)'),

        Node(
            package='moment_perception', executable='color_detector', name='color_detector',
            parameters=[LaunchConfiguration('params_file'),
                        {'use_sim_time': LaunchConfiguration('use_sim_time'),
                         'continuous': LaunchConfiguration('continuous')}],
            output='screen'),
    ])
