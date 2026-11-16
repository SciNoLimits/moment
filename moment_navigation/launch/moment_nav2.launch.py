import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    moment_navigation_dir = get_package_share_directory('moment_navigation')
    
    nav2_rviz_config = os.path.join(
        moment_navigation_dir,
        "rviz",
        "nav2_config.rviz"
    )

    map_file = os.path.join(
        moment_navigation_dir,
        'maps',
        'warehouse_map.yaml'
    )

    params_file = os.path.join(
        moment_navigation_dir,
        'config',
        'nav2_params.yaml'
    )
    

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_bringup_dir,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': 'True',
            'autostart': 'True',
        }.items()
    )
    
    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output='screen',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': True}]
        
    )

    return LaunchDescription([
        nav2_launch,
        TimerAction(period=3.0, actions=[rviz2])
    ])