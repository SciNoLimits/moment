import os

from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    moment_navigation_dir = get_package_share_directory('moment_navigation')
    
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='False',
        description='Launch RViz if True'
    )
    
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
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', nav2_rviz_config],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )
    
    go_to_goal_node = Node(
        package='moment_navigation',
        executable='go_to_goal',
        name='go_to_goal_server',
        output='screen'
    )

    return LaunchDescription([
        rviz_arg,
        nav2_launch,
        TimerAction(
            period=3.0,
            actions=[rviz2, go_to_goal_node]
        )
    ])