import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Package share directories
    mujoco_dir = get_package_share_directory('moment_mujoco')
    nav_dir = get_package_share_directory('moment_navigation')
    manip_dir = get_package_share_directory('moment_manipulation')
    perc_dir = get_package_share_directory('moment_perception')

    # Launch arguments
    scene_arg = DeclareLaunchArgument(
        'scene',
        default_value='moment_combined_warehouse1_scene.xml',
        description='MuJoCo scene XML file to load',
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock if true',
    )

    scene = LaunchConfiguration('scene')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # MuJoCo Simulation
    mujoco_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(mujoco_dir, 'launch', 'moment_mujoco.launch.py')
        ),
        launch_arguments={
            'scene': scene,
            'use_sim_time': use_sim_time,
        }.items(),
    )

    # Nav2 Navigation
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'moment_nav2.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Manipulation
    manipulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(manip_dir, 'launch', 'manipulation.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # Perception
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perc_dir, 'launch', 'perception.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    return LaunchDescription([
        scene_arg,
        use_sim_time_arg,
        mujoco_launch,
        TimerAction(period=4.0, actions=[navigation_launch]),
        TimerAction(period=8.0, actions=[manipulation_launch]),
        TimerAction(period=12.0, actions=[perception_launch]),
        
    ])
