from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    description_share = FindPackageShare("moment_mujoco")

    xacro_file = PathJoinSubstitution([
        description_share,
        "urdf",
        "moment_mujoco.urdf.xacro",
    ])

    controllers_file = PathJoinSubstitution([
        description_share,
        "config",
        "moment_controllers.yaml",
    ])

    plugins_file = PathJoinSubstitution([
        description_share, 
        "config", 
        "moment_mujoco_plugins.yaml"
    ])

    robot_description = {
        "robot_description": ParameterValue(
            Command(["xacro ", xacro_file]),
            value_type=str,
        )
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            robot_description,
            {"use_sim_time": True},
        ],
    )

    control_node = Node(
        package="mujoco_ros2_control",
        executable="ros2_control_node",
        name="controller_manager",
        output="both",
        emulate_tty=True,
        parameters=[
            {"use_sim_time": True},
            controllers_file,
            plugins_file,
        ],
    )

    joint_state_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_position_controller",
            "--param-file",
            controllers_file,
        ],
        output="screen",
    )

    diff_drive_spawner = Node(
      package="controller_manager", executable="spawner",
      arguments=["diff_drive_controller", "-c", "/controller_manager",
                 "--controller-ros-args",
                 "-r /diff_drive_controller/cmd_vel:=/cmd_vel -r /diff_drive_controller/odom:=/odom"],
      output="screen",
    )

    imu_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["imu_sensor_broadcaster", "-c", "/controller_manager",
                "--controller-ros-args", "-r /imu_sensor_broadcaster/imu:=/imu"],
        output="screen",
    )


    return LaunchDescription([
        robot_state_publisher,
        control_node,
        joint_state_spawner,
        arm_controller_spawner,
        diff_drive_spawner,
        imu_spawner,
    ])