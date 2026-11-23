# MOMENT Gazebo simulation (ROS 2 Jazzy)

This workspace contains the supplied `moment_description` package plus `moment_gazebo`.
The Gazebo launch uses the same TurtleBot3 Waffle Pi + OpenMANIPULATOR-X component Xacros,
but a Gazebo-specific top-level description so the MuJoCo hardware plugin is not loaded.

## Requirements

Ubuntu 24.04 / ROS 2 Jazzy with Gazebo Harmonic. Install the simulation/control packages
available for your ROS installation, including `ros_gz_sim`, `ros_gz_bridge`,
`gz_ros2_control`, `ros2_control`, `ros2_controllers`, `xacro`, and `robot_state_publisher`.

## Build and launch

Copy both packages into a ROS 2 workspace `src/` directory (this zip already has `src/`):

```bash
cd ~/moment_gazebo_workspace
source /opt/ros/jazzy/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
ros2 launch moment_gazebo sim.launch.py
```

## Interfaces

- Base velocity command: `/diff_drive_controller/cmd_vel` (`geometry_msgs/msg/Twist`)
- Odometry: `/diff_drive_controller/odom`
- Joint states: `/joint_states`
- Arm + gripper position command: `/arm_position_controller/commands`
  (`std_msgs/msg/Float64MultiArray`, order `joint1`, `joint2`, `joint3`, `joint4`, `gripper_left_joint`)
- Sensors intended: `/scan` (`sensor_msgs/msg/LaserScan`), `/imu` (`sensor_msgs/msg/Imu`),
  `/camera/image` (`sensor_msgs/msg/Image`)

Examples:

```bash
ros2 topic pub --once /diff_drive_controller/cmd_vel geometry_msgs/msg/Twist   "{linear: {x: 0.1}, angular: {z: 0.0}}"

ros2 topic pub --once /arm_position_controller/commands std_msgs/msg/Float64MultiArray   "{data: [0.0, -0.5, 0.8, 0.0, 0.01]}"

ros2 topic echo /scan
ros2 topic echo /imu
```

## Notes

- Gazebo Harmonic's sensor and ros_gz_bridge topic behavior can vary with installed Gazebo/ROS package versions.
  Confirm actual Gazebo Transport topics with `gz topic -l`; adjust `config/bridge.yaml` if the sensor
  topic names differ.
- The included source description was originally configured for MuJoCo. This package deliberately uses
  `urdf/moment_gazebo.urdf.xacro` instead of `moment_description/urdf/moment.urdf.xacro`.
- First run should be treated as an integration baseline: validate collision geometry, wheel axis/sign,
  arm limits, sensor frames, and controller startup in your installed Jazzy/Harmonic environment.
