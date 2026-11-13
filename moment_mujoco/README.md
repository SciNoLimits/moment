# moment_mujoco

MuJoCo simulation of the **MOMENT** mobile manipulator (TurtleBot3 Waffle Pi base + OpenMANIPULATOR-X arm), driven through `ros2_control` with [`mujoco_ros2_control`](https://github.com/ros-controls/mujoco_ros2_control).

The package provides the MuJoCo models, the `ros2_control` hardware description, controller configuration and a single launch file. The robot description (URDF/meshes) is shared with the real robot via `moment_description`.

- **ROS 2:** Jazzy (Ubuntu 24.04)
- **Wheels:** `diff_drive_controller` (velocity commands on `/cmd_vel`)
- **Arm + gripper:** `forward_command_controller` (position commands)
- **Sensors:** IMU, RGB-D camera, 2D lidar, plus ground-truth base pose

---

## Package layout

```
moment_mujoco/
├── config/
│   ├── moment_controllers.yaml       # controller_manager, diff drive, arm, IMU broadcaster
│   └── moment_mujoco_plugins.yaml    # MuJoCo plugins: camera, lidar
├── launch/
│   └── moment_mujoco.launch.py       # robot_state_publisher + MuJoCo + controller spawners
├── models/
│   ├── moment_scene.xml              # world: floor, lights, attaches the robot
│   ├── moment.xml                    # robot MJCF: bodies, actuators, sensors, lidar
│   └── assets/                       # STL meshes referenced by moment.xml
└── urdf/
    ├── moment_mujoco.urdf.xacro      # includes moment_description + the control xacro
    └── moment_mujoco_control.xacro   # <ros2_control> block (joints, IMU, sim options)
```

## Dependencies

| Dependency | Notes |
| --- | --- |
| `moment_description` | Shared robot URDF/xacro and meshes |
| `mujoco_ros2_control`, `mujoco_ros2_control_plugins`, `mujoco_3d_lidar` | **Built from source** (see below), not the apt release |
| `controller_manager`, `diff_drive_controller`, `forward_command_controller`, `joint_state_broadcaster`, `imu_sensor_broadcaster` | `ros-jazzy-ros2-controllers` |
| `robot_state_publisher`, `xacro` | Standard ROS 2 packages |

### Why a source build?

The apt release of `mujoco_ros2_control` for Jazzy does not include the `mujoco.plugin.lidar` MuJoCo extension, so the model fails to load with `plugin mujoco.plugin.lidar not found`. The camera and IMU work with the apt packages; only the lidar needs the newer code.

```bash
cd ~/cs_moment_ws/src
git clone https://github.com/ros-controls/mujoco_ros2_control -b main

source /opt/ros/jazzy/setup.bash
cd ~/cs_moment_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

The build cannot write into the apt-managed MuJoCo folder, so copy the lidar library there once:

```bash
sudo mkdir -p /opt/ros/jazzy/opt/mujoco_vendor/bin/mujoco_plugin
sudo cp "$(find ~/cs_moment_ws/build ~/cs_moment_ws/install -name libmujoco_3d_lidar.so | head -n1)" \
        /opt/ros/jazzy/opt/mujoco_vendor/bin/mujoco_plugin/
```

An apt upgrade of `mujoco_vendor` may remove that file; copy it again if the lidar stops loading. For reproducibility, pin the clone to the commit you built (`git -C src/mujoco_ros2_control rev-parse HEAD`).

If you do not need `/scan`, remove the lidar `<extension>` block and the lidar `<plugin>` sensor line from `models/moment.xml` and the lidar block from `config/moment_mujoco_plugins.yaml`; the apt packages are then enough.

## Build and run

```bash
cd ~/cs_moment_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select moment_description moment_mujoco
source install/setup.bash        # your workspace must be sourced after /opt/ros/jazzy

ros2 launch moment_mujoco moment_mujoco.launch.py
```

A MuJoCo viewer window opens (`headless` and `sim_speed_factor` are set in `urdf/moment_mujoco_control.xacro`). All nodes run with `use_sim_time:=true`; MuJoCo publishes `/clock`.

### Drive the base

`/cmd_vel` is a **`TwistStamped`** on Jazzy:

```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.1}}}"
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/TwistStamped "{twist: {angular: {z: 0.5}}}"
```

Leaving the header stamp at zero is fine for CLI tests. A real publisher must stamp with sim time (`use_sim_time:=true`), otherwise commands can look older than `cmd_vel_timeout` (0.5 s) and are dropped. For keyboard teleop, use a `teleop_twist_keyboard` version that supports `-p stamped:=true`.

Limits (from `moment_controllers.yaml`): 0.26 m/s linear, 1.82 rad/s angular.

### Move the arm

`arm_position_controller` takes five positions in this order: `joint1, joint2, joint3, joint4, gripper_left_joint`.

```bash
ros2 topic pub --once /arm_position_controller/commands std_msgs/msg/Float64MultiArray \
  "{data: [0.0, 0.0, 0.0, 0.0, 0.019]}"
```

| Joint | Type | Range |
| --- | --- | --- |
| `joint1` | revolute (z) | -2.827 … 2.827 rad |
| `joint2` | revolute (y) | -1.791 … 1.571 rad |
| `joint3` | revolute (y) | -0.942 … 1.382 rad |
| `joint4` | revolute (y) | -1.791 … 2.042 rad |
| `gripper_left_joint` | slide | -0.010 … 0.019 m |

The right gripper finger is coupled to the left one by an equality constraint in the MJCF, so only `gripper_left_joint` is commanded.

## Topics

| Topic | Type | Source |
| --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/TwistStamped` | input to `diff_drive_controller` |
| `/odom` | `nav_msgs/Odometry` | `diff_drive_controller`; TF `odom` → `base_footprint` |
| `/simulator/floating_base_state` | `nav_msgs/Odometry` | **ground-truth** base pose from the MuJoCo free joint |
| `/joint_states` | `sensor_msgs/JointState` | `joint_state_broadcaster` |
| `/arm_position_controller/commands` | `std_msgs/Float64MultiArray` | input to the arm controller |
| `/imu` | `sensor_msgs/Imu` | `imu_sensor_broadcaster`, frame `imu_link` |
| `/scan` | `sensor_msgs/LaserScan` | `Mujoco3dLidarPlugin`, frame `base_scan` (360 rays, 0.12–3.5 m, 5 Hz) |
| `/camera/image_raw`, `/camera/depth/image_raw`, `/camera/camera_info` | `sensor_msgs/Image`, `CameraInfo` | `CameraPlugin`, frame `camera_rgb_optical_frame` |
| `/clock` | `rosgraph_msgs/Clock` | MuJoCo simulation time |

Simulation control services are exposed under `/controller_manager/`: `reset_world`, `set_pause`, `step_simulation`, `set_free_joint_state`.

Quick health check:

```bash
ros2 control list_controllers              # all controllers: active
ros2 topic hz /scan                        # ~5 Hz
ros2 topic echo /imu --once                # linear_acceleration.z ≈ +9.8 at rest
ros2 topic info /cmd_vel -v                # subscriber: diff_drive_controller
```

In RViz, set the fixed frame to `base_footprint` or `odom`, and set the LaserScan and Image displays to **Best Effort** reliability if they stay empty.

## How the pieces fit together

- `moment_scene.xml` loads `moment.xml` and attaches it at `base_footprint` with no name prefix, so MJCF and URDF names match.
- The `ros2_control` joints in `moment_mujoco_control.xacro` are matched to MJCF actuators by the joint they drive. The wheels use `<velocity>` actuators and the arm/gripper use `<position>` actuators, matching the command interfaces.
- Both wheel joint axes point along world +Y, so equal positive wheel velocities drive the robot forward. No sign flip is needed in the diff drive config.
- The camera and lidar sit at the URDF's `camera_rgb_frame` / `base_scan` positions, so TF from `robot_state_publisher` lines up with the simulated sensors.
- The IMU sensors (`imu_sensor_quat`, `_gyro`, `_accel`) are attached to the `imu_link` site and exposed through a `<sensor>` entry in the control xacro.

## Model tuning notes

These came out of getting the diff drive to behave and are worth keeping in mind when editing `moment.xml`:

- **Base inertia:** `base_footprint` has an explicit `<inertial>` taken from the URDF. Without it, MuJoCo derives the mass from the collision meshes at a default density and the base ends up far heavier than the real robot.
- **Wheel actuators:** the wheels are light (≈ 28 g each), so a stiff velocity actuator is numerically unstable at the default timestep and the wheels chatter instead of rolling. Keep the actuator gain (`kv`) and force limit low, and keep the joint `armature` (motor inertia) and `damping` small.
- **Casters:** the rear casters are small frictionless sphere geoms (`condim="1"`, `priority="1"`). The priority matters: MuJoCo uses the larger friction of two touching geoms unless one has higher priority.
- **Stability:** the robot has two wheels and rear casters only, so a large forward arm reach can move the centre of mass past the axle and tip it forward. Add front bumper spheres just above the floor if that becomes a problem.
- **Lidar housing:** the lidar mesh is visual-only so the lidar rays do not hit their own housing.

## Troubleshooting

| Symptom | Likely cause and fix |
| --- | --- |
| `plugin mujoco.plugin.lidar not found` on startup | Lidar library not in the MuJoCo plugin folder; repeat the `sudo cp` step, or remove the lidar blocks |
| `/cmd_vel` published but the robot does not move | Message type is not `TwistStamped`, or the stamp is stale versus sim time; check `ros2 topic info /cmd_vel -v` |
| `Velocity command timed out. Braking.` every second | Normal when nothing is publishing `/cmd_vel` |
| Wheel velocities flip sign every step, robot jitters | Actuator gain too high for the wheel inertia; lower `kv` / force limit, raise `armature` |
| Could not enable FIFO RT scheduling | Harmless in simulation |
| `/imu`, `/scan` or `/camera/*` missing | Controller not spawned or plugin YAML not passed to the control node; check the launch file and the startup log |
| New terminal behaves like the old apt version | Source `/opt/ros/jazzy/setup.bash` first, then `~/cs_moment_ws/install/setup.bash` |

## Known limitations

- The lidar needs the source-built `mujoco_ros2_control` plus a manual library copy (see above).
- Arm control is position-only; there is no trajectory or planning layer in this package.
- `/odom` accuracy depends on the diff drive odometry settings in `moment_controllers.yaml`; compare against `/simulator/floating_base_state` when tuning.