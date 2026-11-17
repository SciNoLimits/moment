# Moment manipulation

One ROS 2 Jazzy node exposes the relative actions `pick` and `place`. With the
default namespace these resolve to `/pick` and `/place`. It sends arm and hand
motions to an existing MoveIt `move_group` through two C++ `MoveGroupInterface`
instances.

## Install

Extract `moment_manipulation` into `/workspace/src` next to `moment_interfaces`.
The interface package must already define these actions:

| Type | Goal | Result | Feedback |
| --- | --- | --- | --- |
| `moment_interfaces/action/Pick` | `geometry_msgs/PoseStamped object_pose` | `bool success`, `string message` | `string stage` |
| `moment_interfaces/action/Place` | Empty | `bool success`, `string message` | `string stage` |

The default configuration uses `moment_arm` from `base_link` to
`end_effector_link`, and `moment_gripper` with `gripper_left_joint`.
The SRDF targets are `place` and `home` for the arm and `open` and
`close` for the gripper. Change `home_target` if you add a different
return pose.

For a top-down pick, configure the TCP quaternion in the planning frame and
set the grasp Z offset from the reported object origin. Without a fixed
quaternion, the transformed object orientation is used as the TCP orientation.
Approach and lift are straight segments along the planning frame's vertical
axis. Confirm that the planning frame's +Z points up.

```bash
cd /workspace
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to moment_manipulation
source install/setup.bash
```

Start MuJoCo and the existing MoveIt `move_group` first. The manipulation
launch loads the URDF and SRDF as node parameters and starts only the action
server. The MoveIt demo uses mock hardware and can introduce a second
controller manager if run alongside MuJoCo.

The attached MoveIt config expects these trajectory action servers:

- `/moment_arm_controller/follow_joint_trajectory`
- `/moment_gripper_controller/follow_joint_trajectory`

Check that MuJoCo exposes these names with `ros2 action list -t`. If the
names differ, align the MoveIt controller configuration with the controllers
actually running in MuJoCo before sending a goal. Simulation time is
enabled for the manipulation server, and `move_group` also needs sim time.

```bash
ros2 launch moment_manipulation manipulation.launch.py
```

## Send a goal

Start with `ros2 action list -t` to confirm both servers appear. Substitute actual
cube coordinates and a valid frame and orientation in the pick example. The
numbers below illustrate the message format. The four-joint arm can reach
only certain TCP orientations at a given position. Use a grasp orientation
you have verified at the cube location in RViz.

```bash
ros2 action send_goal /pick moment_interfaces/action/Pick \
  '{object_pose: {header: {frame_id: base_link}, pose: {position: {x: 0.4, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}}' \
  --feedback

ros2 action send_goal /place moment_interfaces/action/Place '{}' --feedback
```

Feedback reports the stage that is starting. The result identifies the stage
that failed. If opening succeeds and returning home fails, the failure message
also says the gripper already opened. Only one goal can run at a time.
This version rejects cancellation requests during execution.

Pick success means that the configured gripper close and lift completed. It
uses the simulator's gripper contact physics and does not measure retention or
attach an object in MoveIt's planning scene. For object-aware collision checking
while carrying a cube, the planning scene needs an attached collision object
and the appropriate allowed gripper contacts. This package expects the MoveIt
hand group to be directly executable. A gripper exposed only by a separate
command interface needs a different gripper implementation.
