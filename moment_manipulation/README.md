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

Update `config/manipulation.yaml` to match the current MoveIt SRDF. The arm needs
named joint states for the place and home poses. The hand needs named states
for open and closed, and a MoveIt controller that can execute hand trajectories.
The shipped arm, hand, and TCP names come from the earlier Panda setup.
Use the actual group names, link names, and named states from the regenerated
config. Add any missing states in MoveIt Setup Assistant under Robot Poses.

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

Start MuJoCo and the existing MoveIt launch first. Confirm that MoveIt itself can
execute arm and hand moves. The manipulation node needs the robot description
and semantic description. If they are missing from the description topics,
pass the same `robot_description` and `robot_description_semantic` dictionaries
as parameters to the manipulation node from your MoveIt launch, or configure
your existing bringup to publish those descriptions. Keep simulation time
enabled while MuJoCo publishes `/clock`.

```bash
ros2 run moment_manipulation manipulation_server --ros-args \
  --params-file /workspace/src/moment_manipulation/config/manipulation.yaml
```

## Send a goal

Start with `ros2 action list -t` to confirm both servers appear. Substitute actual
cube coordinates and a valid frame and orientation in the pick example. The
numbers below illustrate the message format.

```bash
ros2 action send_goal /pick moment_interfaces/action/Pick \
  '{object_pose: {header: {frame_id: fer_link0}, pose: {position: {x: 0.4, y: 0.0, z: 0.2}, orientation: {w: 1.0}}}}' \
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
