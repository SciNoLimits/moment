# moment_perception

Detects the **red, green and blue cubes** with the robot's RGB-D camera and returns each
cube's **centre position** in `base_footprint` (robot centre on the floor; x forward, y left, z up).
Accuracy: ≤ 4 mm up to 1.5 m when the cube is fully in the camera image
(see [TEST_RESULTS.md](TEST_RESULTS.md)). Owner: Charan.

## Quick start

```bash
# 1. get the code
cd ~/cs_ws/src/a-g2-autonomous_mobile_manipulator
git fetch && git checkout feature/object_detection && git pull

# 2. install + build (once)
sudo apt install ros-jazzy-cv-bridge ros-jazzy-tf2-geometry-msgs ros-jazzy-message-filters python3-opencv
cd ~/cs_ws && colcon build --symlink-install && source install/setup.bash

# 3. simulation with test cubes (terminal 1)
ros2 launch moment_mujoco moment_mujoco.launch.py scene:=moment_perception_test_scene.xml

# 4. detector (terminal 2)
ros2 launch moment_perception perception.launch.py

# 5. ask for a cube (terminal 3)
ros2 service call /perception/detect_red std_srvs/srv/Trigger
#  -> message: 'red 0.4989 0.0975 0.0200 base_footprint'
```

MuJoCo must be the source-built `mujoco_ros2_control` (see `moment_mujoco/README.md`).

## Interface

| What | Name | Type |
|---|---|---|
| Find one cube (use before grasping) | `/perception/detect_red`, `/detect_green`, `/detect_blue` | `std_srvs/srv/Trigger` |
| Find all cubes | `/perception/detect_all` | `std_srvs/srv/Trigger` |
| Pose stream | `/perception/<colour>/pose` | `geometry_msgs/msg/PoseStamped` |
| RViz markers | `/perception/markers` | `visualization_msgs/msg/MarkerArray` |
| Camera image with detections | `/perception/debug_image` | `sensor_msgs/msg/Image` |

**Service reply** (`message`), one line per colour: `<colour> <x> <y> <z> <frame>` or `<colour> not_found`.
`success` is false if a requested colour is missing or the camera image is older than 1 s.

**What the position means**
- Cube **centre**, metres, in `base_footprint`. z = 0.02 for a 4 cm cube on the floor (not measured).
- **No orientation** (always w = 1): grasp from above.
- Face the cube before asking: a cube cut off at the image edge can be off by ~1 cm.
- Pose topics only publish after a service call, or always with `continuous:=true`.

## Use it in your node

```python
import rclpy
from std_srvs.srv import Trigger

def get_cube(node, color):
    """(x, y, z, frame) of the cube centre, or None if not found."""
    cli = node.create_client(Trigger, f'/perception/detect_{color}')
    if not cli.wait_for_service(timeout_sec=5.0):
        node.get_logger().error('perception not running')
        return None
    fut = cli.call_async(Trigger.Request())
    rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    res = fut.result()
    if res is None or not res.success:
        return None
    _, x, y, z, frame = res.message.split()[:5]
    return float(x), float(y), float(z), frame
```

**Need it relative to the arm base?** Convert with TF:

```python
import tf2_geometry_msgs  # noqa: F401 (registers PoseStamped with tf2)
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

# in __init__:
self.tf_buffer = Buffer()
self.tf_listener = TransformListener(self.tf_buffer, self)

# when a PoseStamped `msg` arrives on /perception/red/pose:
p = self.tf_buffer.transform(msg, 'link1', timeout=Duration(seconds=0.2))
```

## Move the test cubes

```bash
ros2 service call /controller_manager/set_free_joint_state mujoco_ros2_control_msgs/srv/SetFreeJointState \
"{free_joints: [{name: red_cube, pose: {pose: {position: {x: 0.6, y: 0.2, z: 0.02}, orientation: {w: 1.0}}}}]}"
```
World coordinates. Bodies: `red_cube`, `green_cube`, `blue_cube`.

## Settings (`config/perception.yaml`)

| Setting | Default | Meaning |
|---|---|---|
| `target_frame` | `base_footprint` | frame of the returned positions |
| `cube_size` | `0.04` | cube edge length (m) |
| `min_object_size` / `max_object_size` | `0.015` / `0.08` | blobs outside this size are ignored (rejects the coloured bays) |
| `hsv.red/green/blue` | see file | colour ranges (OpenCV HSV) |

Override at start, e.g. `ros2 launch moment_perception perception.launch.py continuous:=true`.

## Known limits

- One cube per colour (the largest cube-sized blob wins).
- Position only, no rotation.
- Cubes cut off at the image border give ~1 cm error (fix pending).

Finds the **red, green and blue** cubes with an RGB-D camera and returns each one's 3-D position in the robot frame.
ROS 2 Jazzy · Gazebo Harmonic · OpenCV. Owner: Charan Kalyana Kumar

```
RGB image ──► HSV threshold ──► largest blob ──► centroid pixels ┐
                                                                  ├─► median 3-D point ──► TF ──► base_footprint
Depth image ─────────────────────────────────────► depth per pixel┘   (camera optical frame)
```

## Interface (for manipulation / mission)

| What | Name | Type |
|---|---|---|
| Detect one colour | `/perception/detect_red`, `/perception/detect_green`, `/perception/detect_blue` | `std_srvs/Trigger` |
| Detect all | `/perception/detect_all` | `std_srvs/Trigger` |
| Pose per colour | `/perception/<color>/pose` | `geometry_msgs/PoseStamped` (in `target_frame`) |
| RViz markers | `/perception/markers` | `visualization_msgs/MarkerArray` |
| Debug image | `/perception/debug_image` | `sensor_msgs/Image` |

Service reply, one line per colour: `red 0.4120 0.0790 0.0300 base_footprint` or `red not_found`.
`success` is false if any requested colour is missing, the frame is stale, or there's no camera data.

Intended sequence: robot parks at the pickup location → `detect_red` → pick → `detect_green` → pick → `detect_blue`.
Re-detect before each pick: the scene changes after every grasp.

## 1. Add the camera to the robot (moment_description)

```xml
<xacro:include filename="$(find moment_perception)/sim/rgbd_camera.urdf.xacro"/>
<xacro:rgbd_camera parent="base_link" xyz="0.07 0 0.20" rpy="0 0.5 0"/>
```
Adjust `xyz`/`rpy` so the pickup area is in view. The macro adds `camera_link` and `camera_optical_frame`
(z forward, x right, y down). Images are stamped with the optical frame. Without it, every 3-D point is rotated wrong.

Bridge the camera (if the sim launch doesn't already): `sim/camera_bridge.yaml`, or launch with `bridge:=true`.

## 2. Build

```bash
sudo apt install ros-jazzy-cv-bridge ros-jazzy-tf2-geometry-msgs ros-jazzy-message-filters ros-jazzy-ros-gz-bridge python3-opencv
cd ~/cs_ws/src && cp -r <path>/moment_perception .
cd ~/cs_ws && colcon build --symlink-install --packages-select moment_perception && source install/setup.bash
```

## 3. Run and test

```bash
# T1: simulation with the camera-equipped robot
# T2: test cubes in front of the robot (world coordinates are printed)
bash ~/cs_ws/src/moment_perception/sim/spawn_cubes.sh 0.5 0.0
# T3: detector (continuous:=true while tuning; false for normal service use)
ros2 launch moment_perception perception.launch.py bridge:=true continuous:=true
# T4: ask for positions
ros2 service call /perception/detect_red std_srvs/srv/Trigger
ros2 service call /perception/detect_all std_srvs/srv/Trigger
```

RViz: Image display on `/perception/debug_image`, MarkerArray on `/perception/markers`.

### Checks
| Check | Command | Expect |
|---|---|---|
| Camera publishing | `ros2 topic hz /camera/image` and `/camera/depth_image` | ~15 Hz each |
| Intrinsics | detector log | `Camera intrinsics received: fx=…` |
| TF | `ros2 run tf2_ros tf2_echo base_footprint camera_optical_frame` | prints a transform |
| Accuracy | compare service output with spawn positions (robot at origin → base ≈ world) | within ~1–2 cm |

## 4. Tuning

- **Colour missed / wrong blobs:** set `continuous: true`, watch `/perception/debug_image`, widen or narrow the HSV range in `config/perception.yaml`.
  Red needs two hue ranges (it wraps at H=0/180).
- **Centre vs surface:** default `output_point: center` returns the cube centre (what the gripper needs). `ground_z` is the floor height *in `target_frame`*: 0.0 for `base_footprint` or `map`, but **negative** for an arm-base frame like `link1` (it sits above the floor). Set `cube_on_ground: false` if cubes sit on uneven surfaces; then z is measured.
- **Real RealSense:** use the aligned-depth topics in the yaml comments and `depth_scale: 0.001`.

## Files

| File | Purpose |
|---|---|
| `moment_perception/color_detector.py` | detection maths (no ROS): HSV mask, blob, deprojection |
| `moment_perception/detector_node.py` | ROS node: sync, services, TF, publishing |
| `config/perception.yaml` | topics, frame, HSV ranges, thresholds |
| `sim/rgbd_camera.urdf.xacro` | Gazebo Harmonic RGB-D camera + optical frame |
| `sim/camera_bridge.yaml` | ros_gz_bridge for image, depth, info, points |
| `sim/spawn_cubes.sh` | spawns red/green/blue 3 cm cubes |
| `test/test_color_detector.py` | 8 offline tests on synthetic RGB-D frames + centre estimation |

## Limits

- One object per colour (largest blob wins). Two red cubes → only the bigger one is returned.
- Position only, no orientation. Fine for a top-down grasp of a cube; not for rotated objects.
- Centre x, y assume the camera sees the cube's near side; error is a few mm if the cube is rotated 45°.
- Tested offline on synthetic images; first real run is in your Gazebo setup.
