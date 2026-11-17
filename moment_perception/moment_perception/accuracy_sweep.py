"""accuracy_sweep: automatic perception accuracy test in MuJoCo.

For each distance d, it teleports the red/green/blue cubes to (d, +lat), (d, 0), (d, -lat)
in front of the robot (robot frame), waits for the scene to settle, calls
/perception/detect_all and logs the error to a CSV. Several repeats per distance.

Ground truth: the cubes are placed relative to the robot's TRUE pose (from the simulator's
floating-base topic), so the true position in base_footprint is known exactly.

By default the robot is first teleported to face -x (yaw = pi) from the origin: that view is
open floor, away from the green test patch, so the sweep measures distance only.

Usage (simulation + detector running, detector target_frame:=base_footprint):
  ros2 run moment_perception accuracy_sweep
  ros2 run moment_perception accuracy_sweep --distances 0.3 0.5 0.8 1.0 --repeats 5
  ros2 run moment_perception accuracy_sweep --lateral 0.0 0.15 0.3   # side-angle sweep
"""

import argparse
import csv
import math
import os
import statistics
import time
from typing import Dict, Optional, Tuple

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger

CUBES = {'red': 'red_cube', 'green': 'green_cube', 'blue': 'blue_cube'}


# ---------------- pure helpers (unit-tested, no ROS) ----------------
def parse_reply(message: str) -> Dict[str, Optional[Tuple[float, float, float, str]]]:
    """'red 0.49 0.10 0.02 base_footprint\\ngreen not_found' -> {'red': (..., frame), 'green': None}"""
    out = {}
    for line in message.strip().splitlines():
        parts = line.split()
        if len(parts) == 2 and parts[1] == 'not_found':
            out[parts[0]] = None
        elif len(parts) == 5:
            out[parts[0]] = (float(parts[1]), float(parts[2]), float(parts[3]), parts[4])
    return out


def robot_to_world(rx, ry, ryaw, px, py):
    """Point (px, py) in the robot frame -> world, robot at (rx, ry) with heading ryaw."""
    c, s = math.cos(ryaw), math.sin(ryaw)
    return rx + c * px - s * py, ry + s * px + c * py


def world_to_robot(rx, ry, ryaw, wx, wy):
    c, s = math.cos(ryaw), math.sin(ryaw)
    dx, dy = wx - rx, wy - ry
    return c * dx + s * dy, -s * dx + c * dy


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def extract_pose(msg):
    """Pose from Odometry, PoseStamped, Pose or TransformStamped (whatever the sim publishes)."""
    for path in (('pose', 'pose'), ('pose',), ()):
        obj = msg
        try:
            for a in path:
                obj = getattr(obj, a)
            if hasattr(obj, 'position') and hasattr(obj, 'orientation'):
                return obj.position.x, obj.position.y, obj.position.z, obj.orientation
        except AttributeError:
            pass
    if hasattr(msg, 'transform'):
        t = msg.transform
        return t.translation.x, t.translation.y, t.translation.z, t.rotation
    raise TypeError(f'cannot read a pose from {type(msg).__name__}')


# ---------------- ROS node ----------------
class Sweep(Node):

    def __init__(self, a):
        super().__init__('accuracy_sweep')
        self.a = a
        self.set_param_sim_time()
        self.detect = self.create_client(Trigger, a.detect_service)
        self.setter, self.SetReq, self.FJS = self.make_setter(a.set_service)
        self.base_pose = None
        self.make_base_sub(a.base_topic)

    def set_param_sim_time(self):
        from rclpy.parameter import Parameter
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

    def find_type(self, name, services=True):
        deadline = time.time() + 10.0
        while time.time() < deadline:
            items = (self.get_service_names_and_types() if services
                     else self.get_topic_names_and_types())
            for n, types in items:
                if n == name and types:
                    return types[0]
            rclpy.spin_once(self, timeout_sec=0.2)
        raise RuntimeError(f'{"service" if services else "topic"} {name} not found: is the simulation running?')

    def make_setter(self, name):
        from rosidl_runtime_py.utilities import get_message, get_service
        srv_type = self.find_type(name, services=True)
        srv = get_service(srv_type)
        pkg = srv_type.split('/')[0]
        fjs = get_message(f'{pkg}/msg/FreeJointState')
        self.get_logger().info(f'cube setter: {name} [{srv_type}]')
        return self.create_client(srv, name), srv.Request, fjs

    def make_base_sub(self, topic):
        from rosidl_runtime_py.utilities import get_message
        msg_type = get_message(self.find_type(topic, services=False))
        self.create_subscription(msg_type, topic, self.on_base, 10)
        self.get_logger().info(f'robot ground truth: {topic} [{msg_type.__name__}]')

    def on_base(self, msg):
        self.base_pose = extract_pose(msg)

    # -- helpers --
    def wait(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def call(self, client, req, timeout=5.0):
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f'service {client.srv_name} not available')
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            raise RuntimeError(f'service {client.srv_name} did not answer')
        return fut.result()

    def entry(self, body, x, y, z, yaw):
        e = self.FJS()
        e.name = body
        e.pose.header.frame_id = 'world'
        e.pose.pose.position.x, e.pose.pose.position.y, e.pose.pose.position.z = x, y, z
        e.pose.pose.orientation.z = math.sin(yaw / 2.0)
        e.pose.pose.orientation.w = math.cos(yaw / 2.0)
        return e

    def teleport(self, entries):
        req = self.SetReq()
        req.free_joints = entries
        res = self.call(self.setter, req)
        if not res.success:
            raise RuntimeError(f'set_free_joint_state failed: {res.message}')

    def robot_pose(self):
        self.base_pose = None
        end = time.time() + 5.0
        while self.base_pose is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.base_pose is None:
            raise RuntimeError(f'no message on {self.a.base_topic}')
        x, y, z, q = self.base_pose
        return x, y, z, yaw_from_quat(q)

    # -- the test --
    def run(self):
        a = self.a
        if a.robot_yaw is not None:
            x0, y0, z0, _ = self.robot_pose()
            self.teleport([self.entry(a.robot_body, a.robot_x, a.robot_y, z0, a.robot_yaw)])
            self.get_logger().info(f'robot teleported to ({a.robot_x}, {a.robot_y}), yaw {a.robot_yaw:.2f}')
            self.wait(a.settle)

        os.makedirs(os.path.dirname(os.path.expanduser(a.out)) or '.', exist_ok=True)
        path = os.path.expanduser(a.out)
        rows = []
        with open(path, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['distance_m', 'lateral_m', 'repeat', 'color', 'true_x', 'true_y',
                        'det_x', 'det_y', 'det_z', 'err_x_mm', 'err_y_mm', 'err_xy_mm', 'found'])
            for d in a.distances:
                for lat in a.lateral:
                    for rep in range(a.repeats):
                        rx, ry, rz, ryaw = self.robot_pose()
                        offsets = {'red': lat + a.spacing, 'green': lat, 'blue': lat - a.spacing}
                        placed = {}
                        entries = []
                        for color, body in CUBES.items():
                            wx, wy = robot_to_world(rx, ry, ryaw, d, offsets[color])
                            placed[color] = (wx, wy)
                            entries.append(self.entry(body, wx, wy, a.cube_size / 2.0, ryaw))
                        self.teleport(entries)
                        self.wait(a.settle)
                        rx, ry, rz, ryaw = self.robot_pose()          # true pose after settling
                        res = self.call(self.detect, Trigger.Request())
                        det = parse_reply(res.message)
                        for color in CUBES:
                            tx, ty = world_to_robot(rx, ry, ryaw, *placed[color])
                            r = det.get(color)
                            if r is None:
                                row = [d, lat, rep, color, round(tx, 4), round(ty, 4),
                                       '', '', '', '', '', '', 0]
                            else:
                                ex, ey = (r[0] - tx) * 1000, (r[1] - ty) * 1000
                                row = [d, lat, rep, color, round(tx, 4), round(ty, 4),
                                       r[0], r[1], r[2], round(ex, 1), round(ey, 1),
                                       round(math.hypot(ex, ey), 1), 1]
                            w.writerow(row)
                            rows.append(row)
                        f.flush()
                        self.get_logger().info(
                            f'd={d:.2f} lat={lat:+.2f} rep {rep + 1}/{a.repeats}: ' + ', '.join(
                                f'{c} {"MISS" if det.get(c) is None else "%.1f mm" % rows[-3 + i][11]}'
                                for i, c in enumerate(CUBES)))
        self.summary(rows, path)

    def summary(self, rows, path):
        print(f'\nCSV: {path}\n')
        print(f'{"dist":>5} {"lat":>6} {"color":>6} {"found":>6} {"mean mm":>8} {"max mm":>7}')
        keys = sorted({(r[0], r[1], r[3]) for r in rows}, key=lambda k: (k[0], k[1], list(CUBES).index(k[2])))
        for d, lat, c in keys:
            sel = [r for r in rows if (r[0], r[1], r[3]) == (d, lat, c)]
            errs = [r[11] for r in sel if r[12] == 1]
            found = f'{len(errs)}/{len(sel)}'
            mean = f'{statistics.mean(errs):.1f}' if errs else '-'
            mx = f'{max(errs):.1f}' if errs else '-'
            print(f'{d:5.2f} {lat:+6.2f} {c:>6} {found:>6} {mean:>8} {mx:>7}')


def main():
    p = argparse.ArgumentParser(description='Perception accuracy sweep in MuJoCo')
    p.add_argument('--distances', type=float, nargs='+', default=[0.3, 0.4, 0.5, 0.6, 0.8, 1.0, 1.2, 1.5])
    p.add_argument('--lateral', type=float, nargs='+', default=[0.0],
                   help='offset of the green cube sideways (m); red is +spacing, blue -spacing')
    p.add_argument('--spacing', type=float, default=0.10)
    p.add_argument('--repeats', type=int, default=3)
    p.add_argument('--cube-size', type=float, default=0.04)
    p.add_argument('--settle', type=float, default=1.5, help='seconds to wait after moving')
    p.add_argument('--robot-yaw', type=float, default=math.pi,
                   help='teleport robot to this heading first (default pi = facing -x, away from '
                        'the test patch); use --no-robot-teleport to keep it where it is')
    p.add_argument('--no-robot-teleport', dest='robot_yaw', action='store_const', const=None)
    p.add_argument('--robot-x', type=float, default=0.0)
    p.add_argument('--robot-y', type=float, default=0.0)
    p.add_argument('--robot-body', default='base_footprint')
    p.add_argument('--set-service', default='/controller_manager/set_free_joint_state')
    p.add_argument('--detect-service', default='/perception/detect_all')
    p.add_argument('--base-topic', default='/simulator/floating_base_state')
    p.add_argument('--out', default='~/moment_logs/distance_sweep.csv')
    a, ros_args = p.parse_known_args()

    rclpy.init(args=ros_args)
    node = Sweep(a)
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
