"""color_detector node: finds red, green and blue objects in an RGB-D camera and
returns their 3-D positions in the robot frame.

Inputs  (all remappable via parameters)
  rgb_topic    sensor_msgs/Image        colour image (bgr8 / rgb8)
  depth_topic  sensor_msgs/Image        depth registered to the colour image (32FC1 m or 16UC1 mm)
  info_topic   sensor_msgs/CameraInfo   intrinsics of the colour image

Outputs
  services  /perception/detect_red, /detect_green, /detect_blue, /detect_all   (std_srvs/Trigger)
            response.message, one line per colour:  "<color> <x> <y> <z> <frame>"  or  "<color> not_found"
  topics    /perception/<color>/pose   geometry_msgs/PoseStamped (target_frame)
            /perception/markers        visualization_msgs/MarkerArray   (RViz)
            /perception/debug_image    sensor_msgs/Image (boxes + labels)

Typical use by the mission: robot stops at the pickup location, calls detect_red,
picks it, then detect_green, then detect_blue (re-detecting each time because the
scene changes after every pick).
"""

import math

import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped, PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from std_srvs.srv import Trigger
from tf2_geometry_msgs import do_transform_point
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray

from moment_perception.color_detector import (colors_from_params, cube_center_from_surface,
                                              depth_to_meters, detect)

import cv2

MARKER_RGB = {'red': (1.0, 0.1, 0.1), 'green': (0.1, 0.9, 0.1), 'blue': (0.1, 0.3, 1.0)}
DRAW_BGR = {'red': (0, 0, 255), 'green': (0, 200, 0), 'blue': (255, 80, 0)}

DEFAULT_HSV = {
    'red': [0, 120, 70, 10, 255, 255, 170, 120, 70, 180, 255, 255],
    'green': [40, 80, 60, 85, 255, 255],
    'blue': [100, 120, 60, 130, 255, 255],
}


class ColorDetectorNode(Node):

    def __init__(self):
        super().__init__('color_detector')
        p = self.declare_parameter
        p('rgb_topic', '/camera/image')
        p('depth_topic', '/camera/depth_image')
        p('info_topic', '/camera/camera_info')
        p('target_frame', 'base_footprint')
        p('depth_scale', 1.0)              # 1.0 for 32FC1 metres, 0.001 for 16UC1 millimetres
        p('colors', ['red', 'green', 'blue'])
        for c, v in DEFAULT_HSV.items():
            p(f'hsv.{c}', v)
        p('min_area_px', 150)
        p('min_depth', 0.1)
        p('max_depth', 4.0)
        p('erode_px', 3)
        p('min_object_size', 0.015)        # metres; smaller blobs are specks / far noise
        p('max_object_size', 0.08)         # metres; larger blobs are floor zones, walls, bays
        p('output_point', 'center')        # 'center' = cube centre (for grasping), 'surface' = visible surface
        p('cube_size', 0.04)               # metres, edge length
        p('cube_on_ground', True)          # True: centre z = ground_z + cube_size/2 (exact)
        p('ground_z', 0.0)                 # height of the surface the cubes rest on, in target_frame
        p('max_frame_age', 1.0)            # seconds; older frames are rejected by the services
        p('tf_latest_fallback', False)     # True only if the robot is guaranteed to stand still
        p('continuous', False)             # True: detect + publish on every frame
        p('publish_debug_image', True)

        g = lambda n: self.get_parameter(n).value
        self.target_frame = g('target_frame')
        self.depth_scale = float(g('depth_scale'))
        self.color_names = list(g('colors'))
        self.colors = colors_from_params({c: list(g(f'hsv.{c}')) for c in self.color_names
                                          if self.has_parameter(f'hsv.{c}')})
        self.min_area = int(g('min_area_px'))
        self.min_depth, self.max_depth = float(g('min_depth')), float(g('max_depth'))
        self.erode_px = int(g('erode_px'))
        self.min_size = float(g('min_object_size'))
        self.max_size = float(g('max_object_size'))
        self.output_center = str(g('output_point')).lower() == 'center'
        self.cube_size = float(g('cube_size'))
        self.ground_z = float(g('ground_z')) if bool(g('cube_on_ground')) else None
        self.max_age = float(g('max_frame_age'))
        self.tf_fallback = bool(g('tf_latest_fallback'))
        self.continuous = bool(g('continuous'))
        self.debug = bool(g('publish_debug_image'))

        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.K = None
        self.latest = None   # (rgb_msg, depth_msg)

        self.create_subscription(CameraInfo, g('info_topic'), self.on_info, qos_profile_sensor_data)
        rgb_sub = Subscriber(self, Image, g('rgb_topic'), qos_profile=qos_profile_sensor_data)
        depth_sub = Subscriber(self, Image, g('depth_topic'), qos_profile=qos_profile_sensor_data)
        self.sync = ApproximateTimeSynchronizer([rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.sync.registerCallback(self.on_frames)

        self.pose_pubs = {c: self.create_publisher(PoseStamped, f'/perception/{c}/pose', 10)
                          for c in self.color_names}
        self.marker_pub = self.create_publisher(MarkerArray, '/perception/markers', 10)
        self.debug_pub = self.create_publisher(Image, '/perception/debug_image', 10)

        for c in self.color_names:
            self.create_service(Trigger, f'/perception/detect_{c}',
                                lambda req, res, c=c: self.handle_detect(res, [c]))
        self.create_service(Trigger, '/perception/detect_all',
                            lambda req, res: self.handle_detect(res, self.color_names))

        self.get_logger().info(
            f"color_detector ready: colors={self.color_names}, target_frame={self.target_frame}, "
            f"rgb={g('rgb_topic')}, depth={g('depth_topic')}")

    # ---------- callbacks ----------
    def on_info(self, msg: CameraInfo):
        if self.K is None:
            self.get_logger().info(f'Camera intrinsics received: fx={msg.k[0]:.1f} fy={msg.k[4]:.1f}')
        self.K = list(msg.k)

    def on_frames(self, rgb: Image, depth: Image):
        self.latest = (rgb, depth)
        if self.continuous and self.K is not None:
            self.run(self.color_names)

    def handle_detect(self, res, names):
        if self.K is None or self.latest is None:
            res.success, res.message = False, 'no camera data yet (check topics / bridge)'
            return res
        age = (self.get_clock().now() - Time.from_msg(self.latest[0].header.stamp)).nanoseconds * 1e-9
        if age > self.max_age:
            res.success, res.message = False, f'camera frame is {age:.2f} s old (max {self.max_age})'
            return res
        results = self.run(names)
        lines, found_all = [], True
        for c in names:
            r = results.get(c)
            if r is None:
                lines.append(f'{c} not_found'); found_all = False
            else:
                lines.append(f'{c} {r[0]:.4f} {r[1]:.4f} {r[2]:.4f} {self.target_frame}')
        res.success, res.message = found_all, '\n'.join(lines)
        return res

    # ---------- core ----------
    def run(self, names):
        rgb_msg, depth_msg = self.latest
        bgr = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')
        depth = depth_to_meters(self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='passthrough'),
                                self.depth_scale)
        if depth.shape[:2] != bgr.shape[:2]:
            self.get_logger().error('depth and colour sizes differ: depth must be registered to colour')
            return {}
        dets = detect(bgr, depth, self.K, {c: self.colors[c] for c in names},
                      self.min_area, self.min_depth, self.max_depth, self.erode_px,
                      self.min_size, self.max_size)

        out, markers = {}, MarkerArray()
        for i, c in enumerate(names):
            d = dets.get(c)
            if d is None or d.point_cam is None:
                continue
            pt = d.point_cam
            ps = PointStamped()
            ps.header = rgb_msg.header
            ps.point.x, ps.point.y, ps.point.z = map(float, pt)
            try:
                tf = self.tf_buffer.lookup_transform(self.target_frame, rgb_msg.header.frame_id,
                                                     Time.from_msg(rgb_msg.header.stamp),
                                                     timeout=Duration(seconds=0.2))
            except TransformException as e:
                # In 'map' the robot may be moving: a transform from a different time would put the
                # cube in the wrong place, so by default we refuse instead of guessing.
                if not self.tf_fallback:
                    self.get_logger().warn(f'TF {rgb_msg.header.frame_id} -> {self.target_frame} '
                                           f'at image time failed: {e}')
                    continue
                try:
                    tf = self.tf_buffer.lookup_transform(self.target_frame, rgb_msg.header.frame_id, Time())
                except TransformException as e2:
                    self.get_logger().warn(f'TF {rgb_msg.header.frame_id} -> {self.target_frame}: {e2}')
                    continue
            p = do_transform_point(ps, tf).point
            if self.output_center:
                t = tf.transform.translation      # camera origin in target_frame
                cx_, cy_, cz_ = cube_center_from_surface(np.array([p.x, p.y, p.z]),
                                                         np.array([t.x, t.y, t.z]),
                                                         self.cube_size, self.ground_z)
                p.x, p.y, p.z = float(cx_), float(cy_), float(cz_)
            out[c] = (p.x, p.y, p.z)

            pose = PoseStamped()
            pose.header.stamp = rgb_msg.header.stamp
            pose.header.frame_id = self.target_frame
            pose.pose.position = p
            pose.pose.orientation.w = 1.0
            self.pose_pubs[c].publish(pose)
            markers.markers.append(self.marker(i, c, pose))

        if markers.markers:
            self.marker_pub.publish(markers)
        if self.debug:
            self.publish_debug(bgr, dets, out, rgb_msg.header)
        return out

    def marker(self, idx, color, pose):
        m = Marker()
        m.header = pose.header
        m.ns, m.id, m.type, m.action = 'perception', idx, Marker.SPHERE, Marker.ADD
        m.pose = pose.pose
        m.scale.x = m.scale.y = m.scale.z = 0.04
        m.color.r, m.color.g, m.color.b = MARKER_RGB.get(color, (1.0, 1.0, 1.0))
        m.color.a = 0.9
        m.lifetime.sec = 2
        return m

    def publish_debug(self, bgr, dets, out, header):
        img = bgr.copy()
        for c, d in dets.items():
            if d is None:
                continue
            x, y, w, h = d.bbox
            col = DRAW_BGR.get(c, (255, 255, 255))
            cv2.rectangle(img, (x, y), (x + w, y + h), col, 2)
            cv2.circle(img, (int(d.pixel[0]), int(d.pixel[1])), 4, col, -1)
            label = c if c not in out else f'{c} ({out[c][0]:.2f}, {out[c][1]:.2f}, {out[c][2]:.2f})'
            if d.size_m is not None:
                label += f' {d.size_m * 100:.0f}cm'
            if d.point_cam is None:
                label += ' no depth'
            cv2.putText(img, label, (x, max(15, y - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, col, 2)
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header = header
        self.debug_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ColorDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
