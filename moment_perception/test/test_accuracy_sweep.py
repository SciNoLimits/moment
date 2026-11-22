"""Offline tests for the pure helpers of accuracy_sweep (no ROS needed)."""

import math
import sys
import types

import pytest

# accuracy_sweep imports rclpy / std_srvs at module level; stub them so the helpers can be tested offline
for mod in ('rclpy', 'rclpy.node', 'std_srvs', 'std_srvs.srv'):
    sys.modules.setdefault(mod, types.ModuleType(mod))
sys.modules['rclpy.node'].Node = object
sys.modules['std_srvs.srv'].Trigger = object

from moment_perception.accuracy_sweep import (extract_pose, parse_reply, robot_to_world,  # noqa: E402
                                              world_to_robot, yaw_from_quat)


def test_parse_reply():
    r = parse_reply('red 0.4989 0.0975 0.0200 base_footprint\ngreen not_found\nblue 0.5 -0.1 0.02 map')
    assert r['red'] == (0.4989, 0.0975, 0.02, 'base_footprint')
    assert r['green'] is None
    assert r['blue'][3] == 'map'


@pytest.mark.parametrize('yaw', [0.0, math.pi / 2, math.pi, -2.0])
def test_robot_world_roundtrip(yaw):
    wx, wy = robot_to_world(1.0, -2.0, yaw, 0.5, 0.1)
    px, py = world_to_robot(1.0, -2.0, yaw, wx, wy)
    assert px == pytest.approx(0.5) and py == pytest.approx(0.1)


def test_facing_minus_x():
    # robot at origin facing -x: a point 0.5 m ahead is at world x = -0.5
    wx, wy = robot_to_world(0.0, 0.0, math.pi, 0.5, 0.1)
    assert wx == pytest.approx(-0.5) and wy == pytest.approx(-0.1)


class Q:
    def __init__(self, x, y, z, w):
        self.x, self.y, self.z, self.w = x, y, z, w


def test_yaw_from_quat():
    assert yaw_from_quat(Q(0, 0, math.sin(0.6), math.cos(0.6))) == pytest.approx(1.2)


def test_extract_pose_odometry_like():
    pos = types.SimpleNamespace(x=1.0, y=2.0, z=0.0)
    inner = types.SimpleNamespace(position=pos, orientation=Q(0, 0, 0, 1))
    odom = types.SimpleNamespace(pose=types.SimpleNamespace(pose=inner))
    x, y, z, q = extract_pose(odom)
    assert (x, y, z) == (1.0, 2.0, 0.0)
