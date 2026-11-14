"""Offline tests: synthetic RGB-D frames with known cube positions."""

import numpy as np
import cv2
import pytest

from moment_perception.color_detector import (DEFAULT_COLORS, colors_from_params, depth_to_meters,
                                              detect)

W, H = 640, 480
FX = FY = 554.25          # ~60 deg horizontal FOV at 640 px, like the Gazebo camera
CX, CY = 320.0, 240.0
K = [FX, 0, CX, 0, FY, CY, 0, 0, 1]

BGR = {'red': (0, 0, 220), 'green': (0, 200, 0), 'blue': (220, 0, 0)}


def make_scene(objects, background_depth=1.5, background=(140, 140, 140)):
    """objects: list of (color, x, y, z, size_m) in camera optical frame. Draws square faces."""
    img = np.full((H, W, 3), background, np.uint8)
    depth = np.full((H, W), background_depth, np.float32)
    for color, x, y, z, s in objects:
        u0 = int(round(FX * (x - s / 2) / z + CX)); u1 = int(round(FX * (x + s / 2) / z + CX))
        v0 = int(round(FY * (y - s / 2) / z + CY)); v1 = int(round(FY * (y + s / 2) / z + CY))
        img[v0:v1, u0:u1] = BGR[color]
        depth[v0:v1, u0:u1] = z
    return img, depth


def test_three_cubes_positions():
    objs = [('red', -0.15, 0.05, 0.60, 0.05),
            ('green', 0.00, 0.05, 0.70, 0.05),
            ('blue', 0.18, 0.02, 0.55, 0.05)]
    img, depth = make_scene(objs)
    res = detect(img, depth, K, DEFAULT_COLORS)
    for color, x, y, z, _ in objs:
        d = res[color]
        assert d is not None, color
        assert np.allclose(d.point_cam, [x, y, z], atol=0.01), (color, d.point_cam)


def test_missing_color_returns_none():
    img, depth = make_scene([('red', 0.0, 0.0, 0.6, 0.05)])
    res = detect(img, depth, K, DEFAULT_COLORS)
    assert res['red'] is not None
    assert res['green'] is None and res['blue'] is None


def test_red_hue_wraparound():
    img, depth = make_scene([])
    hsv = np.zeros((40, 40, 3), np.uint8); hsv[:] = (175, 220, 200)   # red on the high side
    img[200:240, 300:340] = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)
    depth[200:240, 300:340] = 0.5
    res = detect(img, depth, K, DEFAULT_COLORS)
    assert res['red'] is not None


def test_largest_blob_wins_and_noise_ignored():
    img, depth = make_scene([('blue', 0.1, 0.0, 0.6, 0.06)])
    img[10:14, 10:14] = BGR['blue']        # 16 px speck: below min_area
    res = detect(img, depth, K, DEFAULT_COLORS)
    assert abs(res['blue'].point_cam[0] - 0.1) < 0.01


def test_invalid_depth_gives_none_point():
    img, depth = make_scene([('green', 0.0, 0.0, 0.6, 0.05)])
    depth[:] = np.nan
    res = detect(img, depth_to_meters(depth, 1.0), K, DEFAULT_COLORS)
    assert res['green'] is not None and res['green'].point_cam is None


def test_millimetre_depth_scaling():
    img, depth = make_scene([('red', 0.0, 0.0, 0.8, 0.05)])
    depth_mm = (depth * 1000).astype(np.uint16)
    res = detect(img, depth_to_meters(depth_mm, 0.001), K, DEFAULT_COLORS)
    assert abs(res['red'].point_cam[2] - 0.8) < 0.01


def test_cube_center_from_surface():
    from moment_perception.color_detector import cube_center_from_surface
    cam = np.array([0.07, 0.0, 0.20])                 # camera in base frame, above and behind
    # median visible point: on the near (camera-facing) side of a 3 cm cube centred at (0.40, 0.10, 0.015)
    centre = np.array([0.40, 0.10, 0.015])
    d = centre[:2] - cam[:2]; d /= np.linalg.norm(d)
    surface = np.array([*(centre[:2] - 0.015 * d), 0.022])
    est = cube_center_from_surface(surface, cam, 0.03, ground_z=0.0)
    assert np.allclose(est, centre, atol=1e-6)
    # ground unknown: z kept as measured
    est2 = cube_center_from_surface(surface, cam, 0.03, ground_z=None)
    assert abs(est2[2] - 0.022) < 1e-9


def test_colors_from_params():
    c = colors_from_params({'red': [0, 120, 70, 10, 255, 255, 170, 120, 70, 180, 255, 255]})
    assert len(c['red'].ranges) == 2
    with pytest.raises(ValueError):
        colors_from_params({'bad': [1, 2, 3]})
