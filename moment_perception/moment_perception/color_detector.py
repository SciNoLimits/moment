"""Pure detection maths, no ROS. Unit-tested in test/test_color_detector.py.

Pipeline per colour:
  1. BGR -> HSV, threshold with the colour's hue/sat/value ranges (red uses two
     hue ranges because it wraps around 0/180 in OpenCV).
  2. Clean the mask (open + close) and keep the largest blob above min_area.
  3. Erode that blob so edge pixels (which mix object and background depth) are dropped.
  4. Deproject every remaining pixel with its depth through the pinhole model:
        X = (u - cx) * Z / fx,   Y = (v - cy) * Z / fy,   Z = depth
     and take the per-axis median -> one robust 3-D point in the camera OPTICAL frame
     (x right, y down, z forward).
"""

from dataclasses import dataclass
from typing import Dict, List, Optional, Sequence, Tuple

import cv2
import numpy as np


@dataclass
class ColorRange:
    name: str
    ranges: List[Tuple[Sequence[int], Sequence[int]]]  # list of (hsv_low, hsv_high)


@dataclass
class Detection:
    color: str
    pixel: Tuple[float, float]          # (u, v) centroid of the blob
    area_px: int                        # blob area in pixels
    point_cam: Optional[np.ndarray]     # (x, y, z) in camera optical frame, metres; None if no valid depth
    valid_depth_px: int                 # pixels that contributed to the 3-D estimate
    bbox: Tuple[int, int, int, int]     # x, y, w, h
    size_m: Optional[float] = None      # estimated physical size of the blob (metres)


DEFAULT_COLORS: Dict[str, ColorRange] = {
    'red': ColorRange('red', [((0, 120, 70), (10, 255, 255)),
                              ((170, 120, 70), (180, 255, 255))]),
    'green': ColorRange('green', [((40, 80, 60), (85, 255, 255))]),
    'blue': ColorRange('blue', [((100, 120, 60), (130, 255, 255))]),
}


def color_mask(hsv: np.ndarray, color: ColorRange) -> np.ndarray:
    mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
    for low, high in color.ranges:
        mask |= cv2.inRange(hsv, np.array(low, np.uint8), np.array(high, np.uint8))
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def largest_blob(mask: np.ndarray, min_area: int):
    """Return (blob_mask, contour) of the largest connected blob, or (None, None)."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= min_area]
    if not contours:
        return None, None
    best = max(contours, key=cv2.contourArea)
    blob = np.zeros_like(mask)
    cv2.drawContours(blob, [best], -1, 255, thickness=cv2.FILLED)
    return blob, best


def candidate_blobs(mask: np.ndarray, min_area: int):
    """All blobs above min_area as (blob_mask, contour), largest first."""
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted((c for c in contours if cv2.contourArea(c) >= min_area),
                      key=cv2.contourArea, reverse=True)
    out = []
    for c in contours:
        blob = np.zeros_like(mask)
        cv2.drawContours(blob, [c], -1, 255, thickness=cv2.FILLED)
        out.append((blob, c))
    return out


def depth_to_meters(depth: np.ndarray, depth_scale: float) -> np.ndarray:
    """32FC1 is already metres (Gazebo). 16UC1 is millimetres (RealSense): pass depth_scale=0.001."""
    d = depth.astype(np.float32) * depth_scale
    d[~np.isfinite(d)] = 0.0
    return d


def deproject_blob(blob: np.ndarray, depth_m: np.ndarray, fx: float, fy: float,
                   cx: float, cy: float, min_depth: float, max_depth: float,
                   erode_px: int = 3):
    """Median 3-D point of the blob in the camera optical frame. Returns (point|None, n_pixels)."""
    if erode_px > 0:
        inner = cv2.erode(blob, np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8))
        if cv2.countNonZero(inner) > 0:
            blob = inner
    vs, us = np.nonzero(blob)
    z = depth_m[vs, us]
    ok = (z > min_depth) & (z < max_depth)
    if not np.any(ok):
        return None, 0
    us, vs, z = us[ok].astype(np.float64), vs[ok].astype(np.float64), z[ok].astype(np.float64)
    x = (us - cx) * z / fx
    y = (vs - cy) * z / fy
    pt = np.array([np.median(x), np.median(y), np.median(z)])
    return pt, int(ok.sum())


def blob_size_m(blob: np.ndarray, depth_m: np.ndarray, fx: float, fy: float,
                cx: float, cy: float, min_depth: float, max_depth: float,
                erode_px: int = 3) -> Optional[float]:
    """Physical size of a blob in metres (largest 5-95 percentile extent along x, y or z).

    Measured on the ERODED blob, like the position: edge pixels can carry the depth of
    whatever is behind the object, which inflated the z extent of a 4 cm cube at 1.5 m to
    5-8+ cm (Run 4). The trimmed edge is added back to x and y (erode_px pixels per side,
    converted to metres at the blob's depth), so a 4 cm cube still measures ~4 cm.
    """
    used = blob
    trim = 0
    if erode_px > 0:
        inner = cv2.erode(blob, np.ones((2 * erode_px + 1, 2 * erode_px + 1), np.uint8))
        if cv2.countNonZero(inner) >= 5:
            used, trim = inner, erode_px
    vs, us = np.nonzero(used)
    z = depth_m[vs, us]
    ok = (z > min_depth) & (z < max_depth)
    if ok.sum() < 5:
        return None
    us, vs, z = us[ok].astype(np.float64), vs[ok].astype(np.float64), z[ok].astype(np.float64)
    pts = np.stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])
    ext = np.percentile(pts, 95, axis=1) - np.percentile(pts, 5, axis=1)
    zm = float(np.median(z))
    ext[0] += 2 * trim * zm / fx
    ext[1] += 2 * trim * zm / fy
    return float(np.max(ext))


def detect(bgr: np.ndarray, depth_m: np.ndarray, K: Sequence[float],
           colors: Dict[str, ColorRange], min_area: int = 150,
           min_depth: float = 0.1, max_depth: float = 4.0,
           erode_px: int = 3, min_size_m: float = 0.0,
           max_size_m: float = float('inf')) -> Dict[str, Optional[Detection]]:
    """Detect each colour's object and its 3-D position. K = row-major 3x3 intrinsics.

    Blobs are tried largest first; the first whose physical size lies in
    [min_size_m, max_size_m] wins. This rejects same-coloured floor zones (too big)
    and specks (too small). With the default limits every blob passes (old behaviour).
    """
    fx, fy, cx, cy = K[0], K[4], K[2], K[5]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    out: Dict[str, Optional[Detection]] = {}
    for name, rng in colors.items():
        out[name] = None
        for blob, contour in candidate_blobs(color_mask(hsv, rng), min_area):
            size = blob_size_m(blob, depth_m, fx, fy, cx, cy, min_depth, max_depth, erode_px)
            if size is not None and not (min_size_m <= size <= max_size_m):
                continue                                   # wrong size: zone or speck
            m = cv2.moments(contour)
            u, v = m['m10'] / m['m00'], m['m01'] / m['m00']
            pt, n = deproject_blob(blob, depth_m, fx, fy, cx, cy, min_depth, max_depth, erode_px)
            out[name] = Detection(name, (u, v), int(cv2.contourArea(contour)), pt, n,
                                  cv2.boundingRect(contour), size)
            break
    return out


def cube_center_from_surface(surface_pt: np.ndarray, camera_pos: np.ndarray,
                             cube_size: float, ground_z: Optional[float]) -> np.ndarray:
    """Estimate a cube's CENTRE from the median visible-surface point.

    Both points are in the same frame with z pointing up (e.g. base_footprint).
      x, y: move half a cube horizontally, away from the camera (the camera sees the near side).
      z:    if the cube rests on the ground, its centre is ground_z + size/2 (known exactly);
            otherwise keep the measured z.
    """
    out = np.array(surface_pt, dtype=float)
    horiz = out[:2] - np.asarray(camera_pos, dtype=float)[:2]
    n = np.linalg.norm(horiz)
    if n > 1e-6:
        out[:2] += (cube_size / 2.0) * horiz / n
    if ground_z is not None:
        out[2] = ground_z + cube_size / 2.0
    return out


def colors_from_params(params: Dict[str, list]) -> Dict[str, ColorRange]:
    """params: {'red': [h1,s1,v1,h2,s2,v2, (h1,s1,v1,h2,s2,v2)...], ...} flat lists of 6*N ints."""
    result = {}
    for name, flat in params.items():
        if len(flat) % 6 != 0 or not flat:
            raise ValueError(f'{name}: HSV list must have 6*N values, got {len(flat)}')
        ranges = [(tuple(flat[i:i + 3]), tuple(flat[i + 3:i + 6])) for i in range(0, len(flat), 6)]
        result[name] = ColorRange(name, ranges)
    return result
