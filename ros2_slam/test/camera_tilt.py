#!/usr/bin/env python3
"""Measure the depth camera's mounting tilt (pitch / roll) and height from the floor.

Point the robot at an open patch of floor, then run this while
robot_rgbd.launch.py is publishing. It averages a few depth frames, fits the
dominant plane (RANSAC + least squares) and reports:

  pitch : how far the optical axis looks DOWN from horizontal (deg)
  roll  : rotation about the optical axis (deg, + = right side down)
  height: camera above the floor plane (m)

plus the matching robot_rgbd.launch.py / bringup.launch.py arguments
(base_to_camera_pitch / _roll / _z, in REP-103 radians: + pitch = nose down).

The "dominant plane" is assumed to be the floor -- check the saved overlay
image (green = points used as floor). If a wall or table won instead, aim the
camera at more floor and rerun.

Usage (laptop or Pi, same ROS_DOMAIN_ID as the robot):
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 camera_tilt.py                  # overlay -> ./camera_tilt.png
  /usr/bin/python3 camera_tilt.py --frames 20 --out /tmp/tilt.png
"""
import argparse
import math
import time
import warnings

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

MIN_MM, MAX_MM = 600, 4000        # Astra Pro's trustworthy range
INLIER_M = 0.015                  # RANSAC inlier distance


def depth_to_array(msg: Image) -> np.ndarray:
    if msg.encoding in ('16UC1', 'mono16'):
        arr = np.frombuffer(msg.data, np.dtype('>u2' if msg.is_bigendian else '<u2'))
        return arr.reshape(msg.height, msg.step // 2)[:, :msg.width].astype(np.float32)
    if msg.encoding == '32FC1':
        arr = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(arr * 1000.0)
    raise ValueError(f'unsupported depth encoding {msg.encoding}')


def fit_plane(pts: np.ndarray, iters: int = 400, seed: int = 0):
    """RANSAC plane, refined by SVD on the inliers. Returns (unit normal n, d, inlier mask), n.p + d = 0."""
    rng = np.random.default_rng(seed)
    best = None
    for _ in range(iters):
        a, b, c = pts[rng.choice(len(pts), 3, replace=False)]
        n = np.cross(b - a, c - a)
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            continue
        n /= norm
        inl = np.abs(pts @ n - n @ a) < INLIER_M
        if best is None or inl.sum() > best.sum():
            best = inl
    p = pts[best]
    centroid = p.mean(axis=0)
    # full_matrices=False: only the 3x3 Vt is needed; the default would also
    # build an N x N U matrix (gigabytes for a few thousand inliers).
    n = np.linalg.svd(p - centroid, full_matrices=False)[2][-1]
    d = -n @ centroid
    inl = np.abs(pts @ n + d) < INLIER_M
    return n, d, inl


def tilt_from_plane(n: np.ndarray, d: float):
    """Camera optical frame (x right, y down, z forward) + floor plane -> pitch_down, roll, height."""
    if d < 0:            # orient the normal toward the camera (= world up)
        n, d = -n, -d
    height = d           # distance from the optical center (origin) to the plane
    pitch_down = -math.asin(np.clip(n[2], -1, 1))   # forward axis below horizontal
    # REP-103 roll (+ = left side up = right side down): world 'up' appears
    # rotated toward the camera's left (-x) when the camera rolls right-side-down.
    roll = math.atan2(-n[0], -n[1])
    return pitch_down, roll, height


def median_depth(frames) -> np.ndarray:
    """Per-pixel median over depth frames (mm), ignoring zeros (= no return)."""
    stack = np.stack(frames).astype(np.float32)
    stack[stack == 0] = np.nan
    with np.errstate(all='ignore'), warnings.catch_warnings():
        warnings.simplefilter('ignore', RuntimeWarning)   # all-NaN pixels = never valid -> 0
        return np.nan_to_num(np.nanmedian(stack, axis=0))


def measure_tilt(depth: np.ndarray, info: CameraInfo, max_points: int = 20000, iters: int = 400):
    """Depth image (mm) + its CameraInfo -> floor-plane tilt.

    Returns None when there are too few valid points, else a dict with pitch /
    roll (rad, REP-103: + pitch = nose down, + roll = right side down), height
    (m), floor_ratio, residual_mm, and the floor pixel coords (vs, us).
    """
    h, w = depth.shape
    fx, fy, cx, cy = info.k[0], info.k[4], info.k[2], info.k[5]
    if (info.width, info.height) != (w, h):   # info for another resolution
        sx, sy = w / info.width, h / info.height
        fx, cx, fy, cy = fx * sx, cx * sx, fy * sy, cy * sy

    vs, us = np.nonzero((depth > MIN_MM) & (depth < MAX_MM))
    z = depth[vs, us] / 1000.0
    pts = np.column_stack([(us - cx) * z / fx, (vs - cy) * z / fy, z])
    if len(pts) < 500:
        return None

    sample = pts if len(pts) <= max_points else \
        pts[np.random.default_rng(1).choice(len(pts), max_points, replace=False)]
    n, d, _ = fit_plane(sample, iters=iters)
    inl = np.abs(pts @ n + d) < INLIER_M
    pitch, roll, height = tilt_from_plane(n, d)
    return {
        'pitch': pitch, 'roll': roll, 'height': height,
        'floor_ratio': float(inl.mean()),
        'residual_mm': float(np.abs(pts[inl] @ n + d).std() * 1000),
        'valid_points': len(pts),
        'floor_vs': vs[inl], 'floor_us': us[inl],
    }


class Grabber(Node):
    def __init__(self, depth_topic, info_topic):
        super().__init__('camera_tilt')
        self.frames, self.info = [], None
        self.create_subscription(Image, depth_topic, self._depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._info, qos_profile_sensor_data)

    def _depth(self, msg):
        self.frames.append(depth_to_array(msg))

    def _info(self, msg):
        self.info = msg


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    ap.add_argument('--frames', type=int, default=10)
    ap.add_argument('--depth-topic', default='/camera/depth_raw/image')
    ap.add_argument('--info-topic', default='/camera/depth_raw/camera_info')
    ap.add_argument('--out', default='camera_tilt.png')
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args.depth_topic, args.info_topic)
    t0 = time.time()
    while (len(node.frames) < args.frames or node.info is None) and time.time() - t0 < 20:
        rclpy.spin_once(node, timeout_sec=0.1)
    if not node.frames or node.info is None:
        print('No depth frames / camera_info received -- is robot_rgbd.launch.py running '
              'with the same ROS_DOMAIN_ID?')
        return 1

    depth = median_depth(node.frames)
    h, w = depth.shape
    t = measure_tilt(depth, node.info)
    if t is None:
        print(f'Too few valid depth points in {MIN_MM}-{MAX_MM}mm -- point the camera at more floor.')
        return 1
    pitch, roll, height, ratio = t['pitch'], t['roll'], t['height'], t['floor_ratio']

    vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255.0 / MAX_MM), cv2.COLORMAP_JET)
    vis[depth == 0] = 0
    overlay = vis.copy()
    overlay[t['floor_vs'], t['floor_us']] = (0, 255, 0)
    vis = cv2.addWeighted(vis, 0.4, overlay, 0.6, 0)
    cv2.imwrite(args.out, cv2.resize(vis, (w * 2, h * 2), interpolation=cv2.INTER_NEAREST))

    print(f'frames used      : {len(node.frames)}  ({w}x{h}, {t["valid_points"]} valid points)')
    print(f'floor plane      : {ratio * 100:.0f}% of valid points, residual {t["residual_mm"]:.1f} mm  -> {args.out}')
    print(f'pitch (down)     : {math.degrees(pitch):6.1f} deg')
    print(f'roll             : {math.degrees(roll):6.1f} deg  (+ = right side down)')
    print(f'camera height    : {height:6.3f} m above the floor plane')
    print()
    print('launch arguments (base_link on the floor under the wheel center):')
    print(f'  base_to_camera_pitch:={pitch:.4f} base_to_camera_roll:={roll:.4f} base_to_camera_z:={height:.3f}')
    if ratio < 0.3:
        print('\nWARNING: the plane covers <30% of the view -- check the overlay; it may not be the floor.')
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
