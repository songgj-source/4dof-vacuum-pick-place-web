#!/usr/bin/env python3
"""Live web viewer for the robot's Astra Pro RGB + depth streams.

Runs on the Raspberry Pi next to robot_rgbd.launch.py: subscribes to the
camera topics locally (the camera devices themselves are already opened by
openni2/usb_cam, so this never touches /dev/video* or OpenNI) and serves

  http://<pi-ip>:8080/                 page with both streams + live stats
  http://<pi-ip>:8080/stream/rgb.mjpg  MJPEG of the RGB image
  http://<pi-ip>:8080/stream/depth.mjpg?max=4000
                                       MJPEG of the depth image, colorized 0..max mm
  http://<pi-ip>:8080/api/stats        JSON: fps, depth min/median/max, center distance,
                                       camera tilt (pitch/roll/height from the floor plane)
  http://<pi-ip>:8080/api/depth_at?x=0.5&y=0.5
                                       JSON: depth (mm) at a normalized image point
  GET  /api/mount                      JSON: camera mount saved for robot_rgbd.launch.py
  POST /api/mount {"include_height": false}
                                       save the current pitch/roll (+ height) to
                                       ~/.ros/car2_camera_mount.yaml; applied the next
                                       time robot_rgbd.launch.py starts
  GET  /api/robot                      JSON: is robot_rgbd.launch.py running, last log lines
  GET  /api/roi                        JSON: saved RoiRatios + what the running rtabmap has
  POST /api/roi {"roi": [l, r, t, b], "grid": true}
                                       save RoiRatios (~/.ros/car2_roi.yaml) and push them to
                                       the laptop's rtabmap (Vis/ + Kp/RoiRatios, optionally
                                       Grid/DepthRoiRatios); re-pushed whenever rtabmap restarts
  POST /api/robot/restart              stop robot_rgbd.launch.py (however it was started)
                                       and start it again in tmux session 'robot_rgbd'
  POST /api/drive {"linear": 0.1, "angular": 0.0}
                                       drive (m/s, rad/s; + angular = left) via /cmd_vel for
                                       drive_hold_s (0.5 s) -- resend to keep moving
  POST /api/drive/stop                 stop now
  GET  /api/drive                      JSON: current command, limits, driver connected, /odom

Driving publishes geometry_msgs/Twist on /cmd_vel for car2_serial_node.
car2_serial_node keeps executing the LAST /cmd_vel forever, so every drive
command here expires after drive_hold_s: the page resends while a button / key
is held, and if it stops (released, tab hidden, browser closed, WiFi drop) the
node itself publishes a zero Twist. Speeds are capped at what the wheels can
do (3400 step/s, 0.032 m radius, 0.43 m track -> ~0.16 m/s, ~0.77 rad/s).

JPEG encoding only happens while a browser is watching, once per new frame
regardless of how many browsers are open. The tilt (same floor-plane fit as
camera_tilt.py) is recomputed about once a second, also only while watched.

Usage (on the Pi, robot_rgbd.launch.py already running, same ROS_DOMAIN_ID):
  source /opt/ros/jazzy/setup.bash
  /usr/bin/python3 camera_test.py
  /usr/bin/python3 camera_test.py --ros-args -p port:=8081 -p max_fps:=10.0
"""
import collections
import json
import math
import os
import shlex
import shutil
import signal
import statistics
import subprocess
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from urllib.parse import parse_qs, urlparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CameraInfo, Image
import yaml

from camera_tilt import measure_tilt, median_depth


def image_to_array(msg: Image) -> np.ndarray:
    """sensor_msgs/Image -> numpy without cv_bridge (BGR for color, uint16 mm for depth)."""
    if msg.encoding in ('rgb8', 'bgr8'):
        arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
        arr = arr.reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR) if msg.encoding == 'rgb8' else arr.copy()
    if msg.encoding in ('16UC1', 'mono16'):
        dtype = np.dtype('>u2' if msg.is_bigendian else '<u2')
        arr = np.frombuffer(msg.data, dtype).reshape(msg.height, msg.step // 2)[:, :msg.width]
        return arr.astype(np.uint16)
    if msg.encoding == '32FC1':  # meters -> mm
        arr = np.frombuffer(msg.data, np.float32).reshape(msg.height, msg.step // 4)[:, :msg.width]
        return np.nan_to_num(arr * 1000.0).astype(np.uint16)
    raise ValueError(f'unsupported encoding {msg.encoding}')


class Stream:
    """Latest frame of one topic + its rate, and a JPEG cache shared by all viewers."""

    def __init__(self):
        self.lock = threading.Condition()
        self.frame = None
        self.seq = 0
        self.stamps = []          # receive times, for fps
        self.jpeg_cache = {}      # (seq, key) -> bytes

    def put(self, frame):
        with self.lock:
            self.frame = frame
            self.seq += 1
            now = time.time()
            self.stamps = [t for t in self.stamps if now - t < 2.0] + [now]
            self.jpeg_cache.clear()
            self.lock.notify_all()

    def fps(self):
        with self.lock:
            now = time.time()
            recent = [t for t in self.stamps if now - t < 2.0]
            return len(recent) / 2.0

    def wait_newer(self, seq, timeout=2.0):
        with self.lock:
            self.lock.wait_for(lambda: self.seq != seq, timeout=timeout)
            return self.seq, self.frame

    def jpeg(self, seq, key, render):
        with self.lock:
            cached = self.jpeg_cache.get((seq, key))
        if cached is None:
            cached = render()
            with self.lock:
                if self.seq == seq:
                    self.jpeg_cache[(seq, key)] = cached
        return cached


def colorize_depth(depth: np.ndarray, max_mm: int) -> np.ndarray:
    vis = cv2.applyColorMap(cv2.convertScaleAbs(depth, alpha=255.0 / max(max_mm, 1)), cv2.COLORMAP_JET)
    vis[depth == 0] = 0  # no return -> black
    # Upscale small depth (320x240) so it lines up with the 640x480 RGB view.
    if vis.shape[1] < 640:
        vis = cv2.resize(vis, (640, 640 * vis.shape[0] // vis.shape[1]), interpolation=cv2.INTER_NEAREST)
    return vis


class CameraViewer(Node):

    def __init__(self):
        super().__init__('camera_web_viewer')
        self.declare_parameter('rgb_topic', '/camera/color/image_raw')
        self.declare_parameter('depth_topic', '/camera/depth_raw/image')
        self.declare_parameter('depth_info_topic', '/camera/depth_raw/camera_info')
        self.declare_parameter('tilt_period_s', 1.0)
        # Must match CAMERA_MOUNT_FILE in robot_rgbd.launch.py.
        self.declare_parameter('mount_file', os.environ.get(
            'CAR2_CAMERA_MOUNT_FILE', os.path.expanduser('~/.ros/car2_camera_mount.yaml')))
        self.declare_parameter('port', 8080)
        self.declare_parameter('max_fps', 15.0)
        self.declare_parameter('jpeg_quality', 80)
        self.declare_parameter('depth_max_mm', 4000)

        self.port = self.get_parameter('port').value
        self.min_period = 1.0 / max(self.get_parameter('max_fps').value, 0.1)
        self.jpeg_quality = int(self.get_parameter('jpeg_quality').value)
        self.depth_max_mm = int(self.get_parameter('depth_max_mm').value)

        self.rgb = Stream()
        self.depth = Stream()
        self.depth_history = collections.deque(maxlen=5)   # for the tilt's multi-frame median
        self.depth_info = None
        self.tilt = None
        self.tilt_history = collections.deque(maxlen=5)     # median of these is what gets saved
        self.last_viewed = 0.0
        self.mount_file = self.get_parameter('mount_file').value
        # This file lives in <ws>/test/, so the workspace is one level up.
        self.declare_parameter('robot_ws', os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        self.robot = RobotLauncher(self.get_parameter('robot_ws').value, self.get_logger())
        self.declare_parameter('rtabmap_node', '/rtabmap/rtabmap')
        self.declare_parameter('roi_file', os.path.expanduser('~/.ros/car2_roi.yaml'))
        self.roi = RoiSync(self, self.get_parameter('rtabmap_node').value,
                           self.get_parameter('roi_file').value)
        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('depth_info_topic').value
        self.create_subscription(Image, rgb_topic, lambda m: self._on(self.rgb, m), qos_profile_sensor_data)
        self.create_subscription(Image, depth_topic, lambda m: self._on(self.depth, m), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, self._on_info, qos_profile_sensor_data)
        self.get_logger().info(f'subscribed: {rgb_topic}, {depth_topic}, {info_topic}')
        threading.Thread(target=self._tilt_loop, daemon=True).start()

        # ---- driving (/cmd_vel)
        self.declare_parameter('max_linear', 0.16)     # m/s  (wheel limit ~0.167)
        self.declare_parameter('max_angular', 0.75)    # rad/s (in-place spin limit ~0.77)
        self.declare_parameter('drive_hold_s', 0.5)    # a command lives this long unless resent
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)
        self.drive_hold_s = float(self.get_parameter('drive_hold_s').value)
        self.drive_lock = threading.Lock()
        self.drive_cmd = (0.0, 0.0)
        self.drive_deadline = 0.0
        self.drive_zero_left = 0          # zero Twists still to send after a stop
        self.odom = None
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 10)
        self.create_timer(0.1, self._drive_tick)

    def _on(self, stream: Stream, msg: Image):
        try:
            frame = image_to_array(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc), throttle_duration_sec=5.0)
            return
        stream.put(frame)
        if stream is self.depth:
            self.depth_history.append(frame)

    def _on_info(self, msg: CameraInfo):
        self.depth_info = msg

    def _on_odom(self, msg: Odometry):
        self.odom = (msg.twist.twist.linear.x, msg.twist.twist.angular.z, time.time())

    def drive(self, linear: float, angular: float):
        linear = max(-self.max_linear, min(self.max_linear, float(linear)))
        angular = max(-self.max_angular, min(self.max_angular, float(angular)))
        with self.drive_lock:
            self.drive_cmd = (linear, angular)
            self.drive_deadline = time.time() + self.drive_hold_s
        self._publish(linear, angular)   # react now, not at the next tick

    def stop_drive(self):
        with self.drive_lock:
            self.drive_cmd = (0.0, 0.0)
            self.drive_deadline = 0.0
            self.drive_zero_left = 3
        self._publish(0.0, 0.0)

    def _publish(self, linear: float, angular: float):
        msg = Twist()
        msg.linear.x, msg.angular.z = linear, angular
        self.cmd_pub.publish(msg)

    def _drive_tick(self):
        # Keep /cmd_vel alive while a command is fresh; when it expires, send a
        # few zero Twists (the driver would otherwise keep the last command).
        with self.drive_lock:
            if self.drive_deadline:
                if time.time() < self.drive_deadline:
                    cmd = self.drive_cmd
                else:
                    self.drive_cmd, self.drive_deadline, self.drive_zero_left = (0.0, 0.0), 0.0, 3
                    cmd = (0.0, 0.0)
                    self.drive_zero_left -= 1
            elif self.drive_zero_left > 0:
                self.drive_zero_left -= 1
                cmd = (0.0, 0.0)
            else:
                return
        self._publish(*cmd)

    def drive_status(self) -> dict:
        with self.drive_lock:
            linear, angular = self.drive_cmd
            active = time.time() < self.drive_deadline
        odom = self.odom if self.odom and time.time() - self.odom[2] < 2.0 else None
        return {
            'linear': linear if active else 0.0, 'angular': angular if active else 0.0, 'moving': active,
            'max_linear': self.max_linear, 'max_angular': self.max_angular, 'hold_s': self.drive_hold_s,
            # car2_serial_node (or anything else) listening on /cmd_vel
            'driver_connected': self.cmd_pub.get_subscription_count() > 0,
            'odom': None if odom is None else {'linear': round(odom[0], 3), 'angular': round(odom[1], 3)},
        }

    def _tilt_loop(self):
        period = self.get_parameter('tilt_period_s').value
        while rclpy.ok():
            time.sleep(period)
            # Only spend Pi CPU on RANSAC while a browser is polling the stats.
            if time.time() - self.last_viewed > 3.0 or self.depth_info is None or not self.depth_history:
                continue
            # Fewer RANSAC iterations / points than camera_tilt.py: this runs
            # continuously on the Pi; the result is still within ~0.1 deg.
            t = measure_tilt(median_depth(list(self.depth_history)), self.depth_info,
                             max_points=8000, iters=150)
            self.tilt = None if t is None else {
                'pitch_deg': round(math.degrees(t['pitch']), 1),
                'roll_deg': round(math.degrees(t['roll']), 1),
                'height_m': round(t['height'], 3),
                'floor_pct': round(100 * t['floor_ratio']),
                'residual_mm': round(t['residual_mm'], 1),
                'pitch_rad': round(t['pitch'], 4),
                'roll_rad': round(t['roll'], 4),
                'time': time.time(),
            }
            if self.tilt is not None:
                self.tilt_history.append(self.tilt)

    def encode(self, img: np.ndarray) -> bytes:
        ok, buf = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
        return buf.tobytes() if ok else b''

    def stats(self) -> dict:
        self.last_viewed = time.time()
        out = {'rgb_fps': self.rgb.fps(), 'depth_fps': self.depth.fps()}
        tilt = self.tilt
        if tilt is not None and time.time() - tilt['time'] < 5.0:
            out['tilt'] = {k: v for k, v in tilt.items() if k != 'time'}
        with self.rgb.lock:
            rgb = self.rgb.frame
        with self.depth.lock:
            depth = self.depth.frame
        if rgb is not None:
            out['rgb_size'] = [int(rgb.shape[1]), int(rgb.shape[0])]
        if depth is not None:
            valid = depth[depth > 0]
            h, w = depth.shape
            center = depth[h // 2 - 2:h // 2 + 3, w // 2 - 2:w // 2 + 3]
            center = center[center > 0]
            out.update({
                'depth_size': [int(w), int(h)],
                'valid_pct': round(100.0 * valid.size / depth.size, 1),
                'min_mm': int(valid.min()) if valid.size else None,
                'median_mm': int(np.median(valid)) if valid.size else None,
                'max_mm': int(valid.max()) if valid.size else None,
                'center_mm': int(np.median(center)) if center.size else None,
            })
        return out

    def saved_mount(self) -> dict:
        try:
            with open(self.mount_file) as f:
                lines = f.read().splitlines()
        except FileNotFoundError:
            return {'file': self.mount_file, 'saved': None}
        values = {}
        for line in lines:
            key, sep, val = line.partition(':')
            if sep and key.startswith('base_to_camera_'):
                values[key] = float(val.split('#')[0])
        saved_at = os.path.getmtime(self.mount_file)
        return {'file': self.mount_file, 'saved': values,
                'saved_at': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(saved_at))}

    def save_mount(self, include_height: bool) -> dict:
        now = time.time()
        recent = [t for t in self.tilt_history if now - t['time'] < 10.0]
        if not recent:
            raise ValueError('최근 측정값이 없습니다. 페이지를 열어 둔 채로 몇 초 기다린 뒤 다시 누르세요.')
        floor_pct = min(t['floor_pct'] for t in recent)
        if floor_pct < 30:
            raise ValueError(f'바닥 인식이 {floor_pct}%로 낮아 저장하지 않았습니다. 바닥이 넓게 보이게 한 뒤 다시 누르세요.')
        pitch = statistics.median(t['pitch_rad'] for t in recent)
        roll = statistics.median(t['roll_rad'] for t in recent)
        height = statistics.median(t['height_m'] for t in recent)
        lines = [
            '# Camera mount for robot_rgbd.launch.py (base_link -> camera_link, REP-103 rad/m).',
            f'# Saved {time.strftime("%Y-%m-%d %H:%M:%S")} from the camera_test.py web UI:',
            f'# median of {len(recent)} floor-plane fits, '
            f'pitch {math.degrees(pitch):.1f} deg (down), roll {math.degrees(roll):.1f} deg (+ right side down), '
            f'floor >= {floor_pct}%.',
            f'base_to_camera_pitch: {pitch:.4f}',
            f'base_to_camera_roll: {roll:.4f}',
        ]
        if include_height:
            lines.append(f'base_to_camera_z: {height:.3f}')
        os.makedirs(os.path.dirname(self.mount_file), exist_ok=True)
        tmp = self.mount_file + '.tmp'
        with open(tmp, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.replace(tmp, self.mount_file)
        self.get_logger().info(f'saved camera mount to {self.mount_file}: '
                               f'pitch={pitch:.4f} roll={roll:.4f}' + (f' z={height:.3f}' if include_height else ''))
        return self.saved_mount()

    def depth_at(self, x: float, y: float):
        with self.depth.lock:
            depth = self.depth.frame
        if depth is None:
            return None
        h, w = depth.shape
        px = min(max(int(x * w), 0), w - 1)
        py = min(max(int(y * h), 0), h - 1)
        # Median of a 5x5 patch: single pixels are often 0 (no return) at edges.
        patch = depth[max(py - 2, 0):py + 3, max(px - 2, 0):px + 3]
        patch = patch[patch > 0]
        return {'x': px, 'y': py, 'mm': int(np.median(patch)) if patch.size else None}


PAGE = """<!doctype html>
<html lang="ko"><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>car2 camera</title>
<style>
  :root { color-scheme: dark; --bg:#16181d; --panel:#21252c; --text:#e6e8eb; --muted:#9aa3ad; --accent:#4cc2ff; }
  body { margin:0; padding:16px; background:var(--bg); color:var(--text); font:14px system-ui, sans-serif; }
  h1 { font-size:18px; margin:0 0 12px; }
  .views { display:grid; grid-template-columns:repeat(auto-fit, minmax(300px, 1fr)); gap:12px; }
  .view { background:var(--panel); border-radius:8px; padding:8px; }
  .view h2 { font-size:13px; margin:0 0 6px; color:var(--muted); font-weight:500; display:flex; justify-content:space-between; }
  .frame { position:relative; }
  .frame img { width:100%; display:block; border-radius:4px; background:#000; aspect-ratio:4/3; }
  #depth { cursor:crosshair; }
  .marker { position:absolute; width:14px; height:14px; margin:-7px 0 0 -7px; border:2px solid #fff;
            border-radius:50%; pointer-events:none; box-shadow:0 0 0 1px #000; }
  .marker span { position:absolute; left:16px; top:-4px; white-space:nowrap; background:#000c; padding:1px 5px;
                 border-radius:3px; font-size:12px; }
  .stats { margin-top:12px; background:var(--panel); border-radius:8px; padding:10px 12px;
           display:grid; grid-template-columns:repeat(auto-fit, minmax(140px, 1fr)); gap:6px 16px; }
  .stats div b { display:block; color:var(--muted); font-weight:400; font-size:12px; }
  .stats div span { font-variant-numeric:tabular-nums; font-size:16px; }
  label { color:var(--muted); font-size:12px; display:flex; align-items:center; gap:6px; }
  input[type=range] { width:120px; }
  .hint { color:var(--muted); font-size:12px; margin-top:8px; }
  .drive { margin-top:12px; background:var(--panel); border-radius:8px; padding:12px; display:flex; flex-wrap:wrap;
           gap:16px 24px; align-items:center; }
  .pad { display:grid; grid-template-columns:repeat(3, 64px); grid-template-rows:repeat(3, 64px); gap:6px;
         touch-action:none; user-select:none; -webkit-user-select:none; }
  .pad button { border:0; border-radius:10px; font-size:24px; background:#2e343d; color:var(--text); cursor:pointer; }
  .pad button.on { background:var(--accent); color:#06222f; }
  #stopbtn { background:#5a2a2a; color:#ffcdd2; font-size:20px; }
  .drivectl { flex:1; min-width:260px; display:flex; flex-direction:column; gap:8px; }
  .spd { display:flex; align-items:center; gap:10px; }
  .spd b { min-width:5em; color:var(--muted); font-weight:400; font-size:13px; }
  .spd input { flex:1; }
  .spd span { min-width:6.5em; font-variant-numeric:tabular-nums; }
  .drivestate { font-size:13px; font-variant-numeric:tabular-nums; }
  .drivestate.off { color:#ff8a80; } .drivestate.moving { color:var(--accent); }
  .tilt .wide { grid-column:1 / -1; }
  .tilt code { font-size:12px; color:var(--accent); word-break:break-all; }
  .warn { color:#ffb74c; }
  .mount { display:flex; flex-wrap:wrap; align-items:center; gap:10px 16px; padding-top:6px;
           border-top:1px solid #ffffff14; }
  .mount button { background:var(--accent); color:#06222f; border:0; border-radius:6px; padding:7px 14px;
                  font:600 14px system-ui, sans-serif; cursor:pointer; }
  .mount button:disabled { opacity:.5; cursor:default; }
  .mount label { font-size:13px; }
  #mountv { font-size:12px; color:var(--muted); }
  #mountv.ok { color:#7ddc8a; } #mountv.err { color:#ff8a80; }
  .roi { position:absolute; inset:0; width:100%; height:100%; pointer-events:none; }
  .roi path { fill:#ff3b30; fill-opacity:.32; fill-rule:evenodd; }
  .roi rect { fill:none; stroke:#ff8a80; stroke-width:2; stroke-dasharray:6 4; vector-effect:non-scaling-stroke; }
  .roilabel { position:absolute; pointer-events:none; font-size:11px; background:#000b; color:#ffcdd2;
              padding:1px 6px; border-radius:3px; white-space:nowrap; }
  .roipanel { grid-template-columns:repeat(auto-fit, minmax(230px, 1fr)); }
  .roipanel .wide { grid-column:1 / -1; }
  .roictl { display:flex; align-items:center; gap:8px; }
  .roictl b { min-width:3.2em; display:inline; }
  .roictl input[type=range] { flex:1; min-width:60px; }
  .roictl input[type=number] { width:4.5em; background:#14171c; color:var(--text); border:1px solid #ffffff22;
                               border-radius:4px; padding:3px 4px; }
  .roibar { display:flex; flex-wrap:wrap; align-items:center; gap:10px 16px; }
  .roibar button { background:var(--accent); color:#06222f; border:0; border-radius:6px; padding:7px 14px;
                   font:600 14px system-ui, sans-serif; cursor:pointer; }
  .roibar button.ghost { background:transparent; color:var(--muted); border:1px solid #ffffff33; }
  .roibar label { font-size:13px; }
  #roistate { font-size:12px; color:var(--muted); }
  #roistate.ok { color:#7ddc8a; } #roistate.err { color:#ff8a80; } #roistate.pending { color:#ffb74c; }
  .roinote { font-size:12px; color:var(--muted); }
  .robot .wide { grid-column:1 / -1; }
  .robotbar { display:flex; flex-wrap:wrap; align-items:center; gap:10px 16px; }
  #restart { background:#ff8a65; color:#2b0d03; border:0; border-radius:6px; padding:7px 14px;
             font:600 14px system-ui, sans-serif; cursor:pointer; }
  #restart:disabled { opacity:.5; cursor:default; }
  #robotstate { font-size:13px; }
  .on { color:#7ddc8a; } .off { color:#ff8a80; } .busy { color:#ffb74c; }
  #robotlog { margin:0; font-size:11px; color:var(--muted); white-space:pre-wrap; word-break:break-all;
              max-height:9em; overflow:auto; }
</style></head><body>
<h1>car2 카메라 실시간 보기</h1>
<div class="views">
  <div class="view"><h2>RGB</h2><div class="frame"><img id="rgb" src="/stream/rgb.mjpg" alt="RGB">__ROI_OVERLAY__</div></div>
  <div class="view">
    <h2>Depth
      <label>최대 <input id="max" type="range" min="1000" max="10000" step="500" value="__MAX__"> <span id="maxv">__MAX__</span> mm</label>
    </h2>
    <div class="frame"><img id="depth" src="/stream/depth.mjpg?max=__MAX__" alt="Depth">
      __ROI_OVERLAY__
      <div id="marker" class="marker" hidden><span id="markerv"></span></div></div>
  </div>
</div>
<div class="drive">
  <div class="pad">
    <span></span><button class="dir" data-l="1" data-a="0" title="전진 (↑ / W)">▲</button><span></span>
    <button class="dir" data-l="0" data-a="1" title="좌회전 (← / A)">◀</button>
    <button id="stopbtn" title="정지 (Space)">■</button>
    <button class="dir" data-l="0" data-a="-1" title="우회전 (→ / D)">▶</button>
    <span></span><button class="dir" data-l="-1" data-a="0" title="후진 (↓ / S)">▼</button><span></span>
  </div>
  <div class="drivectl">
    <div class="spd"><b>직진 속도</b><input id="vlin" type="range" min="0.02" max="0.16" step="0.01" value="0.08">
      <span id="vlinv">0.08 m/s</span></div>
    <div class="spd"><b>회전 속도</b><input id="vang" type="range" min="0.1" max="0.75" step="0.05" value="0.4">
      <span id="vangv">0.40 rad/s</span></div>
    <div id="drivestate" class="drivestate">드라이버 확인 중…</div>
    <div class="hint">누르고 있는 동안만 움직입니다 (떼면 0.5초 안에 정지). 키보드: ↑↓←→ 또는 WASD, 대각선은 두 키 동시,
      Space 정지, +/− 직진 속도. 탭을 바꾸거나 창을 벗어나면 자동 정지합니다.</div>
  </div>
</div>
<div class="stats">
  <div><b>RGB fps</b><span id="rgb_fps">-</span></div>
  <div><b>Depth fps</b><span id="depth_fps">-</span></div>
  <div><b>중앙 거리</b><span id="center_mm">-</span></div>
  <div><b>최소 / 중간 / 최대</b><span id="range">-</span></div>
  <div><b>유효 픽셀</b><span id="valid_pct">-</span></div>
  <div><b>해상도 (RGB / Depth)</b><span id="sizes">-</span></div>
</div>
<div class="stats tilt">
  <div><b>Pitch (아래로 숙인 각)</b><span id="pitch">-</span></div>
  <div><b>Roll (+ 오른쪽이 아래)</b><span id="roll">-</span></div>
  <div><b>카메라 높이</b><span id="height">-</span></div>
  <div><b>바닥 인식 / 평면 오차</b><span id="floor">-</span></div>
  <div class="wide"><b>launch 인자</b><code id="args">-</code></div>
  <div class="wide mount">
    <button id="save">현재 값으로 설정</button>
    <label><input id="withz" type="checkbox"> 높이(z)도 저장</label>
    <span id="mountv">저장된 값 확인 중…</span>
  </div>
</div>
<div class="stats roipanel">
  <div class="wide"><b>로봇 몸체 제외 영역 (RoiRatios) — 화면 가장자리에서 잘라낼 비율. 빨간 영역은 RTAB-Map이 쓰지 않습니다.</b></div>
  <div class="roictl"><b>왼쪽</b><input id="roi0" type="range" min="0" max="0.5" step="0.025" value="0"><input id="roin0" type="number" min="0" max="0.5" step="0.025" value="0"></div>
  <div class="roictl"><b>오른쪽</b><input id="roi1" type="range" min="0" max="0.5" step="0.025" value="0"><input id="roin1" type="number" min="0" max="0.5" step="0.025" value="0"></div>
  <div class="roictl"><b>위</b><input id="roi2" type="range" min="0" max="0.5" step="0.025" value="0"><input id="roin2" type="number" min="0" max="0.5" step="0.025" value="0"></div>
  <div class="roictl"><b>아래</b><input id="roi3" type="range" min="0" max="0.5" step="0.025" value="0"><input id="roin3" type="number" min="0" max="0.5" step="0.025" value="0"></div>
  <div class="wide roibar">
    <label><input id="roigrid" type="checkbox"> 2D/3D 맵 생성에도 적용 (Grid/DepthRoiRatios)</label>
    <button id="roiapply">적용</button>
    <button id="roizero" class="ghost">0으로</button>
    <span id="roistate">확인 중…</span>
  </div>
  <div class="wide roinote">값은 0.025 단위로 맞춰집니다 (영상 크기가 나누어떨어지지 않으면 RTAB-Map이 ROI를 무시함). 특징점(Vis/RoiRatios, Kp/RoiRatios)에는 항상 적용됩니다. Depth 화면의 표시는 RGB와 시야가 조금 달라 대략적인 위치입니다.</div>
</div>
<div class="stats robot">
  <div class="wide robotbar">
    <button id="restart">robot_rgbd 재시작</button>
    <span id="robotstate">상태 확인 중…</span>
  </div>
  <pre id="robotlog" class="wide"></pre>
</div>
<p class="hint">Depth 화면을 클릭하면 그 지점의 거리를 표시합니다. 검은 부분은 측정값이 없는 곳입니다 (Astra Pro는 약 0.6m 이내를 잘 못 잽니다).<br>
Pitch / Roll / 높이는 화면에서 가장 큰 평면을 바닥으로 보고 1초마다 계산합니다. 바닥이 화면에 넓게 보여야 정확하며, 바닥 인식이 30% 미만이면 주황색으로 표시됩니다.</p>
<script>
const $ = id => document.getElementById(id);
const mm = v => v == null ? '-' : (v / 1000).toFixed(2) + ' m';
let pick = null;
async function refresh() {
  try {
    const s = await (await fetch('/api/stats')).json();
    $('rgb_fps').textContent = s.rgb_fps.toFixed(1);
    $('depth_fps').textContent = s.depth_fps.toFixed(1);
    $('center_mm').textContent = mm(s.center_mm);
    $('range').textContent = s.min_mm == null ? '-' : `${mm(s.min_mm)} / ${mm(s.median_mm)} / ${mm(s.max_mm)}`;
    $('valid_pct').textContent = s.valid_pct == null ? '-' : s.valid_pct + ' %';
    $('sizes').textContent = `${s.rgb_size ? s.rgb_size.join('x') : '-'} / ${s.depth_size ? s.depth_size.join('x') : '-'}`;
    const t = s.tilt;
    $('pitch').textContent = t ? t.pitch_deg.toFixed(1) + '°' : '측정 중…';
    $('roll').textContent = t ? (t.roll_deg > 0 ? '+' : '') + t.roll_deg.toFixed(1) + '°' : '-';
    $('height').textContent = t ? t.height_m.toFixed(3) + ' m' : '-';
    $('floor').textContent = t ? `${t.floor_pct} % / ${t.residual_mm} mm` : '-';
    // <30% floor: the dominant plane may be a wall/table, not the floor.
    $('floor').className = t && t.floor_pct < 30 ? 'warn' : '';
    $('args').textContent = t ? `base_to_camera_pitch:=${t.pitch_rad} base_to_camera_roll:=${t.roll_rad} base_to_camera_z:=${t.height_m}` : '-';
    if (pick) {
      const d = await (await fetch(`/api/depth_at?x=${pick.x}&y=${pick.y}`)).json();
      $('markerv').textContent = mm(d.mm);
    }
  } catch (e) { $('rgb_fps').textContent = '연결 끊김'; }
}
setInterval(refresh, 500); refresh();

// ---- driving --------------------------------------------------------------
const held = new Set();               // 'up' | 'down' | 'left' | 'right' currently pressed
let driveTimer = null;
const KEY = { ArrowUp: 'up', KeyW: 'up', ArrowDown: 'down', KeyS: 'down',
              ArrowLeft: 'left', KeyA: 'left', ArrowRight: 'right', KeyD: 'right' };
const vlin = () => parseFloat($('vlin').value), vang = () => parseFloat($('vang').value);
function driveCmd() {
  const l = (held.has('up') ? 1 : 0) - (held.has('down') ? 1 : 0);
  const a = (held.has('left') ? 1 : 0) - (held.has('right') ? 1 : 0);
  // reversing with a turn key: steer like a car (left key turns the nose left)
  return { linear: l * vlin(), angular: a * vang() * (l < 0 ? -1 : 1) };
}
function sendDrive() {
  const c = driveCmd();
  if (!c.linear && !c.angular) { stopDrive(); return; }
  fetch('/api/drive', { method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(c) })
    .catch(() => {});
}
function updateDrive() {
  document.querySelectorAll('.pad .dir').forEach(b => b.classList.toggle('on', held.has(b.dataset.dir)));
  if (held.size) { sendDrive(); if (!driveTimer) driveTimer = setInterval(sendDrive, 100); }
  else stopDrive();
}
function stopDrive() {
  held.clear(); clearInterval(driveTimer); driveTimer = null;
  document.querySelectorAll('.pad .dir').forEach(b => b.classList.remove('on'));
  fetch('/api/drive/stop', { method: 'POST' }).catch(() => {});
}
const dirName = b => b.dataset.l === '1' ? 'up' : b.dataset.l === '-1' ? 'down' : b.dataset.a === '1' ? 'left' : 'right';
document.querySelectorAll('.pad .dir').forEach(b => {
  b.dataset.dir = dirName(b);
  b.addEventListener('pointerdown', e => { e.preventDefault(); b.setPointerCapture(e.pointerId); held.add(b.dataset.dir); updateDrive(); });
  for (const ev of ['pointerup', 'pointercancel', 'lostpointercapture'])
    b.addEventListener(ev, () => { held.delete(b.dataset.dir); updateDrive(); });
});
$('stopbtn').addEventListener('click', stopDrive);
addEventListener('keydown', e => {
  if (e.target.tagName === 'INPUT' && e.target.type !== 'range') return;   // typing in a number box
  if (e.code === 'Space') { e.preventDefault(); stopDrive(); return; }
  if (e.key === '+' || e.key === '=') { $('vlin').stepUp(); $('vlin').dispatchEvent(new Event('input')); return; }
  if (e.key === '-' || e.key === '_') { $('vlin').stepDown(); $('vlin').dispatchEvent(new Event('input')); return; }
  const d = KEY[e.code];
  if (!d) return;
  e.preventDefault();
  if (!held.has(d)) { held.add(d); updateDrive(); }
});
addEventListener('keyup', e => { const d = KEY[e.code]; if (d && held.delete(d)) updateDrive(); });
addEventListener('blur', () => held.size && stopDrive());
document.addEventListener('visibilitychange', () => document.hidden && held.size && stopDrive());
for (const [id, unit] of [['vlin', 'm/s'], ['vang', 'rad/s']])
  $(id).addEventListener('input', e => { $(id + 'v').textContent = parseFloat(e.target.value).toFixed(2) + ' ' + unit; });
async function driveStatus() {
  try {
    const d = await (await fetch('/api/drive')).json();
    $('vlin').max = d.max_linear; $('vang').max = d.max_angular;
    const st = $('drivestate');
    const odom = d.odom ? `실제(odom) ${d.odom.linear.toFixed(2)} m/s, ${d.odom.angular.toFixed(2)} rad/s` : 'odom 없음';
    if (!d.driver_connected) {
      st.className = 'drivestate off';
      st.textContent = '차량 드라이버가 /cmd_vel을 구독하지 않습니다 — robot_rgbd(또는 car2_driver)가 꺼져 있습니다';
    } else {
      st.className = 'drivestate' + (d.moving ? ' moving' : '');
      st.textContent = (d.moving ? `명령 ${d.linear.toFixed(2)} m/s, ${d.angular.toFixed(2)} rad/s` : '정지') + ' · ' + odom;
    }
  } catch (e) { $('drivestate').textContent = '상태 확인 실패'; }
}
setInterval(driveStatus, 300); driveStatus();
const deg = r => (r * 180 / Math.PI).toFixed(1) + '°';
function showMount(m, cls, prefix) {
  const v = m.saved;
  $('mountv').className = cls || '';
  if (!v) { $('mountv').textContent = (prefix || '') + `저장된 값 없음 (${m.file})`; return; }
  const z = v.base_to_camera_z != null ? `, z ${v.base_to_camera_z.toFixed(3)} m` : '';
  $('mountv').textContent = (prefix || '저장된 값: ') +
    `pitch ${deg(v.base_to_camera_pitch)}, roll ${deg(v.base_to_camera_roll)}${z} — ${m.saved_at}`;
}
fetch('/api/mount').then(r => r.json()).then(m => showMount(m));
async function robotStatus() {
  try {
    const r = await (await fetch('/api/robot')).json();
    const busy = r.state !== 'idle';
    const st = $('robotstate');
    st.className = busy ? 'busy' : (r.running ? 'on' : 'off');
    st.textContent = (busy ? r.message : (r.running ? '● 실행 중' : '○ 실행 중 아님')) +
                     (!busy && r.message ? ' · ' + r.message : '');
    $('restart').disabled = busy;
    $('restart').textContent = r.running || busy ? 'robot_rgbd 재시작' : 'robot_rgbd 시작';
    // Strip the "[component-N] [INFO] [stamp]" prefixes for readability.
    $('robotlog').textContent = r.log.map(l => l.replace(/^\\[[^\\]]+\\]\\s*(\\[[A-Z]+\\]\\s*)?(\\[[0-9.]+\\]\\s*)?/, '')).join('\\n');
  } catch (e) { $('robotstate').textContent = '상태 확인 실패'; }
}
setInterval(robotStatus, 1000); robotStatus();

// ---- RoiRatios editor + overlay -------------------------------------------
let roiSaved = null;          // {roi:[l,r,t,b], grid:bool} as stored on the robot
let roiLoaded = false;        // inputs initialised from the saved values once
const roiNow = () => [0, 1, 2, 3].map(i => Math.min(Math.max(parseFloat($('roin' + i).value) || 0, 0), 0.5));
const same = (a, b) => a && b && a.every((v, i) => Math.abs(v - b[i]) < 1e-6);
function drawRoi() {
  const [l, r, t, b] = roiNow().map(v => v * 100);
  const inner = `M${l} ${t}H${100 - r}V${100 - b}H${l}Z`;
  const pending = !roiSaved || !same(roiNow(), roiSaved.roi) || $('roigrid').checked !== roiSaved.grid;
  const any = l || r || t || b;
  document.querySelectorAll('.roi').forEach(svg => {
    svg.querySelector('path').setAttribute('d', any ? `M0 0H100V100H0Z ${inner}` : '');
    const rect = svg.querySelector('rect');
    rect.setAttribute('x', l); rect.setAttribute('y', t);
    rect.setAttribute('width', 100 - l - r); rect.setAttribute('height', 100 - t - b);
    rect.style.display = any ? '' : 'none';
  });
  document.querySelectorAll('.roilabel').forEach(el => {
    el.hidden = !any;
    el.style.left = `calc(${l}% + 4px)`; el.style.top = `calc(${t}% + 4px)`;
    el.textContent = '특징점 사용 영역' + (pending ? ' (미적용)' : '');
  });
}
for (const i of [0, 1, 2, 3]) {
  $('roi' + i).addEventListener('input', e => { $('roin' + i).value = e.target.value; drawRoi(); });
  $('roin' + i).addEventListener('input', e => { $('roi' + i).value = e.target.value; drawRoi(); });
}
$('roigrid').addEventListener('change', drawRoi);
function setInputs(roi) { roi.forEach((v, i) => { $('roi' + i).value = v; $('roin' + i).value = v; }); }
function showRoi(s) {
  roiSaved = s.saved;
  if (!roiLoaded && s.saved) { setInputs(s.saved.roi); $('roigrid').checked = s.saved.grid; }
  roiLoaded = true;
  const st = $('roistate');
  const pending = !roiSaved || !same(roiNow(), roiSaved.roi) || $('roigrid').checked !== roiSaved.grid;
  const saved = roiSaved ? `저장됨 [${roiSaved.roi.join(', ')}]${roiSaved.grid ? ' +맵' : ''}` : '저장된 값 없음';
  const rt = s.rtabmap;
  st.className = pending ? 'pending' : (rt.message.startsWith('동기화 실패') ? 'err' : (rt.running ? 'ok' : ''));
  st.textContent = (pending ? '변경 사항 미적용 · ' : '') + saved + ' · ' + rt.message;
  drawRoi();
}
async function roiStatus() { try { showRoi(await (await fetch('/api/roi')).json()); } catch (e) {} }
setInterval(roiStatus, 2000); roiStatus();
$('roiapply').addEventListener('click', async () => {
  $('roiapply').disabled = true;
  try {
    const r = await fetch('/api/roi', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                        body: JSON.stringify({ roi: roiNow(), grid: $('roigrid').checked }) });
    const s = await r.json();
    if (!r.ok) { $('roistate').className = 'err'; $('roistate').textContent = s.error; return; }
    showRoi(s);
  } finally { $('roiapply').disabled = false; }
});
$('roizero').addEventListener('click', () => { setInputs([0, 0, 0, 0]); drawRoi(); });
$('restart').addEventListener('click', async () => {
  if (!confirm('robot_rgbd(차량 드라이버 + 카메라)를 재시작합니다. 차가 멈추고 영상이 잠시 끊깁니다. 계속할까요?')) return;
  $('restart').disabled = true;
  const r = await fetch('/api/robot/restart', { method: 'POST' });
  if (!r.ok) { const e = await r.json(); $('robotstate').className = 'off'; $('robotstate').textContent = e.error; }
  robotStatus();
});
$('save').addEventListener('click', async () => {
  $('save').disabled = true;
  try {
    const r = await fetch('/api/mount', { method: 'POST', headers: { 'Content-Type': 'application/json' },
                                          body: JSON.stringify({ include_height: $('withz').checked }) });
    const m = await r.json();
    if (!r.ok) { $('mountv').className = 'err'; $('mountv').textContent = m.error; return; }
    showMount(m, 'ok', '저장됨 (아래 robot_rgbd 재시작 버튼으로 적용): ');
  } catch (e) { $('mountv').className = 'err'; $('mountv').textContent = '저장 실패: ' + e; }
  finally { $('save').disabled = false; }
});
$('depth').addEventListener('click', e => {
  const r = e.target.getBoundingClientRect();
  pick = { x: (e.clientX - r.left) / r.width, y: (e.clientY - r.top) / r.height };
  const m = $('marker');
  m.style.left = (pick.x * 100) + '%'; m.style.top = (pick.y * 100) + '%'; m.hidden = false;
  refresh();
});
$('max').addEventListener('input', e => { $('maxv').textContent = e.target.value; });
$('max').addEventListener('change', e => { $('depth').src = '/stream/depth.mjpg?max=' + e.target.value; });
// Reconnect a stream if it stalls (robot launch restarted, WiFi hiccup).
for (const id of ['rgb', 'depth']) $(id).addEventListener('error', e => {
  setTimeout(() => { e.target.src = e.target.src.split('#')[0] + '#' + Date.now(); }, 1000);
});
</script></body></html>"""


class RoiSync:
    """Keeps the laptop's rtabmap RoiRatios equal to the ones saved here.

    The robot body is fixed in the camera image, so features on it look static
    and poison visual registration / loop closure. RoiRatios ("left right top
    bottom" fractions cut from the image edges) keep rtabmap from using them.
    The saved values live on the robot (~/.ros/car2_roi.yaml) and are pushed to
    the rtabmap node over ROS parameter services -- rtabmap applies them live
    ("Parameters event received!") -- and re-pushed whenever rtabmap
    (re)appears, so rtabmap_mapping.launch.py needs no copy of them.
    """

    FEATURE_PARAMS = ('Vis/RoiRatios', 'Kp/RoiRatios')
    GRID_PARAM = 'Grid/DepthRoiRatios'

    def __init__(self, node: Node, rtabmap_node: str, roi_file: str):
        self.node = node
        self.file = roi_file
        self.rtabmap_node = rtabmap_node
        self.set_cli = node.create_client(SetParameters, f'{rtabmap_node}/set_parameters')
        self.get_cli = node.create_client(GetParameters, f'{rtabmap_node}/get_parameters')
        self.lock = threading.Lock()
        self.desired = self._load()          # {'roi': [l, r, t, b], 'grid': bool} or None
        self.rtabmap = {'running': False, 'current': None, 'message': ''}
        threading.Thread(target=self._loop, daemon=True).start()

    @staticmethod
    def fmt(roi) -> str:
        return ' '.join(f'{v:g}' for v in roi)

    @staticmethod
    def parse(text: str):
        try:
            vals = [float(v) for v in text.split()]
        except (AttributeError, ValueError):
            return None
        return vals if len(vals) == 4 else None

    @staticmethod
    def validate(roi):
        if not (isinstance(roi, list) and len(roi) == 4):
            raise ValueError('roi는 [왼쪽, 오른쪽, 위, 아래] 4개 값이어야 합니다.')
        # Snap to multiples of 1/40: rtabmap / point_cloud_xyz silently IGNORE the
        # ROI unless the cropped image size divides by their decimation (e.g.
        # 0.22 of 480 rows leaves 374, not divisible by Grid/DepthDecimation 4).
        # 1/40 keeps 640x480 (decimation 4) and 320x240 (decimation 2) exact.
        roi = [round(float(v) * 40) / 40 for v in roi]
        if any(v < 0 or v >= 1 for v in roi):
            raise ValueError('각 비율은 0 이상 1 미만이어야 합니다.')
        if roi[0] + roi[1] >= 0.9 or roi[2] + roi[3] >= 0.9:
            raise ValueError('남는 영역이 너무 작습니다 (왼+오, 위+아래 합이 0.9 미만이어야 함).')
        return roi

    def _load(self):
        try:
            with open(self.file) as f:
                data = yaml.safe_load(f) or {}
            roi = self.validate(data['roi'])
            if roi != [float(v) for v in data['roi']]:
                self.node.get_logger().warning(
                    f'{self.file}: RoiRatios {data["roi"]} snapped to {roi} (must be multiples of 0.025 '
                    'or rtabmap ignores them)')
            return {'roi': roi, 'grid': bool(data.get('apply_to_grid', False))}
        except FileNotFoundError:
            return None
        except (KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.node.get_logger().warning(f'ignoring invalid {self.file}: {exc}')
            return None

    def wanted(self) -> dict:
        roi = self.fmt(self.desired['roi'])
        out = {name: roi for name in self.FEATURE_PARAMS}
        out[self.GRID_PARAM] = roi if self.desired['grid'] else '0 0 0 0'
        return out

    def _call(self, client, request, timeout=3.0):
        done = threading.Event()
        future = client.call_async(request)
        future.add_done_callback(lambda _: done.set())
        if not done.wait(timeout):
            raise TimeoutError(f'{self.rtabmap_node} did not answer')
        return future.result()

    def sync(self):
        """Read rtabmap's RoiRatios and push the saved ones if they differ."""
        with self.lock:
            if not self.set_cli.service_is_ready():
                self.rtabmap = {'running': False, 'current': None,
                                'message': 'rtabmap 미실행 — 시작되면 자동 적용'}
                return
            names = list(self.FEATURE_PARAMS) + [self.GRID_PARAM]
            try:
                res = self._call(self.get_cli, GetParameters.Request(names=names))
                current = {n: v.string_value for n, v in zip(names, res.values)}
                message = '적용됨'
                if self.desired is not None:
                    want = self.wanted()
                    stale = [n for n in names if self.parse(current.get(n)) != self.parse(want[n])]
                    if stale:
                        req = SetParameters.Request(parameters=[Parameter(
                            name=n, value=ParameterValue(type=ParameterType.PARAMETER_STRING,
                                                         string_value=want[n])) for n in stale])
                        results = self._call(self.set_cli, req).results
                        failed = [n for n, r in zip(stale, results) if not r.successful]
                        if failed:
                            raise RuntimeError(f'rtabmap rejected {failed}')
                        current.update({n: want[n] for n in stale})
                        message = f'적용됨 ({time.strftime("%H:%M:%S")} 전송)'
                        self.node.get_logger().info(f'pushed RoiRatios to rtabmap: { {n: want[n] for n in stale} }')
                else:
                    message = '저장된 값 없음 (rtabmap 기본값 사용 중)'
                self.rtabmap = {'running': True, 'current': current, 'message': message}
            except Exception as exc:  # noqa: BLE001 -- shown in the UI
                self.rtabmap = {'running': True, 'current': None, 'message': f'동기화 실패: {exc}'}

    def _loop(self):
        while rclpy.ok():
            self.sync()
            time.sleep(2.0)

    def save(self, roi, grid: bool) -> dict:
        roi = self.validate(roi)
        lines = [
            '# rtabmap RoiRatios for the robot camera (left right top bottom fractions cut',
            '# from the image edges; excludes the robot body). Written by camera_test.py;',
            '# pushed to the running rtabmap as Vis/RoiRatios + Kp/RoiRatios',
            '# (+ Grid/DepthRoiRatios when apply_to_grid is true).',
            f'# Saved {time.strftime("%Y-%m-%d %H:%M:%S")}',
            f'roi: [{", ".join(f"{v:g}" for v in roi)}]',
            f'apply_to_grid: {"true" if grid else "false"}',
        ]
        os.makedirs(os.path.dirname(self.file), exist_ok=True)
        tmp = self.file + '.tmp'
        with open(tmp, 'w') as f:
            f.write('\n'.join(lines) + '\n')
        os.replace(tmp, self.file)
        self.desired = {'roi': roi, 'grid': bool(grid)}
        self.sync()
        return self.status()

    def status(self) -> dict:
        saved_at = None
        if self.desired is not None and os.path.exists(self.file):
            saved_at = time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(os.path.getmtime(self.file)))
        return {'file': self.file, 'saved': self.desired, 'saved_at': saved_at, 'rtabmap': self.rtabmap}


class RobotLauncher:
    """Stops / (re)starts robot_rgbd.launch.py on this machine.

    Stopping finds the launch process however it was started (tmux, a plain
    terminal, ...) and SIGINTs it so the driver sends STOP and the camera is
    released; anything still holding the serial port / camera afterwards is
    killed. Starting runs scripts/run_robot_rgbd.sh in tmux session
    'robot_rgbd' (log: ~/robot_rgbd.log), with this process's ROS_DOMAIN_ID.
    """

    LAUNCH_PATTERN = 'ros2 launch car2_bringup robot_rgbd.launch.py'
    # Children that hold /dev/ttyS0 or the camera, in case the launch died uncleanly.
    CHILD_PATTERNS = ['car2_driver/car2_serial_node', 'openni2_camera_driver',
                      'component_container.*__ns:=/camera', 'usb_cam_node_exe.*__ns:=/camera']
    SESSION = 'robot_rgbd'

    def __init__(self, ws_dir: str, logger):
        self.ws_dir = ws_dir
        self.log_file = os.path.expanduser('~/robot_rgbd.log')
        self.logger = logger
        self.lock = threading.Lock()
        self.state = 'idle'          # idle | stopping | starting
        self.message = ''

    @staticmethod
    def _pids(pattern: str):
        out = subprocess.run(['pgrep', '-f', pattern], capture_output=True, text=True).stdout
        return [int(p) for p in out.split() if int(p) != os.getpid()]

    @staticmethod
    def _wait_gone(pids, timeout: float) -> bool:
        end = time.time() + timeout
        while time.time() < end:
            if not any(os.path.exists(f'/proc/{p}') for p in pids):
                return True
            time.sleep(0.3)
        return False

    def _signal(self, pids, sig):
        for pid in pids:
            try:
                os.kill(pid, sig)
            except ProcessLookupError:
                pass

    def status(self) -> dict:
        try:
            with open(self.log_file, errors='replace') as f:
                lines = f.read().splitlines()[-200:]
        except FileNotFoundError:
            lines = []
        # The lines worth showing: what the mount was loaded from, errors, readiness.
        keys = ('camera mount', 'placeholder camera mount', 'ERROR', 'died', 'ready on', 'Starting ', 'found.')
        # Known-harmless errors on this robot (Astra Pro color is UVC, not OpenNI2;
        # no depth calibration file) would otherwise crowd out the useful lines.
        harmless = ('Unsupported color video mode', 'camera_calibration_parsers')
        interesting = [ln for ln in lines
                       if any(k in ln for k in keys) and not any(h in ln for h in harmless)][-6:]
        return {'running': bool(self._pids(self.LAUNCH_PATTERN)), 'state': self.state,
                'message': self.message, 'log': interesting}

    def restart(self):
        if not self.lock.acquire(blocking=False):
            raise RuntimeError('이미 재시작 중입니다.')
        threading.Thread(target=self._restart, daemon=True).start()

    def _restart(self):
        try:
            self.state, self.message = 'stopping', '기존 robot_rgbd 종료 중…'
            launch = self._pids(self.LAUNCH_PATTERN)
            self._signal(launch, signal.SIGINT)
            if launch and not self._wait_gone(launch, 15):
                self._signal(launch, signal.SIGTERM)
                self._wait_gone(launch, 5)
            leftovers = [p for pat in self.CHILD_PATTERNS for p in self._pids(pat)]
            if leftovers:
                self._signal(leftovers, signal.SIGTERM)
                if not self._wait_gone(leftovers, 5):
                    self._signal(leftovers, signal.SIGKILL)
            if shutil.which('tmux'):
                subprocess.run(['tmux', 'kill-session', '-t', self.SESSION], capture_output=True)

            self.state, self.message = 'starting', 'robot_rgbd 시작 중…'
            # Export the ROS env inside the command: a tmux session runs with
            # the (possibly already running) tmux server's environment, not ours.
            ros_env = {'ROS_DISTRO': os.environ.get('ROS_DISTRO', 'jazzy')}
            for key in ('ROS_DOMAIN_ID', 'RMW_IMPLEMENTATION', 'ROS_AUTOMATIC_DISCOVERY_RANGE',
                        'CAR2_CAMERA_MOUNT_FILE'):
                if key in os.environ:
                    ros_env[key] = os.environ[key]
            exports = ' '.join(f'{k}={shlex.quote(v)}' for k, v in ros_env.items())
            cmd = (f'export {exports}; cd {shlex.quote(self.ws_dir)} && '
                   f'./scripts/run_robot_rgbd.sh 2>&1 | tee {shlex.quote(self.log_file)}')
            if shutil.which('tmux'):
                subprocess.run(['tmux', 'new-session', '-d', '-s', self.SESSION, cmd], check=True)
            else:
                subprocess.Popen(['bash', '-c', cmd], start_new_session=True,
                                 stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            # Ready once the launch process is up and the driver reports in.
            end = time.time() + 30
            while time.time() < end and not any('ready on' in ln for ln in self.status()['log']):
                time.sleep(1)
            running = bool(self._pids(self.LAUNCH_PATTERN))
            self.message = ('재시작 완료' if running else 'robot_rgbd가 바로 종료됨 — 로그 확인 필요') + \
                f' ({time.strftime("%H:%M:%S")})'
            self.logger.info(f'robot_rgbd restart: {self.message}')
        except Exception as exc:  # noqa: BLE001 -- surface anything to the UI
            self.message = f'재시작 실패: {exc}'
            self.logger.error(self.message)
        finally:
            self.state = 'idle'
            self.lock.release()


ROI_OVERLAY = ('<svg class="roi" viewBox="0 0 100 100" preserveAspectRatio="none">'
               '<path d=""/><rect/></svg><div class="roilabel" hidden></div>')


def make_handler(node: CameraViewer):

    class Handler(BaseHTTPRequestHandler):
        protocol_version = 'HTTP/1.0'

        def log_message(self, fmt, *args):  # keep the terminal quiet
            pass

        def _send(self, code, body: bytes, ctype: str):
            self.send_response(code)
            self.send_header('Content-Type', ctype)
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def _json(self, obj, code=200):
            self._send(code, json.dumps(obj).encode(), 'application/json')

        def _mjpeg(self, stream: Stream, render, key):
            self.send_response(200)
            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            seq, last_sent = -1, 0.0
            try:
                while rclpy.ok():
                    seq, frame = stream.wait_newer(seq)
                    if frame is None:
                        continue
                    wait = node.min_period - (time.time() - last_sent)
                    if wait > 0:
                        time.sleep(wait)
                        seq, frame = stream.wait_newer(-1, timeout=0)  # take the newest after sleeping
                    jpg = stream.jpeg(seq, key, lambda: node.encode(render(frame)))
                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n'
                                     + f'Content-Length: {len(jpg)}\r\n\r\n'.encode() + jpg + b'\r\n')
                    last_sent = time.time()
            except (BrokenPipeError, ConnectionResetError):
                pass  # browser closed the tab

        def do_GET(self):
            url = urlparse(self.path)
            q = parse_qs(url.query)
            if url.path == '/':
                page = PAGE.replace('__MAX__', str(node.depth_max_mm)).replace('__ROI_OVERLAY__', ROI_OVERLAY)
                self._send(200, page.encode(), 'text/html; charset=utf-8')
            elif url.path == '/stream/rgb.mjpg':
                self._mjpeg(node.rgb, lambda f: f, 'rgb')
            elif url.path == '/stream/depth.mjpg':
                max_mm = int(q.get('max', [node.depth_max_mm])[0])
                self._mjpeg(node.depth, lambda f: colorize_depth(f, max_mm), f'depth{max_mm}')
            elif url.path == '/api/stats':
                self._json(node.stats())
            elif url.path == '/api/mount':
                self._json(node.saved_mount())
            elif url.path == '/api/robot':
                self._json(node.robot.status())
            elif url.path == '/api/roi':
                self._json(node.roi.status())
            elif url.path == '/api/drive':
                self._json(node.drive_status())
            elif url.path == '/api/depth_at':
                try:
                    x, y = float(q['x'][0]), float(q['y'][0])
                except (KeyError, ValueError):
                    self._json({'error': 'x, y (0..1) required'}, 400)
                    return
                self._json(node.depth_at(x, y) or {'mm': None})
            else:
                self._json({'error': 'not found'}, 404)

        def do_POST(self):
            path = urlparse(self.path).path
            if path == '/api/drive/stop':
                node.stop_drive()
                self._json(node.drive_status())
                return
            if path == '/api/drive':
                try:
                    length = int(self.headers.get('Content-Length') or 0)
                    body = json.loads(self.rfile.read(length) or b'{}')
                    node.drive(body.get('linear', 0.0), body.get('angular', 0.0))
                except (ValueError, TypeError) as exc:
                    node.stop_drive()
                    self._json({'error': f'잘못된 주행 명령: {exc}'}, 400)
                    return
                self._json(node.drive_status())
                return
            if path == '/api/roi':
                try:
                    length = int(self.headers.get('Content-Length') or 0)
                    body = json.loads(self.rfile.read(length) or b'{}')
                    self._json(node.roi.save(body.get('roi'), bool(body.get('grid', False))))
                except (ValueError, TypeError) as exc:   # also json.JSONDecodeError
                    self._json({'error': str(exc)}, 400)
                except OSError as exc:
                    self._json({'error': f'파일 저장 실패: {exc}'}, 500)
                return
            if path == '/api/robot/restart':
                try:
                    node.robot.restart()
                except RuntimeError as exc:
                    self._json({'error': str(exc)}, 409)
                    return
                self._json(node.robot.status())
                return
            if path != '/api/mount':
                self._json({'error': 'not found'}, 404)
                return
            try:
                length = int(self.headers.get('Content-Length') or 0)
                body = json.loads(self.rfile.read(length) or b'{}')
            except ValueError:   # also json.JSONDecodeError
                self._json({'error': '요청 형식 오류: JSON body가 필요합니다 (예: {"include_height": false})'}, 400)
                return
            try:
                self._json(node.save_mount(bool(body.get('include_height', False))))
            except ValueError as exc:   # no fresh measurement / floor not visible
                self._json({'error': str(exc)}, 409)
            except OSError as exc:
                self._json({'error': f'파일 저장 실패: {exc}'}, 500)

    return Handler


def main():
    rclpy.init()
    node = CameraViewer()
    server = ThreadingHTTPServer(('0.0.0.0', node.port), make_handler(node))
    server.daemon_threads = True
    threading.Thread(target=server.serve_forever, daemon=True).start()
    node.get_logger().info(f'web UI: http://<this-host>:{node.port}/')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
