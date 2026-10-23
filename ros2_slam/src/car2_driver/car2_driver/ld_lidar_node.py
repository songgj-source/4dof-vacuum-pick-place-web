#!/usr/bin/env python3
"""ROS 2 driver for LDROBOT LD-series 2D lidars (LD06 / LD19 / STL-19P protocol).

The sensor streams on its own (no commands) at 230400 baud: 47-byte packets
  0x54 0x2C | speed u16 (deg/s) | start_angle u16 (0.01 deg) |
  12 x [distance u16 (mm), intensity u8] | end_angle u16 | timestamp u16 (ms) | crc8
The angles grow CLOCKWISE seen from above; ROS angles are counter-clockwise,
so they are mirrored (clockwise:=true). Points of one revolution are binned
into a fixed-resolution sensor_msgs/LaserScan on /scan (frame laser_frame).

Measured on car2's unit (2026-10-02): ~4000 points/s, 6 Hz rotation
(~670 points/rev, ~0.54 deg), ranges 0.19 .. 8 m in a room.

Parameters:
  port               serial device (default: the CP2102 by-id path, stable across reboots)
  baud_rate          230400
  frame_id           laser_frame
  angle_resolution_deg 0.5  LaserScan bin size
  range_min / range_max   0.05 / 12.0 m
  min_intensity      points below this confidence are dropped (0 = keep all)
  angle_offset_deg   added after mirroring: use it if the lidar's 0 deg mark
                     doesn't point to the robot's front (or set the TF yaw instead)
  clockwise          true for LD06/LD19 (mirror angles to ROS CCW)
  self_mask_deg      [min1, max1, min2, max2, ...] angle sectors (deg, in the published
                     laser_frame angles) where the robot's own parts sit; returns closer
                     than self_mask_range there are dropped, farther ones (seen past the
                     part's edge) are kept
  self_mask_range    0.35 m
"""
import math
import struct
import threading
import time

import rclpy
import serial
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

PACKET_LEN = 47
POINTS_PER_PACKET = 12

CRC_TABLE = [
    0x00, 0x4d, 0x9a, 0xd7, 0x79, 0x34, 0xe3, 0xae, 0xf2, 0xbf, 0x68, 0x25, 0x8b, 0xc6, 0x11, 0x5c,
    0xa9, 0xe4, 0x33, 0x7e, 0xd0, 0x9d, 0x4a, 0x07, 0x5b, 0x16, 0xc1, 0x8c, 0x22, 0x6f, 0xb8, 0xf5,
    0x1f, 0x52, 0x85, 0xc8, 0x66, 0x2b, 0xfc, 0xb1, 0xed, 0xa0, 0x77, 0x3a, 0x94, 0xd9, 0x0e, 0x43,
    0xb6, 0xfb, 0x2c, 0x61, 0xcf, 0x82, 0x55, 0x18, 0x44, 0x09, 0xde, 0x93, 0x3d, 0x70, 0xa7, 0xea,
    0x3e, 0x73, 0xa4, 0xe9, 0x47, 0x0a, 0xdd, 0x90, 0xcc, 0x81, 0x56, 0x1b, 0xb5, 0xf8, 0x2f, 0x62,
    0x97, 0xda, 0x0d, 0x40, 0xee, 0xa3, 0x74, 0x39, 0x65, 0x28, 0xff, 0xb2, 0x1c, 0x51, 0x86, 0xcb,
    0x21, 0x6c, 0xbb, 0xf6, 0x58, 0x15, 0xc2, 0x8f, 0xd3, 0x9e, 0x49, 0x04, 0xaa, 0xe7, 0x30, 0x7d,
    0x88, 0xc5, 0x12, 0x5f, 0xf1, 0xbc, 0x6b, 0x26, 0x7a, 0x37, 0xe0, 0xad, 0x03, 0x4e, 0x99, 0xd4,
    0x7c, 0x31, 0xe6, 0xab, 0x05, 0x48, 0x9f, 0xd2, 0x8e, 0xc3, 0x14, 0x59, 0xf7, 0xba, 0x6d, 0x20,
    0xd5, 0x98, 0x4f, 0x02, 0xac, 0xe1, 0x36, 0x7b, 0x27, 0x6a, 0xbd, 0xf0, 0x5e, 0x13, 0xc4, 0x89,
    0x63, 0x2e, 0xf9, 0xb4, 0x1a, 0x57, 0x80, 0xcd, 0x91, 0xdc, 0x0b, 0x46, 0xe8, 0xa5, 0x72, 0x3f,
    0xca, 0x87, 0x50, 0x1d, 0xb3, 0xfe, 0x29, 0x64, 0x38, 0x75, 0xa2, 0xef, 0x41, 0x0c, 0xdb, 0x96,
    0x42, 0x0f, 0xd8, 0x95, 0x3b, 0x76, 0xa1, 0xec, 0xb0, 0xfd, 0x2a, 0x67, 0xc9, 0x84, 0x53, 0x1e,
    0xeb, 0xa6, 0x71, 0x3c, 0x92, 0xdf, 0x08, 0x45, 0x19, 0x54, 0x83, 0xce, 0x60, 0x2d, 0xfa, 0xb7,
    0x5d, 0x10, 0xc7, 0x8a, 0x24, 0x69, 0xbe, 0xf3, 0xaf, 0xe2, 0x35, 0x78, 0xd6, 0x9b, 0x4c, 0x01,
    0xf4, 0xb9, 0x6e, 0x23, 0x8d, 0xc0, 0x17, 0x5a, 0x06, 0x4b, 0x9c, 0xd1, 0x7f, 0x32, 0xe5, 0xa8,
]


def crc8(data: bytes) -> int:
    crc = 0
    for b in data:
        crc = CRC_TABLE[(crc ^ b) & 0xff]
    return crc


class LdLidarNode(Node):

    def __init__(self):
        super().__init__('ld_lidar')
        self.declare_parameter(
            'port', '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0')
        self.declare_parameter('baud_rate', 230400)
        self.declare_parameter('frame_id', 'laser_frame')
        self.declare_parameter('angle_resolution_deg', 0.5)
        self.declare_parameter('range_min', 0.05)
        self.declare_parameter('range_max', 12.0)
        self.declare_parameter('min_intensity', 0)
        self.declare_parameter('angle_offset_deg', 0.0)
        self.declare_parameter('clockwise', True)
        self.declare_parameter('self_mask_deg', [0.0])        # [0.0] = no mask (ROS can't declare an empty list)
        self.declare_parameter('self_mask_range', 0.35)

        self.port = self.get_parameter('port').value
        self.baud = int(self.get_parameter('baud_rate').value)
        self.frame_id = self.get_parameter('frame_id').value
        self.res = math.radians(float(self.get_parameter('angle_resolution_deg').value))
        self.range_min = float(self.get_parameter('range_min').value)
        self.range_max = float(self.get_parameter('range_max').value)
        self.min_intensity = int(self.get_parameter('min_intensity').value)
        self.offset = math.radians(float(self.get_parameter('angle_offset_deg').value))
        self.clockwise = bool(self.get_parameter('clockwise').value)
        self.bins = int(round(2 * math.pi / self.res))
        flat = [float(v) for v in self.get_parameter('self_mask_deg').value]
        if len(flat) % 2:
            flat = flat[:-1]
        self.self_mask = [(math.radians(a), math.radians(b)) for a, b in zip(flat[0::2], flat[1::2]) if b > a]
        self.self_mask_range = float(self.get_parameter('self_mask_range').value)
        self._masked = 0

        self.pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self._ser = None
        self._running = True
        self._stats = {'packets': 0, 'crc_errors': 0, 'scans': 0}
        self._first_scan_logged = False
        threading.Thread(target=self._read_loop, daemon=True).start()
        self.create_timer(10.0, self._report)
        self.get_logger().info(f'ld_lidar reading {self.port} @ {self.baud} -> /scan ({self.frame_id}, '
                               f'{math.degrees(self.res):.2f} deg bins)')
        if self.self_mask:
            self.get_logger().info('self mask (deg, < %.2f m dropped): %s' % (
                self.self_mask_range, ', '.join(f'{math.degrees(a):+.1f}..{math.degrees(b):+.1f}' for a, b in self.self_mask)))

    # ---------------------------------------------------------------- serial
    def _open(self):
        while self._running and rclpy.ok():
            try:
                return serial.Serial(self.port, self.baud, timeout=0.5)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'cannot open {self.port}: {exc} (retrying)', throttle_duration_sec=10.0)
                time.sleep(2.0)
        return None

    def _read_loop(self):
        buf = bytearray()
        rev = []                 # (ros_angle_rad, range_m, intensity) of the current revolution
        rev_start = None         # receive time of the revolution's first packet
        last_start = None
        speed = 0.0
        partial = True
        while self._running and rclpy.ok():
            if self._ser is None:
                self._ser = self._open()
                if self._ser is None:
                    return
                buf.clear()
                rev, rev_start, last_start, partial = [], None, None, True
            try:
                chunk = self._ser.read(PACKET_LEN * 8)
            except (serial.SerialException, OSError) as exc:
                self.get_logger().error(f'serial read failed: {exc} (reopening)')
                try:
                    self._ser.close()
                except Exception:  # noqa: BLE001
                    pass
                self._ser = None
                continue
            if not chunk:
                self.get_logger().warning('no data from the lidar (powered? spinning?)', throttle_duration_sec=5.0)
                continue
            buf += chunk
            now = time.time()
            while len(buf) >= PACKET_LEN:
                if buf[0] != 0x54 or buf[1] != 0x2C:
                    del buf[0]
                    continue
                pkt = bytes(buf[:PACKET_LEN])
                if crc8(pkt[:46]) != pkt[46]:
                    self._stats['crc_errors'] += 1
                    del buf[0]
                    continue
                del buf[:PACKET_LEN]
                self._stats['packets'] += 1
                spd, start = struct.unpack_from('<HH', pkt, 2)
                end = struct.unpack_from('<H', pkt, 42)[0]
                speed = spd
                start_deg, end_deg = start / 100.0, end / 100.0
                # A new revolution starts when the start angle wraps around.
                if last_start is not None and start_deg < last_start and rev:
                    if partial:
                        partial = False       # the first revolution started mid-turn: drop it
                    else:
                        self._publish(rev, rev_start, now, speed)
                    rev, rev_start = [], None
                last_start = start_deg
                if rev_start is None:
                    rev_start = now
                span = (end_deg - start_deg) % 360.0
                step = span / (POINTS_PER_PACKET - 1)
                for k in range(POINTS_PER_PACKET):
                    dist, inten = struct.unpack_from('<HB', pkt, 6 + 3 * k)
                    if dist == 0 or inten < self.min_intensity:
                        continue
                    a = math.radians((start_deg + step * k) % 360.0)
                    if self.clockwise:
                        a = -a
                    a = (a + self.offset + math.pi) % (2 * math.pi) - math.pi   # [-pi, pi)
                    rev.append((a, dist / 1000.0, inten))

    # ---------------------------------------------------------------- scan
    def _publish(self, points, t_first, t_last, speed_dps):
        ranges = [float('inf')] * self.bins
        intens = [0.0] * self.bins
        for a, r, i in points:
            if not (self.range_min <= r <= self.range_max):
                continue
            if r < self.self_mask_range and any(lo <= a <= hi for lo, hi in self.self_mask):
                self._masked += 1         # the robot's own post/frame
                continue
            b = int((a + math.pi) / self.res) % self.bins
            if r < ranges[b]:                     # keep the nearest return per bin
                ranges[b], intens[b] = r, float(i)
        scan_time = 360.0 / speed_dps if speed_dps > 0 else (t_last - t_first)
        msg = LaserScan()
        # Stamp at the revolution's start (as LaserScan expects; points are
        # time_increment apart). Serial latency at 230400 baud is ~2 ms.
        msg.header.stamp = rclpy.time.Time(seconds=t_last - scan_time).to_msg()
        msg.header.frame_id = self.frame_id
        msg.angle_min = -math.pi
        msg.angle_max = -math.pi + self.res * (self.bins - 1)
        msg.angle_increment = self.res
        msg.scan_time = scan_time
        msg.time_increment = scan_time / self.bins
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = ranges
        msg.intensities = intens
        self.pub.publish(msg)
        self._stats['scans'] += 1
        if not self._first_scan_logged:
            self._first_scan_logged = True
            valid = sum(1 for r in ranges if r != float('inf'))
            self.get_logger().info(f'first scan published: {len(points)} points, {valid}/{self.bins} bins, '
                                   f'{1.0 / scan_time:.1f} Hz rotation' if scan_time else 'first scan published')

    def _report(self):
        s = self._stats
        self.get_logger().info(f"scans {s['scans'] / 10.0:.1f} Hz, packets {s['packets'] / 10.0:.0f}/s, "
                               f"crc errors {s['crc_errors']}, self-masked {self._masked / 10.0:.0f} pts/s")
        self._masked = 0
        self._stats = {'packets': 0, 'crc_errors': 0, 'scans': 0}

    def destroy_node(self):
        self._running = False
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LdLidarNode()
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
