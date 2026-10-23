#!/usr/bin/env python3
"""ROS 2 driver for the car2 4-wheel skid-steer base (ESP32 + ST3215 serial protocol).

Protocol reference: car2/car2/car2.ino, car2/car2/Car.cpp (this repo).
  - Drive command : "D,<left_steps_per_s>,<right_steps_per_s>\n" -> "ACK,D,..."
  - Status query   : "STAT\n" -> 4x "STAT,<FL|FR|RL|RR>,<id>,pos=..,spd=..,..." + "ACK,STAT"
  - Speed unit is step/s, range [-3400, 3400], 4096 steps = 1 wheel revolution.
  - Firmware auto-stops if no command arrives for 500ms, so drive commands must
    be resent continuously (this node resends every control tick regardless of
    whether cmd_vel changed).

wheel_separation_m is the *effective* skid-steer track (0.53m), not the
measured 윤거 (0.43m; wheelbase/축거 0.20m): the wheels scrub sideways when
turning, so the body rotates less than (v_r - v_l) / 0.43 predicts. 0.43
over-reported yaw by ~19-20% (RTAB-Map's visual registration; a 360 deg spin
matched against the lidar: actual/odom = 0.833 with r=0.032). wheel_radius_m 0.0329
comes from a 0.87 m straight run measured by the lidar (actual/odom = 1.027); the
track was scaled with it (0.0329 / (0.833 * 0.032 / 0.43) = 0.53) so the rotation
calibration still holds.
"""
import math
import re
import threading
from typing import Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
import serial

# Must match INVERT_DEFAULT in car2/car2/Car.cpp -- STAT reports raw servo
# speed (pre-inversion), so we invert back here to get vehicle-forward-positive.
INVERT = {'FL': True, 'FR': False, 'RL': True, 'RR': False}

STAT_RE = re.compile(
    r'STAT,(?P<name>FL|FR|RL|RR),(?P<id>\d+),'
    r'pos=(?P<pos>-?\d+),spd=(?P<spd>-?\d+),load=(?P<load>-?\d+),'
    r'V=(?P<v>[\d.]+),T=(?P<t>-?\d+),cur=(?P<cur>-?\d+)'
)


class Car2SerialNode(Node):

    def __init__(self):
        super().__init__('car2_serial_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('wheel_radius_m', 0.0329)  # lidar-verified straight run (was 0.032)
        self.declare_parameter('wheel_separation_m', 0.53)  # effective track (measured 윤거 0.43)
        self.declare_parameter('steps_per_rev', 4096)
        self.declare_parameter('max_step_speed', 3400)
        self.declare_parameter('control_period_s', 0.05)
        self.declare_parameter('odom_poll_every_n', 4)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('odom_frame_id', 'odom')
        self.declare_parameter('base_frame_id', 'base_link')
        self.declare_parameter('accel', 20)
        # Stop if no /cmd_vel arrives for this long (0 = keep the last command
        # forever, the old behavior). Protects against a publisher that dies
        # mid-move (Nav2 Ctrl-C/crash, WiFi drop) -- the firmware's own 500ms
        # timeout doesn't help because this node resends every tick.
        self.declare_parameter('cmd_vel_timeout_s', 1.0)

        self.port = self.get_parameter('serial_port').value
        self.baud = self.get_parameter('baud_rate').value
        self.wheel_radius_m = self.get_parameter('wheel_radius_m').value
        self.wheel_separation_m = self.get_parameter('wheel_separation_m').value
        self.steps_per_rev = self.get_parameter('steps_per_rev').value
        self.max_step_speed = self.get_parameter('max_step_speed').value
        self.control_period_s = self.get_parameter('control_period_s').value
        self.odom_poll_every_n = max(1, self.get_parameter('odom_poll_every_n').value)
        self.publish_tf = self.get_parameter('publish_tf').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        self.accel = self.get_parameter('accel').value
        self.cmd_vel_timeout_s = float(self.get_parameter('cmd_vel_timeout_s').value)

        self._lock = threading.Lock()
        self._target_v = 0.0
        self._target_w = 0.0
        self._last_cmd_time = None
        self._timed_out = False
        self._tick = 0

        self._x = 0.0
        self._y = 0.0
        self._theta = 0.0
        self._last_odom_time = self.get_clock().now()

        self._ser: Optional[serial.Serial] = None
        self._open_serial()

        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self._cmd_vel_cb, 10)

        self.create_timer(self.control_period_s, self._control_tick)
        self.get_logger().info(
            f'car2_serial_node ready on {self.port} @ {self.baud} baud '
            f'(wheel_radius_m={self.wheel_radius_m}, '
            f'wheel_separation_m={self.wheel_separation_m} -- '
            f'verify these against the real robot)'
        )

    def _open_serial(self):
        self._ser = serial.Serial(self.port, self.baud, timeout=0.2)
        # car2.ino runs on an ESP32; opening the port resets the board, which
        # then prints "SERVOS,x/4" / "READY" boot lines before it accepts commands.
        import time
        time.sleep(2.0)
        self._ser.reset_input_buffer()
        try:
            self._send_command(f'ACC,{self.accel}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'Initial ACC command failed: {exc}')

    def _send_command(self, cmd: str) -> list:
        """Write one command and read back lines up to and including ACK/ERR."""
        self._ser.write((cmd + '\n').encode('ascii'))
        lines = []
        for _ in range(8):
            raw = self._ser.readline()
            if not raw:
                break
            line = raw.decode('ascii', errors='replace').strip()
            if not line:
                continue
            lines.append(line)
            if line.startswith('ACK,') or line.startswith('ERR,'):
                break
        return lines

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self._target_v = msg.linear.x
            self._target_w = msg.angular.z
            self._last_cmd_time = self.get_clock().now()
            self._timed_out = False

    def _wheel_speeds_steps(self, v: float, w: float):
        v_l = v - w * self.wheel_separation_m / 2.0
        v_r = v + w * self.wheel_separation_m / 2.0
        to_steps = self.steps_per_rev / (2.0 * math.pi * self.wheel_radius_m)
        left_steps = int(round(v_l * to_steps))
        right_steps = int(round(v_r * to_steps))
        left_steps = max(-self.max_step_speed, min(self.max_step_speed, left_steps))
        right_steps = max(-self.max_step_speed, min(self.max_step_speed, right_steps))
        return left_steps, right_steps

    def _control_tick(self):
        with self._lock:
            if (self.cmd_vel_timeout_s > 0 and self._last_cmd_time is not None and not self._timed_out
                    and (self.get_clock().now() - self._last_cmd_time).nanoseconds * 1e-9 > self.cmd_vel_timeout_s
                    and (self._target_v or self._target_w)):
                self._target_v = self._target_w = 0.0
                self._timed_out = True
                self.get_logger().warning(
                    f'no /cmd_vel for {self.cmd_vel_timeout_s:.1f}s -- stopping the car')
            v, w = self._target_v, self._target_w
        left_steps, right_steps = self._wheel_speeds_steps(v, w)

        try:
            self._send_command(f'D,{left_steps},{right_steps}')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial write failed, reconnecting: {exc}')
            self._reconnect()
            return

        self._tick += 1
        if self._tick % self.odom_poll_every_n == 0:
            self._poll_odom()

    def _poll_odom(self):
        try:
            lines = self._send_command('STAT')
        except (serial.SerialException, OSError) as exc:
            self.get_logger().error(f'Serial STAT failed, reconnecting: {exc}')
            self._reconnect()
            return

        speeds = {}
        for line in lines:
            m = STAT_RE.match(line)
            if not m:
                continue
            name = m.group('name')
            raw_spd = int(m.group('spd'))
            speeds[name] = -raw_spd if INVERT[name] else raw_spd

        if not all(k in speeds for k in ('FL', 'FR', 'RL', 'RR')):
            return  # a wheel didn't answer this cycle; skip rather than guess

        steps_to_wheel_lin = (2.0 * math.pi * self.wheel_radius_m) / self.steps_per_rev
        v_left = (speeds['FL'] + speeds['RL']) / 2.0 * steps_to_wheel_lin
        v_right = (speeds['FR'] + speeds['RR']) / 2.0 * steps_to_wheel_lin
        v = (v_left + v_right) / 2.0
        w = (v_right - v_left) / self.wheel_separation_m

        now = self.get_clock().now()
        dt = (now - self._last_odom_time).nanoseconds * 1e-9
        self._last_odom_time = now
        if dt <= 0.0 or dt > 1.0:
            return  # first sample or a stall; don't integrate a bogus dt

        self._x += v * math.cos(self._theta) * dt
        self._y += v * math.sin(self._theta) * dt
        self._theta += w * dt

        self._publish_odom(now, v, w)

    def _publish_odom(self, stamp, v: float, w: float):
        qz = math.sin(self._theta / 2.0)
        qw = math.cos(self._theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp.to_msg()
        odom.header.frame_id = self.odom_frame_id
        odom.child_frame_id = self.base_frame_id
        odom.pose.pose.position.x = self._x
        odom.pose.pose.position.y = self._y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = stamp.to_msg()
            t.header.frame_id = self.odom_frame_id
            t.child_frame_id = self.base_frame_id
            t.transform.translation.x = self._x
            t.transform.translation.y = self._y
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def _reconnect(self):
        try:
            if self._ser is not None:
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass
        try:
            self._open_serial()
            self.get_logger().info('Serial reconnected.')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'Reconnect failed: {exc}')

    def destroy_node(self):
        try:
            if self._ser is not None and self._ser.is_open:
                self._send_command('STOP')
                self._ser.close()
        except Exception:  # noqa: BLE001
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Car2SerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
