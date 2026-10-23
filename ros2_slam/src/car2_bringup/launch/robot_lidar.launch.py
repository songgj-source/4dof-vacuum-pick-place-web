"""Robot-side (Raspberry Pi) bringup: car2 4-wheel driver + LD-series 2D lidar.

  car2_serial_node  /cmd_vel -> wheels (/dev/ttyS0), /odom + TF odom->base_link
  ld_lidar_node     LD06/LD19-protocol lidar (CP2102 USB serial) -> /scan (laser_frame)
  static TF         base_link -> laser_frame (lidar mount pose)
  camera (use_camera:=true)  Astra Pro depth + RGB from robot_rgbd.launch.py
                    (its car2 driver left out -- this launch has one)

Usage (on the robot):
  ros2 launch car2_bringup robot_lidar.launch.py
  ros2 launch car2_bringup robot_lidar.launch.py lidar_x:=0.05 lidar_z:=0.25 lidar_yaw:=3.1416
  ros2 launch car2_bringup robot_lidar.launch.py use_car2_driver:=false   # lidar only
  ros2 launch car2_bringup robot_lidar.launch.py use_camera:=true          # + depth/RGB camera

Lidar mount (2026-10-02): centered between the wheels (lidar_x = lidar_y = 0),
its 0-degree direction ~4.5 deg left of the robot's front (lidar_yaw = 0.0785,
from two straight runs); scan angle direction verified CCW by a 360 deg spin. lidar_z (0.2) is still a
placeholder -- it doesn't matter for 2D mapping / navigation.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

LIDAR_PORT = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0'


def generate_launch_description():
    cfg = {name: LaunchConfiguration(name) for name in (
        'use_car2_driver', 'serial_port', 'wheel_radius_m', 'wheel_separation_m', 'odom_poll_every_n',
        'lidar_port', 'lidar_frame', 'lidar_x', 'lidar_y', 'lidar_z', 'lidar_yaw', 'lidar_min_intensity')}

    return LaunchDescription([
        DeclareLaunchArgument('use_car2_driver', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS0'),
        # Wheel radius from a 0.87 m straight run vs the lidar (ICP 0.892 m, front wall
        # 0.887 m): actual/odom = 1.027 -> 0.032 * 1.027 = 0.0329 m (2026-10-02).
        DeclareLaunchArgument('wheel_radius_m', default_value='0.0329'),
        # Effective skid-steer track (measured 윤거 0.43 m; wheels scrub when turning):
        # a 360 deg spin rotated the lidar scans 0.833x the odom yaw (with r=0.032).
        # ... and the track scaled with it (rotation calibration needs radius/track
        # = 0.833 * 0.032/0.43): 0.0329 / 0.06199 = 0.53.
        DeclareLaunchArgument('wheel_separation_m', default_value='0.53'),
        # 10 Hz odom (STAT every 2nd 0.05 s tick) -- SLAM/Nav2 interpolate it per scan.
        DeclareLaunchArgument('odom_poll_every_n', default_value='2'),
        DeclareLaunchArgument('lidar_port', default_value=LIDAR_PORT),
        DeclareLaunchArgument('lidar_frame', default_value='laser_frame'),
        # Centered between the wheels, 0 deg = robot front (measured); z placeholder.
        DeclareLaunchArgument('lidar_x', default_value='0.0'),
        DeclareLaunchArgument('lidar_y', default_value='0.0'),
        DeclareLaunchArgument('lidar_z', default_value='0.2'),
        # The lidar's 0 deg points ~4.5 deg left of the robot's front: two 0.87 m
        # straight runs moved -3.9 / -5.2 deg in laser_frame with no heading change
        # (lidar ICP vs odom, 2026-10-02) -> base_link->laser_frame yaw +4.5 deg.
        DeclareLaunchArgument('lidar_yaw', default_value='0.0785'),
        DeclareLaunchArgument('lidar_min_intensity', default_value='0'),
        DeclareLaunchArgument('use_camera', default_value='false',
                              description='also start the Astra Pro depth + RGB camera (robot_rgbd.launch.py, no driver)'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Astra Pro RGB UVC device, with use_camera:=true'),

        Node(
            package='car2_driver',
            executable='car2_serial_node',
            name='car2_serial_node',
            output='screen',
            parameters=[{
                'serial_port': cfg['serial_port'],
                'wheel_radius_m': cfg['wheel_radius_m'],
                'wheel_separation_m': cfg['wheel_separation_m'],
                'odom_poll_every_n': cfg['odom_poll_every_n'],
            }],
            condition=IfCondition(cfg['use_car2_driver']),
        ),

        Node(
            package='car2_driver',
            executable='ld_lidar_node',
            name='ld_lidar',
            output='screen',
            # Respawn: a USB hiccup (the hub has dropped devices before) must not
            # leave the robot blind until someone restarts the launch.
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'port': cfg['lidar_port'],
                'frame_id': cfg['lidar_frame'],
                'min_intensity': cfg['lidar_min_intensity'],
                # Nothing real can be closer than this: it is inside the robot
                # footprint (radius 0.26 m). Drops the robot's own posts (~0.19-0.21 m)
                # whatever their angle -- leaked post points made collision_monitor
                # see a collision "now" and hold Nav2's /cmd_vel at zero.
                'range_min': 0.25,
                # The robot's 4 frame posts, ~0.2 m from the lidar on both sides
                # (measured 2026-10-02; re-checked: they reach -99.5 / +98.5 deg and
                # show between the two posts of a side, so each side is one sector
                # with a margin). Only returns closer than self_mask_range there are
                # dropped. Angles are in laser_frame, so they stay valid if the TF
                # lidar_yaw changes; re-measure if the lidar itself is remounted/rotated.
                'self_mask_deg': [-103.0, -66.0, 68.0, 102.0],
                'self_mask_range': 0.35,
            }],
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser_tf',
            output='screen',
            arguments=[
                '--x', cfg['lidar_x'], '--y', cfg['lidar_y'], '--z', cfg['lidar_z'],
                '--yaw', cfg['lidar_yaw'], '--pitch', '0', '--roll', '0',
                '--frame-id', 'base_link', '--child-frame-id', cfg['lidar_frame'],
            ],
        ),

        # Camera: robot_rgbd.launch.py without its car2 driver. Scoped group so its
        # launch arguments (use_car2_driver:=false, its own wheel/serial defaults)
        # don't leak into this launch's configuration.
        GroupAction(
            scoped=True,
            condition=IfCondition(LaunchConfiguration('use_camera')),
            actions=[IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('car2_bringup'), 'launch', 'robot_rgbd.launch.py')),
                launch_arguments={'use_car2_driver': 'false',
                                  'video_device': LaunchConfiguration('video_device')}.items(),
            )],
        ),
    ])
