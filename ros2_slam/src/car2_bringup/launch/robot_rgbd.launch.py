"""Robot-side (Raspberry Pi) bringup for remote RTAB-Map mapping: car2 serial
driver + Orbbec Astra Pro depth (OpenNI2) + Astra Pro RGB (UVC, usb_cam) +
the camera static transforms. No SLAM runs here -- the laptop subscribes to
the compressed image streams over the network (rtabmap_mapping.launch.py).

Usage (on the robot):
  ros2 launch car2_bringup robot_rgbd.launch.py
  ros2 launch car2_bringup robot_rgbd.launch.py video_device:=/dev/video1 use_car2_driver:=false

Published for the laptop (image_transport adds /compressed and
/compressedDepth automatically when image_transport_plugins is installed):
  /camera/color/image_raw[/compressed] + /camera/color/camera_info
  /camera/depth_raw/image[/compressedDepth] + /camera/depth_raw/camera_info
  /odom + TF odom->base_link, static TF base_link->camera_link->optical frames

The Astra Pro's RGB sensor is a separate UVC device that OpenNI2 can't open
("Unsupported color video mode"), so RGB comes from usb_cam and depth stays
unregistered here; the laptop registers depth into the RGB frame
(depth_image_proc/register) before RTAB-Map. use_device_time is False so both
streams (and wheel odometry) are stamped with the Pi's system clock and can be
synchronized.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Camera mount angles measured on the robot (written by the "현재 값으로 설정"
# button in test/camera_test.py's web UI). Lives outside the workspace so
# deploy_to_robot.sh never overwrites it. Values become the defaults of the
# base_to_camera_* launch arguments; explicit launch arguments still win.
CAMERA_MOUNT_FILE = os.environ.get(
    'CAR2_CAMERA_MOUNT_FILE', os.path.expanduser('~/.ros/car2_camera_mount.yaml'))


def load_camera_mount():
    try:
        with open(CAMERA_MOUNT_FILE) as f:
            data = yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}
    return {k: float(v) for k, v in data.items() if k.startswith('base_to_camera_')}


def generate_launch_description():
    mount = load_camera_mount()

    def mount_default(name, fallback):
        return str(mount.get(name, fallback))

    bringup_share = get_package_share_directory('car2_bringup')
    openni2_share = get_package_share_directory('openni2_camera')
    # bringup_share = <ws>/install/car2_bringup/share/car2_bringup
    default_openni2_override = os.path.abspath(os.path.join(
        bringup_share, '..', '..', '..', '..', 'third_party', 'orbbec_openni2'))
    default_rgb_info = 'file://' + os.path.join(bringup_share, 'config', 'astra_pro_rgb.yaml')
    rgb_transport_params = os.path.join(bringup_share, 'config', 'robot_rgb_transport.yaml')

    use_car2_driver = LaunchConfiguration('use_car2_driver')
    serial_port = LaunchConfiguration('serial_port')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')
    odom_poll_every_n = LaunchConfiguration('odom_poll_every_n')
    depth_mode = LaunchConfiguration('depth_mode')
    depth_skip = LaunchConfiguration('depth_skip')
    video_device = LaunchConfiguration('video_device')
    rgb_width = LaunchConfiguration('rgb_width')
    rgb_height = LaunchConfiguration('rgb_height')
    rgb_fps = LaunchConfiguration('rgb_fps')
    rgb_pixel_format = LaunchConfiguration('rgb_pixel_format')
    rgb_camera_info_url = LaunchConfiguration('rgb_camera_info_url')
    base_to_camera_x = LaunchConfiguration('base_to_camera_x')
    base_to_camera_y = LaunchConfiguration('base_to_camera_y')
    base_to_camera_z = LaunchConfiguration('base_to_camera_z')
    base_to_camera_roll = LaunchConfiguration('base_to_camera_roll')
    base_to_camera_pitch = LaunchConfiguration('base_to_camera_pitch')
    base_to_camera_yaw = LaunchConfiguration('base_to_camera_yaw')
    openni2_lib_override_dir = LaunchConfiguration('openni2_lib_override_dir')

    return LaunchDescription([
        DeclareLaunchArgument('use_car2_driver', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyS0'),
        # Wheel radius from a 0.87 m straight run vs the lidar (ICP 0.892 m, front wall
        # 0.887 m): actual/odom = 1.027 -> 0.032 * 1.027 = 0.0329 m (2026-10-02).
        DeclareLaunchArgument('wheel_radius_m', default_value='0.0329'),
        # Effective skid-steer track, not the measured 0.43m: wheels scrub when
        # turning, so 0.43 over-reported yaw by ~19-20% (RTAB-Map visual registration;
        # lidar scan matching on a 360 deg spin: actual/odom = 0.833).
        # ... and the track scaled with it (rotation calibration needs radius/track
        # = 0.833 * 0.032/0.43): 0.0329 / 0.06199 = 0.53.
        DeclareLaunchArgument('wheel_separation_m', default_value='0.53'),
        # STAT poll every 2 control ticks (0.05s) -> 10Hz odom/TF. The default
        # 4 (5Hz) is too coarse for RTAB-Map to interpolate odom at image stamps.
        DeclareLaunchArgument('odom_poll_every_n', default_value='2'),
        # 320x240 depth keeps the compressedDepth stream WiFi-friendly; the
        # laptop upsamples it into the 640x480 RGB frame when registering.
        DeclareLaunchArgument('depth_mode', default_value='QVGA_30Hz'),
        # WiFi budget (measured on this robot): depth at the full 30Hz plus RGB
        # at jpeg_quality 95 (~5MB/s) only reached the laptop at 4-6Hz. Publish
        # every 3rd depth frame (10Hz); RGB JPEG quality is set in
        # config/robot_rgb_transport.yaml.
        DeclareLaunchArgument('depth_skip', default_value='2',
                              description='OpenNI2 data_skip: drop N depth frames per published one'),
        DeclareLaunchArgument('video_device', default_value='/dev/video0',
                              description='Astra Pro RGB UVC device (check v4l2-ctl --list-devices)'),
        DeclareLaunchArgument('rgb_width', default_value='640'),
        DeclareLaunchArgument('rgb_height', default_value='480'),
        # Keep the camera's native 30fps: asking usb_cam for fewer frames leaves
        # V4L2 buffers queued, so every published frame was ~1.1s stale at 10fps
        # (~0.6s at 30fps). JPEG 60 keeps 30fps at ~0.8MB/s.
        DeclareLaunchArgument('rgb_fps', default_value='30.0'),
        DeclareLaunchArgument('rgb_pixel_format', default_value='yuyv2rgb'),
        # Placeholder intrinsics -- calibrate (camera_calibration) and point
        # this at the result for accurate registration / mapping.
        DeclareLaunchArgument('rgb_camera_info_url', default_value=default_rgb_info),
        # Placeholder mount offset -- measure on the real robot (same as bringup.launch.py).
        DeclareLaunchArgument('base_to_camera_x', default_value='0.08'),
        DeclareLaunchArgument('base_to_camera_y', default_value='0.0'),
        DeclareLaunchArgument('base_to_camera_z', default_value=mount_default('base_to_camera_z', '0.15')),
        DeclareLaunchArgument('base_to_camera_roll', default_value=mount_default('base_to_camera_roll', '0.0')),
        DeclareLaunchArgument('base_to_camera_pitch', default_value=mount_default('base_to_camera_pitch', '0.0')),
        DeclareLaunchArgument('base_to_camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('openni2_lib_override_dir', default_value=default_openni2_override),
        AppendEnvironmentVariable('LD_LIBRARY_PATH', openni2_lib_override_dir, prepend=True),
        LogInfo(msg=(f'camera mount from {CAMERA_MOUNT_FILE}: {mount}' if mount else
                     f'no {CAMERA_MOUNT_FILE}; using placeholder camera mount angles')),

        Node(
            package='car2_driver',
            executable='car2_serial_node',
            name='car2_serial_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'wheel_radius_m': wheel_radius_m,
                'wheel_separation_m': wheel_separation_m,
                'odom_poll_every_n': odom_poll_every_n,
            }],
            condition=IfCondition(use_car2_driver),
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera_tf',
            output='screen',
            arguments=[
                '--x', base_to_camera_x,
                '--y', base_to_camera_y,
                '--z', base_to_camera_z,
                '--roll', base_to_camera_roll,
                '--pitch', base_to_camera_pitch,
                '--yaw', base_to_camera_yaw,
                '--frame-id', 'base_link',
                '--child-frame-id', 'camera_link',
            ],
        ),

        # Depth only, same reasoning as bringup.launch.py (no OpenNI2 color).
        # Standalone driver executable rather than a component in a
        # '/camera/container': with the laptop on the same ROS_DOMAIN_ID, the
        # laptop's bringup.launch.py (run_mapping.sh) loads its own OpenNI2
        # driver into whatever answers to '/camera/container' -- it landed in
        # this robot's container and crashed the running depth driver. A plain
        # process also makes respawn meaningful (a respawned container would
        # come back empty).
        Node(
            package='openni2_camera',
            executable='openni2_camera_driver',
            name='driver',
            namespace='camera',
            output='screen',
            respawn=True,
            respawn_delay=3.0,
            parameters=[{
                'depth_registration': False,
                'use_device_time': False,
                'depth_mode': depth_mode,
                'data_skip': depth_skip,
                'rgb_frame_id': 'camera_rgb_optical_frame',
                'depth_frame_id': 'camera_depth_optical_frame',
                'ir_frame_id': 'camera_ir_optical_frame',
            }],
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(openni2_share, 'launch', 'tfs.launch.py')),
            launch_arguments={'namespace': 'camera', 'tf_prefix': ''}.items(),
        ),

        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='color',
            namespace='camera',
            output='screen',
            parameters=[rgb_transport_params, {
                'video_device': video_device,
                'image_width': rgb_width,
                'image_height': rgb_height,
                'framerate': rgb_fps,
                'pixel_format': rgb_pixel_format,
                'camera_name': 'astra_pro_rgb',
                'camera_info_url': rgb_camera_info_url,
                'frame_id': 'camera_rgb_optical_frame',
            }],
            remappings=[
                ('image_raw', 'color/image_raw'),
                ('image_raw/compressed', 'color/image_raw/compressed'),
                ('image_raw/compressedDepth', 'color/image_raw/compressedDepth'),
                ('image_raw/theora', 'color/image_raw/theora'),
                ('image_raw/zstd', 'color/image_raw/zstd'),
                ('camera_info', 'color/camera_info'),
            ],
        ),
    ])
