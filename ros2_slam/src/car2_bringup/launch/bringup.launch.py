"""Hardware bringup shared by mapping and navigation: car2 serial driver,
depth camera, depth->laserscan conversion, and the base_link->camera_link
static transform.

Camera mount offset (base_to_camera_*) and the car2_driver wheel geometry
are placeholders -- measure the real robot and override them (via these
launch arguments, or by editing car2_bringup/config directly) before
trusting odometry, the scan-to-base_link geometry, or Nav2 motion.

Ubuntu's stock libopenni2-0 (PS1080 driver) does not recognize the Orbbec
Astra Pro's OEM vendor ID. `openni2_lib_override_dir` prepends Orbbec's own
OpenNI2 runtime (installed by scripts/install_orbbec_openni2.sh into
third_party/orbbec_openni2/) onto LD_LIBRARY_PATH for the camera process, so
ros-jazzy-openni2-camera loads Orbbec's libOpenNI2.so.0 + liborbbec.so
instead. If that directory doesn't exist (script not run yet), this is a
no-op and the camera container falls back to the system library.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    openni2_share = get_package_share_directory('openni2_camera')
    depth_laserscan_config = os.path.join(
        bringup_share, 'config', 'depthimage_to_laserscan.yaml')
    # bringup_share = <ws>/install/car2_bringup/share/car2_bringup
    default_openni2_override = os.path.abspath(os.path.join(
        bringup_share, '..', '..', '..', '..', 'third_party', 'orbbec_openni2'))

    serial_port = LaunchConfiguration('serial_port')
    # use_car2_driver:=false leaves the car driver out so it can run in its own
    # terminal (scripts/run_car2_driver.sh) and be restarted independently.
    use_car2_driver = LaunchConfiguration('use_car2_driver')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')
    camera_namespace = LaunchConfiguration('camera_namespace')
    depth_image_topic = LaunchConfiguration('depth_image_topic')
    depth_camera_info_topic = LaunchConfiguration('depth_camera_info_topic')
    base_to_camera_x = LaunchConfiguration('base_to_camera_x')
    base_to_camera_y = LaunchConfiguration('base_to_camera_y')
    base_to_camera_z = LaunchConfiguration('base_to_camera_z')
    base_to_camera_roll = LaunchConfiguration('base_to_camera_roll')
    base_to_camera_pitch = LaunchConfiguration('base_to_camera_pitch')
    base_to_camera_yaw = LaunchConfiguration('base_to_camera_yaw')
    openni2_lib_override_dir = LaunchConfiguration('openni2_lib_override_dir')

    return LaunchDescription([
        DeclareLaunchArgument('use_car2_driver', default_value='true'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        # Wheel radius from a 0.87 m straight run vs the lidar (ICP 0.892 m, front wall
        # 0.887 m): actual/odom = 1.027 -> 0.032 * 1.027 = 0.0329 m (2026-10-02).
        DeclareLaunchArgument('wheel_radius_m', default_value='0.0329'),
        # Effective skid-steer track, not the measured 0.43m: wheels scrub when
        # turning, so 0.43 over-reported yaw by ~19-20% (RTAB-Map visual registration;
        # lidar scan matching on a 360 deg spin: actual/odom = 0.833).
        # ... and the track scaled with it (rotation calibration needs radius/track
        # = 0.833 * 0.032/0.43): 0.0329 / 0.06199 = 0.53.
        DeclareLaunchArgument('wheel_separation_m', default_value='0.53'),
        DeclareLaunchArgument('camera_namespace', default_value='camera'),
        # openni2_camera (ros-drivers, jazzy branch) publishes the unregistered
        # depth stream as "depth_raw/image" (+ "depth_raw/camera_info"), and
        # only the registered stream as "depth/image" -- there is no
        # "depth/image_raw" topic. depth_registration is set False below (we
        # don't need color-aligned depth for a 2D laserscan), so we consume
        # depth_raw. image_transport's camera publisher is lazy: it only calls
        # startDepthStream() once something actually subscribes, so getting
        # this topic name wrong silently yields zero frames rather than an error.
        DeclareLaunchArgument('depth_image_topic', default_value='/camera/depth_raw/image'),
        DeclareLaunchArgument('depth_camera_info_topic', default_value='/camera/depth_raw/camera_info'),
        # Placeholder mount offset -- measure base_link (ground-projected wheel
        # center) to the camera's optical center and set these accurately.
        DeclareLaunchArgument('base_to_camera_x', default_value='0.08'),
        DeclareLaunchArgument('base_to_camera_y', default_value='0.0'),
        DeclareLaunchArgument('base_to_camera_z', default_value='0.15'),
        DeclareLaunchArgument('base_to_camera_roll', default_value='0.0'),
        DeclareLaunchArgument('base_to_camera_pitch', default_value='0.0'),
        DeclareLaunchArgument('base_to_camera_yaw', default_value='0.0'),
        DeclareLaunchArgument('openni2_lib_override_dir', default_value=default_openni2_override),
        AppendEnvironmentVariable('LD_LIBRARY_PATH', openni2_lib_override_dir, prepend=True),

        Node(
            package='car2_driver',
            executable='car2_serial_node',
            name='car2_serial_node',
            output='screen',
            parameters=[{
                'serial_port': serial_port,
                'wheel_radius_m': wheel_radius_m,
                'wheel_separation_m': wheel_separation_m,
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

        # Own ComposableNodeContainer instead of openni2_camera's camera_only.launch.py:
        # that launch file hardcodes depth_registration=True, which forces the
        # Astra Pro's separate UVC color sensor to start too -- and its default
        # "640x480@30Hz RGB888" request isn't a mode the UVC device actually
        # supports ("Unsupported color video mode" -> depth stream never starts
        # either). We only need raw depth for depthimage_to_laserscan, so skip
        # color/registration entirely. Reuses openni2_camera's own tfs.launch.py
        # for the camera-internal static frames (that part is unaffected).
        ComposableNodeContainer(
            name='container',
            namespace=camera_namespace,
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='openni2_camera',
                    plugin='openni2_wrapper::OpenNI2Driver',
                    name='driver',
                    namespace=camera_namespace,
                    parameters=[{
                        'depth_registration': False,
                        'use_device_time': True,
                        'rgb_frame_id': [camera_namespace, '_rgb_optical_frame'],
                        'depth_frame_id': [camera_namespace, '_depth_optical_frame'],
                        'ir_frame_id': [camera_namespace, '_ir_optical_frame'],
                    }],
                ),
            ],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(openni2_share, 'launch', 'tfs.launch.py')),
            launch_arguments={'namespace': camera_namespace, 'tf_prefix': ''}.items(),
        ),

        Node(
            package='depthimage_to_laserscan',
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan',
            output='screen',
            remappings=[
                ('depth', depth_image_topic),
                ('depth_camera_info', depth_camera_info_topic),
            ],
            parameters=[depth_laserscan_config],
        ),
    ])
