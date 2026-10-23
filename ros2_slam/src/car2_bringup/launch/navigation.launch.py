"""Navigate a previously saved map: bringup (camera + car2 driver + depth->scan)
+ Nav2 (map_server + AMCL + planner/controller/behaviors) via nav2_bringup.

Usage:
  ros2 launch car2_bringup navigation.launch.py map:=/path/to/map.yaml
Then set the robot's initial pose in RViz ("2D Pose Estimate") and send it a
"Nav2 Goal", or use nav2_simple_commander / the GoToPose service pattern from
myrobot_ws.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    default_params = os.path.join(bringup_share, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    serial_port = LaunchConfiguration('serial_port')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'serial_port': serial_port,
            'wheel_radius_m': wheel_radius_m,
            'wheel_separation_m': wheel_separation_m,
        }.items(),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'map': map_yaml,
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    rviz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'rviz_launch.py')),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='Full path to the saved map yaml file'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('use_rviz', default_value='true'),
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
        bringup,
        nav2,
        rviz,
    ])
