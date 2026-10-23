"""Laptop-side navigation in a 2D lidar map (saved by save_lidar_map.sh).

The robot runs robot_lidar.launch.py (./scripts/remote_car2_lidar.sh start).
This launch: map_server + AMCL (nav2_bringup localization_launch.py) on /scan,
Nav2 (navigation_launch.py) with config/nav2_lidar_params.yaml, and Nav2's
RViz. Set the robot's pose with "2D Pose Estimate", then send a "Nav2 Goal".

AMCL publishes nothing (and Nav2 can't start) until it has an initial pose,
so a few seconds after start this launch sends one: by default the map origin
= where mapping started. Start the robot there, pass initial_x/y/yaw, or fix
it with "2D Pose Estimate" (set_initial_pose:=false to skip).

Usage:
  ros2 launch car2_bringup lidar_navigation.launch.py map:=/path/maps/office.yaml
  ... initial_x:=1.2 initial_y:=-0.5 initial_yaw:=1.57
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')

    common = {'params_file': params_file, 'use_sim_time': 'false', 'autostart': 'true'}

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'localization_launch.py')),
        launch_arguments={'map': map_yaml, **common}.items(),
    )
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments=common.items(),
    )
    # Initial pose for AMCL (map frame), sent once AMCL subscribes (-w 1).
    initial_pose = TimerAction(period=4.0, actions=[ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '--once', '-w', '1', '/initialpose',
             'geometry_msgs/msg/PoseWithCovarianceStamped',
             PythonExpression([
                 "'{header: {frame_id: map}, pose: {pose: {position: {x: ' + str(", LaunchConfiguration('initial_x'),
                 ") + ', y: ' + str(", LaunchConfiguration('initial_y'),
                 ") + '}, orientation: {z: ' + str(__import__('math').sin(", LaunchConfiguration('initial_yaw'),
                 " / 2)) + ', w: ' + str(__import__('math').cos(", LaunchConfiguration('initial_yaw'),
                 " / 2)) + '}}, covariance: [0.25,0,0,0,0,0, 0,0.25,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0, 0,0,0,0,0,0.07]}}'"]),
             ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('set_initial_pose')),
    )])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('map', description='map yaml saved by save_lidar_map.sh'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(bringup_share, 'config', 'nav2_lidar_params.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('set_initial_pose', default_value='true'),
        DeclareLaunchArgument('initial_x', default_value='0.0'),
        DeclareLaunchArgument('initial_y', default_value='0.0'),
        DeclareLaunchArgument('initial_yaw', default_value='0.0'),
        localization,
        initial_pose,
        navigation,
        rviz,
    ])
