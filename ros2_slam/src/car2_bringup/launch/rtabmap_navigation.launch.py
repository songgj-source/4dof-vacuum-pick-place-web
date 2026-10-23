"""Laptop-side autonomous navigation in a saved RTAB-Map map.

The robot (Raspberry Pi) runs robot_rgbd.launch.py (./scripts/remote_robot_rgbd.sh
start): car2 driver (/cmd_vel in, /odom + TF out) and the Astra Pro streams.
This launch, on the laptop:

  rtabmap_mapping.launch.py localization:=true
      same image pipeline as mapping; rtabmap loads the database and only
      localizes in it (nothing is added) -> /map + TF map->odom
      (replaces AMCL + map_server)
  rtabmap_util/point_cloud_xyz
      /rtabmap/depth_raw/image (already decompressed for rtabmap) -> 3D points
      /rtabmap/depth_cloud: the costmaps' and collision_monitor's obstacle
      source (the camera looks ~54 deg down, so a laser-like scan from one
      depth row would mostly see floor). roi_ratios drops the robot body.
  nav2_bringup/navigation_launch.py with config/nav2_rtabmap_params.yaml
      planner, MPPI controller, behaviors, velocity smoother, collision
      monitor -> /cmd_vel (speed limits fit car2's wheels: <=0.15 m/s, 0.7 rad/s)
  RViz (nav2_default_view): "2D Pose Estimate" -> /initialpose (rtabmap),
      "Nav2 Goal" -> navigate.

Usage:
  ros2 launch car2_bringup rtabmap_navigation.launch.py database_path:=/path/rtabmap.db
  ... body_roi_ratios:="0 0 0 0.22" use_rviz:=false use_rtabmap_viz:=true
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    nav2_share = get_package_share_directory('nav2_bringup')

    database_path = LaunchConfiguration('database_path')
    params_file = LaunchConfiguration('params_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    body_roi_ratios = LaunchConfiguration('body_roi_ratios')

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_share, 'launch', 'rtabmap_mapping.launch.py')),
        launch_arguments={
            'localization': 'true',
            'database_path': database_path,
            'use_rviz': 'false',              # navigation RViz below instead
            'use_rtabmap_viz': use_rtabmap_viz,
        }.items(),
    )

    depth_cloud = Node(
        package='rtabmap_util',
        executable='point_cloud_xyz',
        name='depth_cloud',
        namespace='rtabmap',
        output='screen',
        parameters=[{
            'decimation': 2,              # 320x240 depth -> 160x120 points
            'voxel_size': 0.05,
            'min_depth': 0.3,
            'max_depth': 3.0,
            'roi_ratios': body_roi_ratios,
            'approx_sync': True,
        }],
        remappings=[
            ('depth/image', '/rtabmap/depth_raw/image'),
            ('depth/camera_info', '/camera/depth_raw/camera_info'),
            ('cloud', '/rtabmap/depth_cloud'),
        ],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'params_file': params_file,
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(nav2_share, 'rviz', 'nav2_default_view.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('database_path', description='RTAB-Map database made by rtabmap_mapping.launch.py'),
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(bringup_share, 'config', 'nav2_rtabmap_params.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='false'),
        # Same fractions as the camera web UI's RoiRatios (robot body at the image
        # bottom). Multiples of 0.025 only: otherwise the cropped 320x240 image
        # doesn't divide by decimation 2 and point_cloud_xyz ignores the ROI.
        DeclareLaunchArgument('body_roi_ratios', default_value='0 0 0 0.225'),
        localization,
        depth_cloud,
        nav2,
        rviz,
    ])
