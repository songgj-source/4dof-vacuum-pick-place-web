"""Laptop-side 2D lidar mapping with slam_toolbox.

The robot (Raspberry Pi) runs robot_lidar.launch.py (./scripts/remote_car2_lidar.sh
start): car2 driver (/cmd_vel in, /odom + TF odom->base_link out) and the
LD-series lidar (/scan, laser_frame). This launch runs slam_toolbox on /scan +
odom TF -> /map + TF map->odom, plus RViz.

Usage:
  ros2 launch car2_bringup lidar_mapping.launch.py
  ros2 launch car2_bringup lidar_mapping.launch.py map_file:=/path/maps/office   # continue a
                                                    # map saved by save_lidar_map.sh (.posegraph)
Save with scripts/save_lidar_map.sh <name>.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    use_rviz = LaunchConfiguration('use_rviz')
    map_file = LaunchConfiguration('map_file')

    # slam_toolbox is a lifecycle node in Jazzy: configure + activate it here
    # (see mapping.launch.py).
    slam = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[
            os.path.join(bringup_share, 'config', 'slam_toolbox_lidar_params.yaml'),
            {'use_sim_time': False, 'use_lifecycle_manager': False,
             # '' = new map; a saved pose graph (without extension) = continue it
             'map_file_name': map_file, 'map_start_at_dock': True},
        ],
    )
    configure = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_CONFIGURE))
    activate = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam, start_state='configuring', goal_state='inactive',
        entities=[
            LogInfo(msg='slam_toolbox configured, activating.'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam), transition_id=Transition.TRANSITION_ACTIVATE)),
        ]))

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', os.path.join(bringup_share, 'rviz', 'mapping.rviz')],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('map_file', default_value='',
                              description='saved slam_toolbox pose graph to continue (path without extension)'),
        slam,
        configure,
        activate,
        rviz,
    ])
