"""Map collection: bringup (camera + car2 driver + depth->scan) + slam_toolbox.

Usage:
  ros2 launch car2_bringup mapping.launch.py
  (use_rviz:=false to skip RViz, e.g. on a headless machine)
Then drive the robot (teleop_twist_keyboard on /cmd_vel) around the space,
and save the map with scripts/save_map.sh once slam_toolbox has built it up.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription,
                            LogInfo, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.events import matches_action
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    slam_params = os.path.join(bringup_share, 'config', 'slam_toolbox_params.yaml')
    rviz_config = os.path.join(bringup_share, 'rviz', 'mapping.rviz')

    serial_port = LaunchConfiguration('serial_port')
    use_car2_driver = LaunchConfiguration('use_car2_driver')
    use_rviz = LaunchConfiguration('use_rviz')
    wheel_radius_m = LaunchConfiguration('wheel_radius_m')
    wheel_separation_m = LaunchConfiguration('wheel_separation_m')

    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, 'launch', 'bringup.launch.py')),
        launch_arguments={
            'serial_port': serial_port,
            'use_car2_driver': use_car2_driver,
            'wheel_radius_m': wheel_radius_m,
            'wheel_separation_m': wheel_separation_m,
        }.items(),
    )

    # Jazzy's slam_toolbox is a lifecycle node: started as a plain Node it sits
    # in "unconfigured" forever (no /map, no map->odom TF). Configure then
    # activate it ourselves, as slam_toolbox's own online_async_launch.py does.
    slam_toolbox_node = LifecycleNode(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
        parameters=[slam_params, {'use_sim_time': False, 'use_lifecycle_manager': False}],
    )

    configure_slam_toolbox = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(slam_toolbox_node),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))

    activate_slam_toolbox = RegisterEventHandler(OnStateTransition(
        target_lifecycle_node=slam_toolbox_node,
        start_state='configuring',
        goal_state='inactive',
        entities=[
            LogInfo(msg='slam_toolbox configured, activating.'),
            EmitEvent(event=ChangeState(
                lifecycle_node_matcher=matches_action(slam_toolbox_node),
                transition_id=Transition.TRANSITION_ACTIVATE,
            )),
        ],
    ))

    # Live view of the map being built: /map, /scan, TF and /odom, plus
    # slam_toolbox's panel (Save Map / Serialize Map buttons). No depth Image
    # display: together with SlamToolboxPlugin it segfaults rviz2 on startup.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': False}],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_car2_driver', default_value='true'),
        # Wheel radius from a 0.87 m straight run vs the lidar (ICP 0.892 m, front wall
        # 0.887 m): actual/odom = 1.027 -> 0.032 * 1.027 = 0.0329 m (2026-10-02).
        DeclareLaunchArgument('wheel_radius_m', default_value='0.0329'),
        # Effective skid-steer track, not the measured 0.43m: wheels scrub when
        # turning, so 0.43 over-reported yaw by ~19-20% (RTAB-Map visual registration;
        # lidar scan matching on a 360 deg spin: actual/odom = 0.833).
        # ... and the track scaled with it (rotation calibration needs radius/track
        # = 0.833 * 0.032/0.43): 0.0329 / 0.06199 = 0.53.
        DeclareLaunchArgument('wheel_separation_m', default_value='0.53'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        bringup,
        slam_toolbox_node,
        configure_slam_toolbox,
        activate_slam_toolbox,
        rviz,
    ])
