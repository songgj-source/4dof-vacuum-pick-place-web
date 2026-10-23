"""Laptop-side RTAB-Map RGB-D mapping from the robot's remote camera streams.

The robot (Raspberry Pi) runs robot_rgbd.launch.py: car2 driver (/odom + TF),
Astra Pro depth (OpenNI2) and RGB (usb_cam). Only compressed images cross the
network; everything heavy runs here:

  /camera/color/image_raw/compressed     --republish--> /rtabmap/rgb/image
  /camera/depth_raw/image/compressedDepth --republish--> /rtabmap/depth_raw/image
  depth_image_proc/register (depth -> RGB optical frame, 320x240 -> 640x480)
  rtabmap_sync/rgbd_sync -> rtabmap_slam/rtabmap (+ wheel odometry from /odom)

Outputs: /map (2D occupancy grid, map_saver-compatible -> scripts/save_map.sh),
/rtabmap/mapData, /rtabmap/mapGraph, /rtabmap/cloud_map, TF map->odom, and the
RTAB-Map database (database_path, default ~/.ros/rtabmap.db).

Usage:
  ros2 launch car2_bringup rtabmap_mapping.launch.py                  # continue the existing database
  ros2 launch car2_bringup rtabmap_mapping.launch.py new_map:=true    # delete the database and start over
  ros2 launch car2_bringup rtabmap_mapping.launch.py use_rtabmap_viz:=true use_rviz:=false
  ros2 launch car2_bringup rtabmap_mapping.launch.py localization:=true  # use the map, don't change it
                                                                         # (rtabmap_navigation.launch.py)
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    bringup_share = get_package_share_directory('car2_bringup')
    rviz_config = os.path.join(bringup_share, 'rviz', 'rtabmap.rviz')

    database_path = LaunchConfiguration('database_path')
    new_map = LaunchConfiguration('new_map')
    localization = LaunchConfiguration('localization')
    use_rviz = LaunchConfiguration('use_rviz')
    use_rtabmap_viz = LaunchConfiguration('use_rtabmap_viz')
    rgb_topic = LaunchConfiguration('rgb_topic')
    rgb_info_topic = LaunchConfiguration('rgb_info_topic')
    depth_topic = LaunchConfiguration('depth_topic')
    depth_info_topic = LaunchConfiguration('depth_info_topic')

    # Everything under one namespace so the intermediate topics don't clutter /.
    ns = 'rtabmap'

    # Decompress the robot's streams once, locally, then register depth into
    # the RGB frame (the Astra Pro's RGB is a separate UVC sensor, so the
    # driver can't hand us registered depth). use_rgb_timestamp makes the
    # registered depth carry the RGB stamp, so rgbd_sync pairs them exactly.
    # Queues are deep because the Astra Pro's UVC RGB frames reach us ~0.5s
    # later than the depth frame with the same capture time (measured).
    # Plain processes, not components in a named container: launch loads
    # components into a container *by name*, and when a previous run's
    # '/rtabmap/rgbd_front_end' was still shutting down the new run's load
    # requests went to it and the pipeline silently never started.
    front_end = [
        Node(
            package='image_transport',
            executable='republish',
            name='rgb_decompress',
            namespace=ns,
            output='screen',
            parameters=[{'in_transport': 'compressed', 'out_transport': 'raw'}],
            remappings=[('in/compressed', [rgb_topic, '/compressed']), ('out', 'rgb/image')],
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='depth_decompress',
            namespace=ns,
            output='screen',
            parameters=[{'in_transport': 'compressedDepth', 'out_transport': 'raw'}],
            remappings=[('in/compressedDepth', [depth_topic, '/compressedDepth']),
                        ('out', 'depth_raw/image')],
        ),
        Node(
            package='depth_image_proc',
            executable='register_node',
            name='register_depth',
            namespace=ns,
            output='screen',
            parameters=[{
                'queue_size': 30,
                'fill_upsampling_holes': True,
                'use_rgb_timestamp': True,
            }],
            remappings=[
                ('depth/image_rect', 'depth_raw/image'),
                ('depth/camera_info', depth_info_topic),
                ('rgb/camera_info', rgb_info_topic),
                ('depth_registered/image_rect', 'depth_registered/image'),
                ('depth_registered/camera_info', 'depth_registered/camera_info'),
            ],
        ),
    ]

    rgbd_sync = Node(
        package='rtabmap_sync',
        executable='rgbd_sync',
        name='rgbd_sync',
        namespace=ns,
        output='screen',
        parameters=[{
            'approx_sync': True,
            'approx_sync_max_interval': 0.02,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
        }],
        remappings=[
            ('rgb/image', 'rgb/image'),
            ('depth/image', 'depth_registered/image'),
            ('rgb/camera_info', rgb_info_topic),
        ],
    )

    rtabmap_params = {
        'frame_id': 'base_link',
        # Empty odom_frame_id = take wheel odometry from the /odom topic and
        # sync it with the images, instead of looking up the odom->base_link TF
        # at each image stamp. When odom arrives late or irregularly from the Pi
        # the TF lookup fails outright ("extrapolation into the future"), while
        # the sync queue simply waits for the matching /odom message.
        'odom_frame_id': '',
        'map_frame_id': 'map',
        'publish_tf': True,
        'subscribe_rgbd': True,
        'subscribe_depth': False,
        'subscribe_rgb': False,
        'subscribe_scan': False,
        'approx_sync': True,
        # /odom is 10Hz, images 10-15Hz: allow a half odom period of mismatch.
        'approx_sync_max_interval': 0.06,
        # Static camera TFs only now, but they also come from the Pi.
        'wait_for_transform': 1.0,
        'topic_queue_size': 30,
        'sync_queue_size': 30,
        'database_path': database_path,
        # --- RTAB-Map core parameters (strings) ---
        'Reg/Force3DoF': 'true',          # ground robot: x, y, yaw only
        'Reg/Strategy': '0',              # visual registration
        'Vis/MinInliers': '12',
        # Re-register every new node against the previous one visually and use
        # that instead of the raw wheel-odometry delta. Without it the map
        # inherits all skid-steer yaw error and only the rare loop closure
        # corrects it.
        'RGBD/NeighborLinkRefining': 'true',
        'RGBD/LinearUpdate': '0.05',
        'RGBD/AngularUpdate': '0.05',
        'RGBD/OptimizeMaxError': '3.0',
        'Rtabmap/DetectionRate': '2',
        # Mapping adds to the database; localization:=true only localizes in it
        # (the saved map is loaded whole and left unchanged). value_type=str:
        # rtabmap parameters are strings, a bare 'false' would become a bool.
        'Mem/IncrementalMemory': ParameterValue(
            PythonExpression(["'false' if '", localization, "' == 'true' else 'true'"]), value_type=str),
        'Mem/InitWMWithAllNodes': ParameterValue(localization, value_type=str),
        'Grid/Sensor': '1',               # occupancy grid from depth
        # 3D local grids so /rtabmap/cloud_map is a real 3D cloud. It is built
        # through the grid filters (Grid/DepthRoiRatios, RangeMin/Max, ground /
        # obstacle heights), so the robot body cut by the web UI's RoiRatios is
        # gone from it -- unlike RViz's MapCloud display, which re-projects the
        # raw depth images itself. /map (2D) is still produced.
        'Grid/3D': 'true',
        'Grid/RayTracing': 'true',
        'Grid/CellSize': '0.05',
        'Grid/RangeMin': '0.6',           # Astra Pro is unreliable closer than ~0.6m
        'Grid/RangeMax': '4.0',
        'Grid/NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '0.05',
        'Grid/MaxObstacleHeight': '0.6',
    }

    def rtabmap_node(arguments, condition):
        return Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            namespace=ns,
            output='screen',
            parameters=[rtabmap_params],
            remappings=[
                ('rgbd_image', 'rgbd_image'),
                ('odom', '/odom'),
                # 2D grid on the standard /map topic so scripts/save_map.sh and
                # Nav2's map_server tooling work unchanged.
                ('map', '/map'),
                # RViz "2D Pose Estimate" -> tell rtabmap where the robot is
                ('initialpose', '/initialpose'),
            ],
            arguments=arguments,
            condition=condition,
        )

    # '-d' = delete the database on start (a fresh map).
    # Never delete the database in localization mode, whatever new_map says.
    delete_db = PythonExpression(["'", new_map, "' == 'true' and '", localization, "' != 'true'"])
    rtabmap_new = rtabmap_node(['-d'], IfCondition(delete_db))
    rtabmap_continue = rtabmap_node([], UnlessCondition(delete_db))

    rtabmap_viz = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        name='rtabmap_viz',
        namespace=ns,
        output='screen',
        parameters=[{
            'frame_id': 'base_link',
            'odom_frame_id': '',
            'subscribe_rgbd': True,
            'subscribe_odom_info': False,
            'approx_sync': True,
            'approx_sync_max_interval': 0.06,
            'wait_for_transform': 1.0,
            'topic_queue_size': 30,
            'sync_queue_size': 30,
        }],
        remappings=[('rgbd_image', 'rgbd_image'), ('odom', '/odom')],
        condition=IfCondition(use_rtabmap_viz),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument('database_path',
                              default_value=os.path.expanduser('~/.ros/rtabmap.db')),
        DeclareLaunchArgument('localization', default_value='false',
                              description='true = localize in the existing map without modifying it'),
        DeclareLaunchArgument('new_map', default_value='false',
                              description='true = delete database_path and start a new map'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('use_rtabmap_viz', default_value='false'),
        DeclareLaunchArgument('rgb_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('rgb_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('depth_topic', default_value='/camera/depth_raw/image'),
        DeclareLaunchArgument('depth_info_topic', default_value='/camera/depth_raw/camera_info'),
        *front_end,
        rgbd_sync,
        rtabmap_new,
        rtabmap_continue,
        rtabmap_viz,
        rviz,
    ])
