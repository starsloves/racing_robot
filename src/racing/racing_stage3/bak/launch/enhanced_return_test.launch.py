"""Stage3 enhanced return test launch
- Simulates Stage2 ending state (robot at map (2.80, 3.25))
- Starts EnhancedReturnNavigator with Pure Pursuit + Avoidance
- Starts Aurora 930 camera for P vision detection (shared Web port 8080)
- Uses map_overlay to provide map->odom TF
- Starts hardware drivers, then auto-triggers phase=3 after delay
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stage3_dir = get_package_share_directory('racing_stage3')
    bringup_dir = get_package_share_directory('origincar_bringup')
    config = os.path.join(stage3_dir, 'config', 'enhanced_return.yaml')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('test_direction', default_value='clockwise'),
        DeclareLaunchArgument('auto_start_phase3', default_value='true'),
        DeclareLaunchArgument('phase3_start_delay_sec', default_value='3.0'),
        DeclareLaunchArgument('include_bringup', default_value='true'),
        DeclareLaunchArgument('include_lidar', default_value='true'),
        DeclareLaunchArgument('include_bno055', default_value='false'),
        DeclareLaunchArgument('include_camera', default_value='true'),
        DeclareLaunchArgument('use_relay', default_value='false'),
        DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('rgb_fps', default_value='15'),
        DeclareLaunchArgument('resolution_mode_index', default_value='2'),

        # Aurora 930 camera node (for P detection, starts first)
        Node(
            package='deptrum-ros-driver-aurora930',
            executable='aurora930_node',
            namespace='aurora',
            parameters=[{
                'rgb_enable': True,
                'ir_enable': False,
                'depth_enable': False,
                'rgbd_enable': False,
                'point_cloud_enable': False,
                'boot_order': 1,
                'rgb_fps': LaunchConfiguration('rgb_fps'),
                'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
                'align_mode': False,
                'log_dir': '/tmp/',
                'stream_sdk_log_enable': False,
                'heart_enable': False,
            }],
            output='log',
            condition=IfCondition(LaunchConfiguration('include_camera')),
        ),

        # Hardware drivers
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(support_launch_path),
            launch_arguments={
                'include_bringup': LaunchConfiguration('include_bringup'),
                'include_lidar': LaunchConfiguration('include_lidar'),
                'include_bno055': LaunchConfiguration('include_bno055'),
                'include_camera': 'false',  # Camera already started above
                'include_depth': 'false',
                'carto_slam': 'false',
            }.items(),
        ),

        # map_overlay: provides map <-> odom_combined TF
        # Stage2 ending position: map (2.80, 3.25)
        # map_to_odom_yaw=180deg: clockwise ending faces -X/180deg
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_overlay_launch_path),
            launch_arguments={
                'map_to_odom_x': '2.80',
                'map_to_odom_y': '3.25',
                'map_to_odom_yaw': '3.14159',  # 180 degrees
                'odom_frame': 'odom_combined',
            }.items(),
        ),

        # Enhanced return navigator (map coordinate waypoints, synced with map_to_odom)
        Node(
            package='racing_stage3',
            executable='enhanced_return_navigator',
            name='enhanced_return_navigator',
            parameters=[config, {
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'test_direction': LaunchConfiguration('test_direction'),
                'map_to_odom_x': 2.80,
                'map_to_odom_y': 3.25,
                'map_to_odom_yaw': 3.14159,
            }],
            output='screen',
            emulate_tty=True,
        ),

        # Optional: cmd_vel relay
        Node(
            package='racing_stage3',
            executable='twist_cmd_relay',
            name='stage3_test_cmd_relay',
            parameters=[{
                'input_topic': LaunchConfiguration('cmd_topic'),
                'output_topic': '/cmd_vel',
            }],
            output='log',
            condition=IfCondition(LaunchConfiguration('use_relay')),
        ),

        # Auto-trigger phase=3 + simulate QR direction after delay
        TimerAction(
            period=LaunchConfiguration('phase3_start_delay_sec'),
            actions=[
                Node(
                    package='racing_stage3',
                    executable='stage3_test_simulator',
                    name='stage3_test_simulator',
                    parameters=[{
                        'phase_topic': 'competition_phase',
                        'task_topic': 'competition_qr_task',
                        'phase_value': 3,
                        'publish_count': 5,
                        'publish_period_sec': 0.2,
                        'start_delay_sec': 0.0,
                        'simulate_qr_task': True,
                        'qr_task_value': LaunchConfiguration('test_direction'),
                    }],
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('auto_start_phase3')),
                ),
            ],
        ),
    ])
