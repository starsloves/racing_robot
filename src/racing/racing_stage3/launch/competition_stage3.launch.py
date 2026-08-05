import os
import logging

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import launch.logging as launch_logging


def generate_launch_description():
    launch_logging.launch_config.level = logging.ERROR
    stage3_dir = get_package_share_directory('racing_stage3')
    bringup_dir = get_package_share_directory('origincar_bringup')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')

    # 使用统一配置文件
    stage3_config = os.path.join(stage3_dir, 'config', 'stage3_controller.yaml')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')

    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='false')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='false')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='false')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='true')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    cmd_topic_arg = DeclareLaunchArgument(
        'cmd_topic',
        default_value='/cmd_vel',
        description='Stage3 command output; production uses the direct /cmd_vel owner',
    )
    standby_arg = DeclareLaunchArgument('standby', default_value='true')
    standalone_map_overlay_arg = DeclareLaunchArgument(
        'standalone_map_overlay',
        default_value='false',
        description='Include map server and map→odom TF for standalone testing',
    )
    include_map_overlay_arg = DeclareLaunchArgument(
        'include_map_overlay',
        default_value='false',
        description='Deprecated compatibility argument; use standalone_map_overlay',
    )
    enable_test_publisher_arg = DeclareLaunchArgument(
        'enable_test_publisher',
        default_value='false',
        description='Publish phase and direction topics for standalone testing',
    )
    map_to_odom_x_arg = DeclareLaunchArgument(
        'map_to_odom_x',
        default_value='0.50',
        description='map→odom x used by Stage3 pose transform',
    )
    map_to_odom_y_arg = DeclareLaunchArgument(
        'map_to_odom_y',
        default_value='0.20',
        description='map→odom y used by Stage3 pose transform',
    )
    map_to_odom_yaw_arg = DeclareLaunchArgument(
        'map_to_odom_yaw',
        default_value='0.1745329252',
        description='map→odom yaw used by Stage3 pose transform',
    )

    actions = []
    if os.path.exists(support_launch_path):
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(support_launch_path),
                launch_arguments={
                    'include_bringup': LaunchConfiguration('include_bringup'),
                    'include_lidar': LaunchConfiguration('include_lidar'),
                    'include_camera': LaunchConfiguration('include_camera'),
                    'include_depth': LaunchConfiguration('include_depth'),
                    'rgb_fps': LaunchConfiguration('rgb_fps'),
                    'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
                    'carto_slam': LaunchConfiguration('carto_slam'),
                }.items(),
            )
        )

    map_overlay_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_overlay_launch_path),
        launch_arguments={
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            'odom_frame': 'odom_combined',
        }.items(),
        condition=IfCondition(LaunchConfiguration('standalone_map_overlay')),
    )

    test_publisher = Node(
        package='racing_stage3',
        executable='stage_test_publisher',
        name='stage_test_publisher',
        parameters=[{
            'stage_number': 3,
            'test_direction': LaunchConfiguration('test_direction'),
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('enable_test_publisher')),
    )

    stage3_return_navigator = Node(
        package='racing_stage3',
        executable='stage3_return_navigator',
        name='stage3_return_navigator',
        parameters=[
            stage3_config,  # 统一配置文件
            {
                'test_direction': LaunchConfiguration('test_direction'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'standby': LaunchConfiguration('standby'),
                'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
                'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
                'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            }
        ],
        output='log',
    )

    # Preserve ROS startup and shutdown diagnostics on disk while keeping the
    # terminal reserved for the operator-facing status messages.
    runtime_ros_log_dir = SetEnvironmentVariable(
        'ROS_LOG_DIR', '/home/sunrise/dev_ws/log/competition_stage3/ros'
    )
    isolate_process_output = SetEnvironmentVariable(
        'OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'own_log'
    )

    return LaunchDescription([
        runtime_ros_log_dir,
        isolate_process_output,
        include_bringup_arg,
        include_lidar_arg,
        include_camera_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        carto_slam_arg,
        test_direction_arg,
        cmd_topic_arg,
        standby_arg,
        standalone_map_overlay_arg,
        include_map_overlay_arg,
        enable_test_publisher_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        *actions,
        map_overlay_stack,
        test_publisher,
        stage3_return_navigator,
    ])
