import os
import logging

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch.logging as launch_logging


def generate_launch_description():
    launch_logging.launch_config.level = logging.ERROR
    stage2_dir = get_package_share_directory('racing_stage2')
    bringup_dir = get_package_share_directory('origincar_bringup')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')

    # 使用统一配置文件
    stage2_config = os.path.join(stage2_dir, 'config', 'stage2_controller.yaml')
    obstacle_marker_config = os.path.join(stage2_dir, 'config', 'obstacle_circle_markers.yaml')

    include_bringup_arg = DeclareLaunchArgument(
        'include_bringup',
        default_value='true',
        description='Standalone Stage2 starts base/EKF support by default; total launch passes false',
    )
    include_lidar_arg = DeclareLaunchArgument(
        'include_lidar',
        default_value='true',
        description='Standalone Stage2 starts lidar by default; total launch passes false',
    )
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='false')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    include_obstacle_markers_arg = DeclareLaunchArgument('include_obstacle_markers', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
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
        default_value='true',
        description='Publish an isolated direction task for standalone testing; total launch passes false',
    )
    map_to_odom_x_arg = DeclareLaunchArgument(
        'map_to_odom_x',
        default_value='2.80',
        description='Stage2 start X position in map frame (channel_entry)',
    )
    map_to_odom_y_arg = DeclareLaunchArgument(
        'map_to_odom_y',
        default_value='3.10',
        description='Stage2 start Y position in map frame (channel_entry)',
    )
    map_to_odom_yaw_arg = DeclareLaunchArgument(
        'map_to_odom_yaw',
        default_value='1.5707963268',
        description='Stage2 start yaw in map frame (rad, 90°)',
    )
    test_direction_arg = DeclareLaunchArgument(
        'test_direction',
        default_value='clockwise',
        description='Fallback direction for standalone testing (clockwise/counterclockwise)',
    )
    cmd_topic_arg = DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel')
    standby_arg = DeclareLaunchArgument('standby', default_value='true')
    task_topic_arg = DeclareLaunchArgument(
        'task_topic',
        default_value='competition_qr_task',
        description='QR task topic for production Stage2',
    )
    test_task_topic_arg = DeclareLaunchArgument(
        'test_task_topic',
        default_value='/stage2_test/competition_qr_task',
        description='Isolated task topic used only when enable_test_publisher=true',
    )

    active_task_topic = PythonExpression([
        "'", LaunchConfiguration('test_task_topic'), "' if '",
        LaunchConfiguration('enable_test_publisher'), "'.lower() in ('true', '1', 'yes') else '",
        LaunchConfiguration('task_topic'), "'"
    ])

    # Optional support stack for standalone runs only
    support_actions = []
    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')
    if os.path.exists(support_launch_path):
        support_actions.append(
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
        package='racing_stage2',
        executable='stage_test_publisher',
        name='stage_test_publisher',
        parameters=[{
            'stage_number': 2,
            'test_direction': LaunchConfiguration('test_direction'),
            'task_topic': active_task_topic,
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('enable_test_publisher')),
    )

    stage2_navigator = Node(
        package='racing_stage2',
        executable='stage2_inertial_navigator',
        name='stage2_inertial_navigator',
        parameters=[
            stage2_config,  # 统一配置文件（包含原 inertial + avoid 参数）
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'cmd_topic': LaunchConfiguration('cmd_topic'),
                'task_topic': active_task_topic,
                'test_direction': LaunchConfiguration('test_direction'),
                'use_test_direction_fallback': LaunchConfiguration('enable_test_publisher'),
                'imu_map_yaw_offset_fallback_enabled': LaunchConfiguration('enable_test_publisher'),
                'standby': LaunchConfiguration('standby'),
            },
        ],
        output='log',
    )

    obstacle_circle_markers = Node(
        package='racing_stage2',
        executable='lidar_obstacle_circle_markers',
        name='lidar_obstacle_circle_markers',
        parameters=[obstacle_marker_config],
        output='log',
        condition=IfCondition(LaunchConfiguration('include_obstacle_markers')),
    )

    # Do not suppress RCUTILS: detailed startup diagnostics must remain
    # available on disk even though child stdout/stderr is terminal-isolated.
    session_root = os.environ.get('RACING_SESSION_ROOT', '').strip()
    runtime_ros_log_dir = SetEnvironmentVariable(
        'ROS_LOG_DIR', os.path.join(session_root, 'ros', 'stage2')
        if session_root else os.path.join(os.environ.get('DEV_WS', os.path.expanduser('~/dev_ws')), 'log', 'stage2', 'ros')
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
        include_obstacle_markers_arg,
        imu_topic_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        carto_slam_arg,
        standalone_map_overlay_arg,
        include_map_overlay_arg,
        enable_test_publisher_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        test_direction_arg,
        cmd_topic_arg,
        task_topic_arg,
        test_task_topic_arg,
        *support_actions,
        map_overlay_stack,
        test_publisher,
        stage2_navigator,
        obstacle_circle_markers,
    ])
