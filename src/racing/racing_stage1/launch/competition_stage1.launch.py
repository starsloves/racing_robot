import os
import math
import logging

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import launch.logging as launch_logging
from racing_common.launch_status import startup_status


def _emergency_stop_action():
    """Ctrl+C 时只发零速度；子进程由 launch 负责正常回收。"""
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'set +e; '
                'timeout --signal=TERM 1s ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" '
                '>/dev/null 2>&1 || true'
            ),
        ],
        output='log',
    )


def _stage1_map_to_odom_defaults(config_path):
    """Read the Stage1-owned map-to-odom transform from its ROS parameter YAML."""
    with open(config_path, 'r', encoding='utf-8') as config_file:
        params = yaml.safe_load(config_file)['competition_controller']['ros__parameters']
    return (
        str(params['map_to_odom_x']),
        str(params['map_to_odom_y']),
        str(math.radians(float(params['imu_initial_map_yaw_deg']))),
    )


def generate_launch_description():
    launch_logging.launch_config.level = logging.ERROR
    bno055_dir = get_package_share_directory('bno055')
    lidar_dir = get_package_share_directory('lslidar_driver')
    qr_dir = get_package_share_directory('qr_scanner')
    stage1_dir = get_package_share_directory('racing_stage1')
    bringup_dir = get_package_share_directory('origincar_bringup')
    bno055_config_path = os.path.join(bno055_dir, 'config', 'bno055_params_i2c.yaml')
    lidar_launch_dir = os.path.join(lidar_dir, 'launch')
    qr_launch_dir = os.path.join(qr_dir, 'launch')
    stage1_config_dir = os.path.join(stage1_dir, 'config')
    stage1_config_path = os.path.join(stage1_config_dir, 'stage1_controller.yaml')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')
    (
        map_to_odom_x_default,
        map_to_odom_y_default,
        map_to_odom_yaw_default,
    ) = _stage1_map_to_odom_defaults(stage1_config_path)

    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    stage2_cmd_topic_arg = DeclareLaunchArgument('stage2_cmd_topic', default_value='/stage2_cmd_vel')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_bno055_arg = DeclareLaunchArgument('include_bno055', default_value='false')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    bno055_i2c_bus_arg = DeclareLaunchArgument('bno055_i2c_bus', default_value='5')
    bno055_i2c_addr_arg = DeclareLaunchArgument('bno055_i2c_addr', default_value='41')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    standalone_map_overlay_arg = DeclareLaunchArgument(
        'standalone_map_overlay',
        default_value='false',
        description='Include map server and map→odom TF for standalone testing'
    )
    include_map_overlay_arg = DeclareLaunchArgument(
        'include_map_overlay',
        default_value='false',
        description='Deprecated compatibility argument; use standalone_map_overlay'
    )
    enable_test_publisher_arg = DeclareLaunchArgument(
        'enable_test_publisher',
        default_value='false',
        description='Publish fixed phase and direction topics for standalone testing'
    )
    map_to_odom_x_arg = DeclareLaunchArgument(
        'map_to_odom_x',
        default_value=map_to_odom_x_default,
        description='Stage1 start X position in map frame'
    )
    map_to_odom_y_arg = DeclareLaunchArgument(
        'map_to_odom_y',
        default_value=map_to_odom_y_default,
        description='Stage1 start Y position in map frame'
    )
    map_to_odom_yaw_arg = DeclareLaunchArgument(
        'map_to_odom_yaw',
        default_value=map_to_odom_yaw_default,
        description='Stage1 start yaw in map frame (rad, derived from imu_initial_map_yaw_deg)'
    )
    test_direction_arg = DeclareLaunchArgument(
        'test_direction',
        default_value='clockwise',
        description='Test direction for standalone testing (clockwise/counterclockwise)'
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'origincar_bringup.launch.py')),
        launch_arguments={
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_bringup')),
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
        package='racing_stage1',
        executable='stage_test_publisher_fixed',
        name='stage_test_publisher',
        parameters=[{
            'stage_number': 1,
            'test_direction': LaunchConfiguration('test_direction'),
            'publish_phase': False,
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('enable_test_publisher')),
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(qr_launch_dir, 'start_competition.launch.py')),
        launch_arguments={
            'device': LaunchConfiguration('device'),
            'include_camera': LaunchConfiguration('include_camera'),
            'include_depth': LaunchConfiguration('include_depth'),
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'lsn10_launch.py')),
        condition=IfCondition(LaunchConfiguration('include_lidar')),
    )

    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[
            bno055_config_path,
            {
                'connection_type': 'i2c',
                'i2c_bus': LaunchConfiguration('bno055_i2c_bus'),
                'i2c_addr': LaunchConfiguration('bno055_i2c_addr'),
                'ros_topic_prefix': 'bno055/',
            },
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('include_bno055')),
    )

    controller_node = Node(
        package='racing_stage1',
        executable='competition_controller',
        name='competition_controller',
        parameters=[
            stage1_config_path,  # 统一配置文件
            {
                'stage2_cmd_topic': LaunchConfiguration('stage2_cmd_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            },
        ],
        output='log',
    )

    # Process output remains off the terminal, while ROS keeps complete
    # startup and failure diagnostics in this stage's runtime log directory.
    runtime_ros_log_dir = SetEnvironmentVariable(
        'ROS_LOG_DIR', '/home/sunrise/dev_ws/log/competition_stage1/ros'
    )
    isolate_process_output = SetEnvironmentVariable(
        'OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'own_log'
    )

    return LaunchDescription([
        runtime_ros_log_dir,
        isolate_process_output,
        device_arg,
        include_camera_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        stage2_cmd_topic_arg,
        include_bringup_arg,
        include_lidar_arg,
        include_bno055_arg,
        imu_topic_arg,
        bno055_i2c_bus_arg,
        bno055_i2c_addr_arg,
        carto_slam_arg,
        standalone_map_overlay_arg,
        include_map_overlay_arg,
        enable_test_publisher_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        test_direction_arg,
        bringup_launch,
        map_overlay_stack,
        test_publisher,
        base_launch,
        lidar_launch,
        bno055_node,
        controller_node,
        RegisterEventHandler(OnProcessStart(
            target_action=controller_node,
            on_start=[startup_status('第一阶段控制器', '/competition_controller')],
        )),
        # 注册 Ctrl+C 时的紧急停车处理器
        RegisterEventHandler(
            OnShutdown(on_shutdown=[_emergency_stop_action()]),
        ),
    ])
