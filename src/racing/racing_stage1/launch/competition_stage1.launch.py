import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def _emergency_stop_action():
    """紧急停车处理器：Ctrl+C 时立即停车并清理进程"""
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'set +e; '
                # 先停车
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" '
                '2>/dev/null & '
                # 杀掉相关进程
                'pkill -15 -f lslidar_driver_node 2>/dev/null; '
                'sleep 0.3; '
                'pkill -9 -f lslidar_driver_node 2>/dev/null; '
                'pkill -9 -f competition_controller 2>/dev/null; '
                'pkill -9 -f qr_scanner 2>/dev/null; '
                'pkill -9 -f stage_test_publisher 2>/dev/null; '
                'pkill -9 -f origincar 2>/dev/null; '
                'true'
            ),
        ],
        output='log',
    )


def _stage1_map_to_odom_defaults(config_path):
    """Read the Stage1-owned static TF translation from its ROS parameter YAML."""
    with open(config_path, 'r', encoding='utf-8') as config_file:
        params = yaml.safe_load(config_file)['competition_controller']['ros__parameters']
    return str(params['map_to_odom_x']), str(params['map_to_odom_y'])


def generate_launch_description():
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
    map_to_odom_x_default, map_to_odom_y_default = _stage1_map_to_odom_defaults(stage1_config_path)

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
        default_value='0.1745329252',
        description='Stage1 start yaw in map frame (rad, default ~10°)'
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
        output='screen',
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
        output='screen',
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
        output='screen',
    )

    return LaunchDescription([
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
        # 注册 Ctrl+C 时的紧急停车处理器
        RegisterEventHandler(
            OnShutdown(on_shutdown=[_emergency_stop_action()]),
        ),
    ])
