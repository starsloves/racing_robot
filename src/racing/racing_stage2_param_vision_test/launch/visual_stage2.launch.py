#!/usr/bin/env python3
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _emergency_stop_action():
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'set +e; '
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null & '
                'pkill -9 -f aurora930 2>/dev/null; '
                'pkill -9 -f visual_stage2_navigator 2>/dev/null; '
                'pkill -9 -f simple_visual_odometry 2>/dev/null; '
                'pkill -9 -f lslidar 2>/dev/null; '
                'pkill -9 -f origincar 2>/dev/null; '
                'true'
            ),
        ],
        output='log',
    )


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')
    param_vision_test_dir = get_package_share_directory('racing_stage2_param_vision_test')
    bringup_dir = get_package_share_directory('origincar_bringup')

    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')

    # Launch 参数
    include_support_arg = DeclareLaunchArgument('include_support', default_value='true')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='false')
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')

    # Support stack（底盘、IMU、EKF）
    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': 'false',
            'include_camera': 'false',
            'include_depth': 'false',
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'carto_slam': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_support')),
    )

    # map_overlay（提供 /map 和 map → odom_combined TF）
    map_overlay_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_overlay_launch_path),
        launch_arguments={
            'map_to_odom_x': '2.50',
            'map_to_odom_y': '2.80',
            'map_to_odom_yaw': '1.5708',  # 90° = π/2 rad
            'odom_frame': 'odom_combined',
        }.items(),
    )

    # Aurora 930 相机节点
    aurora_node = Node(
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
    )

    # 简化视觉里程计（坐标转换）
    visual_odom_node = Node(
        package='racing_stage2_param_vision_test',
        executable='simple_visual_odometry',
        name='simple_visual_odometry',
        output='screen',
    )

    # 视觉导航器（集成车道检测）
    visual_navigator_node = Node(
        package='racing_stage2_param_vision_test',
        executable='visual_stage2_navigator',
        name='visual_stage2_navigator',
        parameters=[{
            'direction': LaunchConfiguration('test_direction'),
            'auto_start': True,
        }],
        output='screen',
    )

    # 禁用 Fast DDS 共享内存
    disable_shm = SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0')
    force_udp = SetEnvironmentVariable('RMW_FASTRTPS_TRANSPORT', 'UDPv4')

    # 重定向 ROS 日志
    ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR',
        os.path.join(os.path.expanduser('~'), 'dev_ws', 'log', 'ros'))

    return LaunchDescription([
        disable_shm,
        force_udp,
        ros_log_dir,

        include_support_arg,
        include_bringup_arg,
        include_lidar_arg,
        test_direction_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,

        # 启动顺序：相机 → 支持栈 → map → 视觉节点
        aurora_node,
        support_stack,
        map_overlay_stack,
        visual_odom_node,
        visual_navigator_node,

        RegisterEventHandler(
            OnShutdown(on_shutdown=[_emergency_stop_action()]),
        ),
    ])
