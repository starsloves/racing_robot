"""Stage3 增强返程 — 独立测试启动
        - 模拟 Stage2 结束状态（车在 (2.80, 3.25) map 坐标）
- 启动 EnhancedReturnNavigator 执行 Pure Pursuit + 避障
- 同 Stage2 的 map_overlay 提供 map→odom TF
- 先启动各驱动，延时后自动触发 phase=3
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
    stage3_dir = get_package_share_directory('racing_stage3_param_test')
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
        DeclareLaunchArgument('use_relay', default_value='false'),
        DeclareLaunchArgument('cmd_topic', default_value='/cmd_vel'),

        # 硬件驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(support_launch_path),
            launch_arguments={
                'include_bringup': LaunchConfiguration('include_bringup'),
                'include_lidar': LaunchConfiguration('include_lidar'),
                'include_bno055': LaunchConfiguration('include_bno055'),
                'include_camera': 'false',
                'include_depth': 'false',
                'carto_slam': 'false',
            }.items(),
        ),

        # map_overlay：提供 map → odom_combined TF（同 Stage2 direct_inertial_test）
        # Stage2 结束位置: map (2.80, 3.25)
        # map_to_odom_yaw=180°: 顺时针结束朝 -X/180° → 传入导航器同参
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_overlay_launch_path),
            launch_arguments={
                'map_to_odom_x': '2.80',
                'map_to_odom_y': '3.25',
                'map_to_odom_yaw': '3.14159',  # 180°
                'odom_frame': 'odom_combined',
            }.items(),
        ),

        # 增强返程导航器（map 坐标系路点，传 map_to_odom 参数直接计算偏移）
        Node(
            package='racing_stage3_param_test',
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

        # 可选：速度中继
        Node(
            package='racing_stage3_param_test',
            executable='twist_cmd_relay',
            name='stage3_test_cmd_relay',
            parameters=[{
                'input_topic': LaunchConfiguration('cmd_topic'),
                'output_topic': '/cmd_vel',
            }],
            output='log',
            condition=IfCondition(LaunchConfiguration('use_relay')),
        ),

        # 延时后自动触发 phase=3 + 模拟 QR 方向
        TimerAction(
            period=LaunchConfiguration('phase3_start_delay_sec'),
            actions=[
                Node(
                    package='racing_stage3_param_test',
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