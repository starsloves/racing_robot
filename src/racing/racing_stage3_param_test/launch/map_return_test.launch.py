"""Stage3 地图导航 — 独立测试启动
- A* 全局路径规划（/map）+ Pure Pursuit + 4态避障
- 模拟 Stage2 结束状态，延时后自动触发 phase=3
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
    config = os.path.join(stage3_dir, 'config', 'map_return.yaml')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('test_direction', default_value='clockwise'),
        DeclareLaunchArgument('auto_start_phase3', default_value='true'),
        DeclareLaunchArgument('phase3_start_delay_sec', default_value='5.0'),
        DeclareLaunchArgument('include_bringup', default_value='true'),
        DeclareLaunchArgument('include_lidar', default_value='true'),
        DeclareLaunchArgument('include_bno055', default_value='true'),
        DeclareLaunchArgument('include_carto', default_value='true'),

        # 硬件驱动（含 SLAM Toolbox → /map）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(support_launch_path),
            launch_arguments={
                'include_bringup': LaunchConfiguration('include_bringup'),
                'include_lidar': LaunchConfiguration('include_lidar'),
                'include_bno055': LaunchConfiguration('include_bno055'),
                'include_camera': 'false',
                'include_depth': 'false',
                'carto_slam': LaunchConfiguration('include_carto'),
            }.items(),
        ),

        # 地图导航返程节点
        Node(
            package='racing_stage3_param_test',
            executable='map_return_navigator',
            name='map_return_navigator',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),

        # 延时后自动触发 phase=3
        TimerAction(
            period=LaunchConfiguration('phase3_start_delay_sec'),
            actions=[
                Node(
                    package='racing_stage3_param_test',
                    executable='stage3_test_simulator',
                    name='stage3_phase3_trigger',
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