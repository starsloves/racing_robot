import json
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
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')
    return_config = os.path.join(stage3_dir, 'config', 'return_stage3.yaml')

    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='false')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    auto_start_phase3_arg = DeclareLaunchArgument('auto_start_phase3', default_value='true')
    phase3_start_delay_arg = DeclareLaunchArgument('phase3_start_delay_sec', default_value='3.0')

    # 起点参数（JSON 格式，类似 Stage2 corridor_waypoints_json）
    # 默认 = Stage2 顺时针整圈终点 (2.38, 3.32) @ 180°
    # 空 = 从 /odom_combined 读取 phase=3 时的实际位姿
    start_json_arg = DeclareLaunchArgument(
        'start_json',
        default_value='[{"x":2.38,"y":3.32,"speed":0.12,"yaw_deg":180.0,"description":"mission_start"}]',
    )

    # 终点参数（默认 P 点 (0.20, 0.20) @ 100°）
    goal_json_arg = DeclareLaunchArgument(
        'goal_json',
        default_value='[{"x":0.20,"y":0.20,"speed":0.10,"yaw_deg":100.0,"description":"p_point"}]',
    )

    # 可选中间路点（空 = 纯 A* 规划）
    waypoints_json_arg = DeclareLaunchArgument('waypoints_json', default_value='[]')

    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': 'true',
            'include_camera': LaunchConfiguration('include_camera'),
            'include_depth': 'false',
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
    )

    stage3_return_navigator = Node(
        package='racing_stage3_param_test',
        executable='stage3_return_navigator',
        name='stage3_return_navigator',
        parameters=[
            return_config,
            {
                'return_start_json': LaunchConfiguration('start_json'),
                'return_goal_json': LaunchConfiguration('goal_json'),
                'return_waypoints_json': LaunchConfiguration('waypoints_json'),
            },
        ],
        output='screen',
    )

    phase3_trigger = Node(
        package='racing_stage3_param_test',
        executable='phase3_test_trigger',
        name='phase3_test_trigger',
        output='screen',
        condition=IfCondition(LaunchConfiguration('auto_start_phase3')),
    )

    return LaunchDescription([
        include_bringup_arg,
        include_lidar_arg,
        include_camera_arg,
        carto_slam_arg,
        auto_start_phase3_arg,
        phase3_start_delay_arg,
        start_json_arg,
        goal_json_arg,
        waypoints_json_arg,
        support_stack,
        stage3_return_navigator,
        TimerAction(
            period=LaunchConfiguration('phase3_start_delay_sec'),
            actions=[phase3_trigger],
        ),
    ])