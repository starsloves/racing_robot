import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')

    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')
    inertial_config = os.path.join(stage2_dir, 'config', 'inertial_stage2.yaml')

    include_support_arg = DeclareLaunchArgument('include_support', default_value='true')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    # 无 Stage1：假定已在 corridor_goal；IMU 对齐 channel_entry 90°（map 2.50, 3.20）。
    assume_channel_entry_yaw_arg = DeclareLaunchArgument(
        'assume_channel_entry_yaw',
        default_value='true',
    )
    field_track_config_arg = DeclareLaunchArgument(
        'field_track_config',
        default_value='',
        description='空=按 test_direction 加载包内 config/field_track_*.yaml',
    )
    enable_cmd_relay_arg = DeclareLaunchArgument('enable_cmd_relay', default_value='true')
    relay_input_topic_arg = DeclareLaunchArgument('relay_input_topic', default_value='/stage2_cmd_vel')
    relay_output_topic_arg = DeclareLaunchArgument('relay_output_topic', default_value='/cmd_vel')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    detour_detect_distance_arg = DeclareLaunchArgument('detour_obstacle_detect_distance', default_value='1.30')
    detour_clear_distance_arg = DeclareLaunchArgument('detour_obstacle_clear_distance', default_value='0.75')
    avoid_watch_distance_arg = DeclareLaunchArgument('avoid_watch_distance_m', default_value='0.80')
    avoid_commit_distance_arg = DeclareLaunchArgument('avoid_commit_distance_m', default_value='0.45')
    avoid_corner_prefer_inside_arg = DeclareLaunchArgument('avoid_corner_prefer_inside', default_value='true')

    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': 'false',
            'include_camera': LaunchConfiguration('include_camera'),
            'include_depth': LaunchConfiguration('include_depth'),
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_support')),
    )

    tester_node = Node(
        package='racing_stage2_param_test',
        executable='direct_inertial_tester',
        name='stage2_inertial_navigator',
        parameters=[
            inertial_config,
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'test_direction': LaunchConfiguration('test_direction'),
                'assume_channel_entry_yaw': LaunchConfiguration('assume_channel_entry_yaw'),
                'field_track_config': LaunchConfiguration('field_track_config'),
                'pre_loop_plan_json': '[]',
                'corridor_path_skip_pre_loop_plan': True,
                'detour_obstacle_detect_distance': LaunchConfiguration('detour_obstacle_detect_distance'),
                'detour_obstacle_clear_distance': LaunchConfiguration('detour_obstacle_clear_distance'),
                'avoid_watch_distance_m': LaunchConfiguration('avoid_watch_distance_m'),
                'avoid_commit_distance_m': LaunchConfiguration('avoid_commit_distance_m'),
                'avoid_corner_prefer_inside': LaunchConfiguration('avoid_corner_prefer_inside'),
            },
        ],
        output='screen',
    )

    cmd_relay_node = Node(
        package='racing_stage2_param_test',
        executable='twist_cmd_relay',
        name='stage2_test_cmd_relay',
        parameters=[{
            'input_topic': LaunchConfiguration('relay_input_topic'),
            'output_topic': LaunchConfiguration('relay_output_topic'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cmd_relay')),
    )

    return LaunchDescription([
        include_support_arg,
        include_bringup_arg,
        include_lidar_arg,
        include_camera_arg,
        include_depth_arg,
        imu_topic_arg,
        test_direction_arg,
        assume_channel_entry_yaw_arg,
        field_track_config_arg,
        enable_cmd_relay_arg,
        relay_input_topic_arg,
        relay_output_topic_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        carto_slam_arg,
        detour_detect_distance_arg,
        detour_clear_distance_arg,
        avoid_watch_distance_arg,
        avoid_commit_distance_arg,
        avoid_corner_prefer_inside_arg,
        support_stack,
        cmd_relay_node,
        tester_node,
    ])