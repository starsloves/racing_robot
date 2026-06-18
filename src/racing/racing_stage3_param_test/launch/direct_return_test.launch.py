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
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    return_track_config_arg = DeclareLaunchArgument('return_track_config', default_value='')
    auto_start_phase3_arg = DeclareLaunchArgument('auto_start_phase3', default_value='true')
    phase3_start_delay_arg = DeclareLaunchArgument('phase3_start_delay_sec', default_value='3.0')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')

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
                'test_direction': LaunchConfiguration('test_direction'),
                'return_track_config': LaunchConfiguration('return_track_config'),
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
        test_direction_arg,
        return_track_config_arg,
        auto_start_phase3_arg,
        phase3_start_delay_arg,
        carto_slam_arg,
        support_stack,
        stage3_return_navigator,
        TimerAction(
            period=LaunchConfiguration('phase3_start_delay_sec'),
            actions=[phase3_trigger],
        ),
    ])
