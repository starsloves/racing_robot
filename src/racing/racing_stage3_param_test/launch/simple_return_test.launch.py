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
    config = os.path.join(stage3_dir, 'config', 'simple_return.yaml')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('test_direction', default_value='clockwise'),
        DeclareLaunchArgument('auto_start_phase3', default_value='true'),
        DeclareLaunchArgument('phase3_start_delay_sec', default_value='3.0'),
        DeclareLaunchArgument('include_bringup', default_value='true'),
        DeclareLaunchArgument('include_lidar', default_value='true'),
        DeclareLaunchArgument('include_bno055', default_value='false'),

        # 启动底盘、IMU、激光雷达等驱动
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

        Node(
            package='racing_stage3_param_test',
            executable='simple_return_navigator',
            name='simple_return_navigator',
            parameters=[config, {
                'test_direction': LaunchConfiguration('test_direction'),
            }],
            output='screen',
            emulate_tty=True,
        ),

        Node(
            package='racing_stage3_param_test',
            executable='twist_cmd_relay',
            name='stage3_test_cmd_relay',
            parameters=[{
                'input_topic': '/stage3_cmd_vel',
                'output_topic': '/cmd_vel',
            }],
            output='log',
        ),

        TimerAction(
            period=LaunchConfiguration('phase3_start_delay_sec'),
            actions=[
                Node(
                    package='racing_stage3_param_test',
                    executable='phase3_test_trigger',
                    name='phase3_test_trigger',
                    output='screen',
                    condition=IfCondition(LaunchConfiguration('auto_start_phase3')),
                ),
            ],
        ),
    ])