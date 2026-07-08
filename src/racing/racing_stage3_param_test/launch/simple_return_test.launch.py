import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stage3_dir = get_package_share_directory('racing_stage3_param_test')
    config = os.path.join(stage3_dir, 'config', 'simple_return.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('test_direction', default_value='clockwise'),
        DeclareLaunchArgument('auto_start_phase3', default_value='true'),
        DeclareLaunchArgument('phase3_start_delay_sec', default_value='3.0'),

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