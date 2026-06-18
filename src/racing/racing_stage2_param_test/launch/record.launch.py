"""record.launch.py — 单独启动数据记录器"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='racing_stage2_param_test',
            executable='data_recorder',
            name='data_recorder',
            parameters=[{
                'record_rate_hz': 20.0,
                'record_subdir': 'data_records',
                'record_filename': 'latest_record.csv',
            }],
            output='screen',
        ),
    ])
