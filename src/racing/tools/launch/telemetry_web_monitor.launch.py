from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='0.0.0.0'),
        DeclareLaunchArgument('port', default_value='8081'),
        Node(
            package='racing_tools',
            executable='telemetry_web_monitor',
            name='telemetry_web_monitor',
            output='screen',
            parameters=[{
                'host': LaunchConfiguration('host'),
                'port': LaunchConfiguration('port'),
                'map_heading_topic': 'map_heading',
                'heading_motion_linear_threshold_mps': 0.015,
                'heading_motion_angular_threshold_rad_s': 0.03,
            }],
        ),
    ])
