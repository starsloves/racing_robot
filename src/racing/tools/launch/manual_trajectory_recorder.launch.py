from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    recorder = Node(
        package='racing_tools',
        executable='manual_trajectory_recorder',
        name='manual_trajectory_recorder',
        output='screen',
        parameters=[{
            'map_frame': LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'record_rate_hz': LaunchConfiguration('record_rate_hz'),
            'sample_distance_m': LaunchConfiguration('sample_distance_m'),
            'output_dir': LaunchConfiguration('output_dir'),
            'record_name': LaunchConfiguration('record_name'),
            'tf_timeout_sec': LaunchConfiguration('tf_timeout_sec'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_combined'),
        DeclareLaunchArgument('record_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('sample_distance_m', default_value='0.03'),
        DeclareLaunchArgument(
            'output_dir', default_value='log/manual_trajectories',
            description='Relative to workspace unless an absolute path is supplied',
        ),
        DeclareLaunchArgument('record_name', default_value='manual_trajectory'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.05'),
        recorder,
    ])
