import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_bringup')
    lidar_dir = get_package_share_directory('lslidar_driver')
    default_map_yaml = os.path.join(bringup_dir, 'map', 'map_restricted.yaml')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'origincar_bringup.launch.py')
        ),
        launch_arguments={'carto_slam': 'false'}.items(),
        condition=IfCondition(LaunchConfiguration('include_base')),
    )
    map_overlay = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')
        ),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'odom_frame': LaunchConfiguration('odom_frame'),
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_map_overlay')),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lsn10_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('include_lidar')),
    )
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
        DeclareLaunchArgument('include_base', default_value='true'),
        DeclareLaunchArgument('include_map_overlay', default_value='true'),
        DeclareLaunchArgument('include_lidar', default_value='true'),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml),
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_footprint'),
        DeclareLaunchArgument('odom_frame', default_value='odom_combined'),
        DeclareLaunchArgument('odom_topic', default_value='/odom_combined'),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.50'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.15'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.3490658504'),
        DeclareLaunchArgument('record_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('sample_distance_m', default_value='0.03'),
        DeclareLaunchArgument('output_dir', default_value='log/manual_trajectories'),
        DeclareLaunchArgument('record_name', default_value='manual_trajectory'),
        DeclareLaunchArgument('tf_timeout_sec', default_value='0.05'),
        base,
        map_overlay,
        lidar,
        recorder,
    ])
