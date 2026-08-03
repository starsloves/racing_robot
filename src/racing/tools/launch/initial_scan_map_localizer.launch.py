import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tools_dir = get_package_share_directory('racing_tools')
    bringup_dir = get_package_share_directory('origincar_bringup')
    lidar_dir = get_package_share_directory('lslidar_driver')
    default_config = os.path.join(tools_dir, 'config', 'initial_scan_map_localizer.yaml')
    default_map_yaml = os.path.join(bringup_dir, 'map', 'map_restricted.yaml')

    map_overlay_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')
        ),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            'odom_frame': LaunchConfiguration('odom_frame'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_map_overlay')),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(lidar_dir, 'launch', 'lsn10_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('include_lidar')),
    )

    localizer_node = Node(
        package='racing_tools',
        executable='initial_scan_map_localizer',
        name='initial_scan_map_localizer',
        output='log',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument(
            'include_map_overlay',
            default_value='true',
            description='Start /map map_server and static map->odom TF for standalone scan-map localization',
        ),
        DeclareLaunchArgument(
            'include_lidar',
            default_value='true',
            description='Start lslidar_driver lsn10 launch to provide /scan',
        ),
        DeclareLaunchArgument('map_yaml', default_value=default_map_yaml),
        DeclareLaunchArgument('odom_frame', default_value='odom_combined'),
        DeclareLaunchArgument('map_to_odom_x', default_value='0.50'),
        DeclareLaunchArgument('map_to_odom_y', default_value='0.20'),
        DeclareLaunchArgument('map_to_odom_yaw', default_value='0.1745329252'),
        DeclareLaunchArgument(
            'localizer_start_delay_sec',
            default_value='3.0',
            description='Delay localizer startup so map_server/lidar can become ready first',
        ),
        map_overlay_launch,
        lidar_launch,
        TimerAction(
            period=LaunchConfiguration('localizer_start_delay_sec'),
            actions=[localizer_node],
        ),
    ])
