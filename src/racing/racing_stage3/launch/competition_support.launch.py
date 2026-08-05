import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    lidar_dir = get_package_share_directory('lslidar_driver')
    bringup_dir = get_package_share_directory('origincar_bringup')

    lidar_launch_path = os.path.join(lidar_dir, 'launch', 'lsn10_launch.py')
    bringup_launch_path = os.path.join(bringup_dir, 'launch', 'origincar_bringup.launch.py')

    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='true')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(bringup_launch_path),
        launch_arguments={
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_bringup')),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(lidar_launch_path),
        condition=IfCondition(LaunchConfiguration('include_lidar')),
    )

    aurora_node = Node(
        package='deptrum-ros-driver-aurora930',
        executable='aurora930_node',
        namespace='aurora',
        parameters=[{
            'rgb_enable': True,
            'ir_enable': False,
            'depth_enable': LaunchConfiguration('include_depth'),
            'rgbd_enable': False,
            'point_cloud_enable': False,
            'boot_order': 1,
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'align_mode': LaunchConfiguration('include_depth'),
            'log_dir': '/tmp/',
            'stream_sdk_log_enable': False,
            'heart_enable': False,
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('include_camera')),
    )

    return LaunchDescription([
        include_bringup_arg,
        include_lidar_arg,
        include_camera_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        carto_slam_arg,
        bringup_launch,
        lidar_launch,
        aurora_node,
    ])
