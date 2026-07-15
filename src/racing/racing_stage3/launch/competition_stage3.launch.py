import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stage3_dir = get_package_share_directory('racing_stage3')
    bringup_dir = get_package_share_directory('origincar_bringup')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')
    return_config = os.path.join(stage3_dir, 'config', 'enhanced_return.yaml')

    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_bno055_arg = DeclareLaunchArgument('include_bno055', default_value='false')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='false')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    bno055_i2c_bus_arg = DeclareLaunchArgument('bno055_i2c_bus', default_value='5')
    bno055_i2c_addr_arg = DeclareLaunchArgument('bno055_i2c_addr', default_value='41')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    include_map_overlay_arg = DeclareLaunchArgument(
        'include_map_overlay',
        default_value='false',
        description='Include map server and map→odom TF for standalone testing'
    )
    map_to_odom_x_arg = DeclareLaunchArgument(
        'map_to_odom_x',
        default_value='2.38',
        description='Stage3 start X position in map frame (rect loop end point)'
    )
    map_to_odom_y_arg = DeclareLaunchArgument(
        'map_to_odom_y',
        default_value='3.32',
        description='Stage3 start Y position in map frame (rect loop end point)'
    )
    map_to_odom_yaw_arg = DeclareLaunchArgument(
        'map_to_odom_yaw',
        default_value='3.1415926536',
        description='Stage3 start yaw in map frame (rad, 180° for clockwise)'
    )

    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': LaunchConfiguration('include_bno055'),
            'include_camera': LaunchConfiguration('include_camera'),
            'include_depth': LaunchConfiguration('include_depth'),
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'bno055_i2c_bus': LaunchConfiguration('bno055_i2c_bus'),
            'bno055_i2c_addr': LaunchConfiguration('bno055_i2c_addr'),
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
    )

    map_overlay_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_overlay_launch_path),
        launch_arguments={
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            'odom_frame': 'odom_combined',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_map_overlay')),
    )

    test_publisher = Node(
        package='racing_stage3',
        executable='stage_test_publisher',
        name='stage_test_publisher',
        parameters=[{
            'stage_number': 3,
            'test_direction': LaunchConfiguration('test_direction'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('include_map_overlay')),
    )

    stage3_return_navigator = Node(
        package='racing_stage3',
        executable='enhanced_return_navigator',
        name='enhanced_return_navigator',
        parameters=[return_config, {
            'test_direction': LaunchConfiguration('test_direction'),
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            'use_global_planner': False,
        }],
        output='screen',
    )

    return LaunchDescription([
        include_bringup_arg,
        include_lidar_arg,
        include_bno055_arg,
        include_camera_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        bno055_i2c_bus_arg,
        bno055_i2c_addr_arg,
        carto_slam_arg,
        test_direction_arg,
        include_map_overlay_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        support_stack,
        map_overlay_stack,
        test_publisher,
        stage3_return_navigator,
    ])
