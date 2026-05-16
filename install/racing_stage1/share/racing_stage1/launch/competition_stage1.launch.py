import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bno055_dir = get_package_share_directory('bno055')
    lidar_dir = get_package_share_directory('lslidar_driver')
    qr_dir = get_package_share_directory('qr_scanner')
    stage1_dir = get_package_share_directory('racing_stage1')
    bringup_dir = get_package_share_directory('origincar_bringup')
    bno055_config_path = os.path.join(bno055_dir, 'config', 'bno055_params_i2c.yaml')
    lidar_launch_dir = os.path.join(lidar_dir, 'launch')
    qr_launch_dir = os.path.join(qr_dir, 'launch')
    stage1_config_dir = os.path.join(stage1_dir, 'config')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    stage2_cmd_topic_arg = DeclareLaunchArgument('stage2_cmd_topic', default_value='/stage2_cmd_vel')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_bno055_arg = DeclareLaunchArgument('include_bno055', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    bno055_i2c_bus_arg = DeclareLaunchArgument('bno055_i2c_bus', default_value='5')
    bno055_i2c_addr_arg = DeclareLaunchArgument('bno055_i2c_addr', default_value='41')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'origincar_bringup.launch.py')),
        launch_arguments={
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_bringup')),
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(qr_launch_dir, 'start_competition.launch.py')),
        launch_arguments={
            'device': LaunchConfiguration('device'),
            'include_camera': LaunchConfiguration('include_camera'),
            'include_depth': LaunchConfiguration('include_depth'),
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'lsn10_launch.py')),
        condition=IfCondition(LaunchConfiguration('include_lidar')),
    )

    bno055_node = Node(
        package='bno055',
        executable='bno055',
        name='bno055',
        parameters=[
            bno055_config_path,
            {
                'connection_type': 'i2c',
                'i2c_bus': LaunchConfiguration('bno055_i2c_bus'),
                'i2c_addr': LaunchConfiguration('bno055_i2c_addr'),
                'ros_topic_prefix': 'bno055/',
            },
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('include_bno055')),
    )

    controller_node = Node(
        package='racing_stage1',
        executable='competition_controller',
        name='competition_controller',
        parameters=[
            os.path.join(stage1_config_dir, 'stage1_controller.yaml'),
            {
                'stage2_cmd_topic': LaunchConfiguration('stage2_cmd_topic'),
                'imu_topic': LaunchConfiguration('imu_topic'),
            },
        ],
        output='screen',
    )

    return LaunchDescription([
        device_arg,
        include_camera_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        stage2_cmd_topic_arg,
        include_bringup_arg,
        include_lidar_arg,
        include_bno055_arg,
        imu_topic_arg,
        bno055_i2c_bus_arg,
        bno055_i2c_addr_arg,
        carto_slam_arg,
        bringup_launch,
        base_launch,
        lidar_launch,
        bno055_node,
        controller_node,
    ])