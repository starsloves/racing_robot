import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')
    stage2_launch_path = os.path.join(stage2_dir, 'launch', 'competition_stage2.launch.py')

    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_bno055_arg = DeclareLaunchArgument('include_bno055', default_value='true')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    include_obstacle_markers_arg = DeclareLaunchArgument('include_obstacle_markers', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/bno055/imu')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    bno055_i2c_bus_arg = DeclareLaunchArgument('bno055_i2c_bus', default_value='5')
    bno055_i2c_addr_arg = DeclareLaunchArgument('bno055_i2c_addr', default_value='41')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')

    return LaunchDescription([
        include_bringup_arg,
        include_lidar_arg,
        include_bno055_arg,
        include_camera_arg,
        include_depth_arg,
        include_obstacle_markers_arg,
        imu_topic_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        bno055_i2c_bus_arg,
        bno055_i2c_addr_arg,
        carto_slam_arg,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(stage2_launch_path),
            launch_arguments={
                'include_bringup': LaunchConfiguration('include_bringup'),
                'include_lidar': LaunchConfiguration('include_lidar'),
                'include_bno055': LaunchConfiguration('include_bno055'),
                'include_camera': LaunchConfiguration('include_camera'),
                'include_depth': LaunchConfiguration('include_depth'),
                'include_obstacle_markers': LaunchConfiguration('include_obstacle_markers'),
                'imu_topic': LaunchConfiguration('imu_topic'),
                'rgb_fps': LaunchConfiguration('rgb_fps'),
                'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
                'bno055_i2c_bus': LaunchConfiguration('bno055_i2c_bus'),
                'bno055_i2c_addr': LaunchConfiguration('bno055_i2c_addr'),
                'carto_slam': LaunchConfiguration('carto_slam'),
            }.items(),
        )
    ])
