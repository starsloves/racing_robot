import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')
    param_test_dir = get_package_share_directory('racing_stage2_param_test')

    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')

    include_support_arg = DeclareLaunchArgument('include_support', default_value='true')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='10')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')

    mask_threshold_arg = DeclareLaunchArgument('mask_threshold', default_value='0.7')
    roi_bottom_arg = DeclareLaunchArgument('roi_bottom', default_value='0.35')
    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/aurora/rgb/image_raw')

    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': 'false',
            'include_bno055': 'false',
            'include_camera': LaunchConfiguration('include_camera'),
            'include_depth': 'false',
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'carto_slam': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_support')),
    )

    preview_node = Node(
        package='racing_stage2_param_test',
        executable='vision_preview',
        name='vision_preview',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'mask_threshold': LaunchConfiguration('mask_threshold'),
            'roi_bottom': LaunchConfiguration('roi_bottom'),
        }],
        output='screen',
    )

    disable_shm = SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0')
    force_udp = SetEnvironmentVariable('RMW_FASTRTPS_TRANSPORT', 'UDPv4')

    return LaunchDescription([
        disable_shm,
        force_udp,

        include_support_arg,
        include_bringup_arg,
        include_camera_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,

        mask_threshold_arg,
        roi_bottom_arg,
        camera_topic_arg,

        support_stack,
        preview_node,
    ])
