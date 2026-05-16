import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    qr_dir = get_package_share_directory('qr_scanner')
    launch_dir = os.path.join(qr_dir, 'launch')
    config_dir = os.path.join(qr_dir, 'config')

    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(launch_dir, 'start_competition.launch.py')),
        launch_arguments={
            'device': LaunchConfiguration('device'),
            'stage2_cmd_topic': '/stage2_cmd_vel',
        }.items(),
    )

    stage2_nav_node = Node(
        package='qr_scanner',
        executable='stage2_inertial_navigator',
        name='stage2_inertial_navigator',
        parameters=[os.path.join(config_dir, 'inertial_stage2.yaml')],
        output='screen',
    )

    person_detection_node = Node(
        package='qr_scanner',
        executable='person_detection_node',
        name='person_detection_node',
        parameters=[os.path.join(config_dir, 'person_detection.yaml')],
        output='screen',
    )

    image_caption_node = Node(
        package='qr_scanner',
        executable='image_caption_node',
        name='image_caption_node',
        parameters=[os.path.join(config_dir, 'image_caption.yaml')],
        output='screen',
    )

    return LaunchDescription([
        device_arg,
        base_launch,
        stage2_nav_node,
        person_detection_node,
        image_caption_node,
    ])