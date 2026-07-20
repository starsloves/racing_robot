import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _stage1_map_to_odom_defaults(config_path):
    """Read the Stage1-owned static TF translation from its ROS parameter YAML."""
    with open(config_path, 'r', encoding='utf-8') as config_file:
        params = yaml.safe_load(config_file)['competition_controller']['ros__parameters']
    return str(params['map_to_odom_x']), str(params['map_to_odom_y'])


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_bringup')
    stage1_dir = get_package_share_directory('racing_stage1')
    stage2_dir = get_package_share_directory('racing_stage2')
    stage3_dir = get_package_share_directory('racing_stage3')
    vision_ai_dir = get_package_share_directory('racing_vision_ai')
    voice_driver_dir = get_package_share_directory('voice_driver')

    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')
    stage1_config_path = os.path.join(stage1_dir, 'config', 'stage1_controller.yaml')
    map_to_odom_x_default, map_to_odom_y_default = _stage1_map_to_odom_defaults(stage1_config_path)
    stage1_launch_path = os.path.join(stage1_dir, 'launch', 'competition_stage1.launch.py')
    stage2_launch_path = os.path.join(stage2_dir, 'launch', 'competition_stage2.launch.py')
    stage3_launch_path = os.path.join(stage3_dir, 'launch', 'competition_stage3.launch.py')

    include_map_overlay_arg = DeclareLaunchArgument('include_map_overlay', default_value='true')
    include_stage1_arg = DeclareLaunchArgument('include_stage1', default_value='true')
    include_stage2_arg = DeclareLaunchArgument('include_stage2', default_value='true')
    include_stage3_arg = DeclareLaunchArgument('include_stage3', default_value='true')
    include_vision_ai_arg = DeclareLaunchArgument('include_vision_ai', default_value='true')
    include_voice_arg = DeclareLaunchArgument('include_voice', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    include_bno055_arg = DeclareLaunchArgument('include_bno055', default_value='false')
    include_obstacle_markers_arg = DeclareLaunchArgument('include_obstacle_markers', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    map_yaml_arg = DeclareLaunchArgument(
        'map_yaml',
        default_value=os.path.join(bringup_dir, 'map', 'map_restricted.yaml'),
    )
    map_to_odom_x_arg = DeclareLaunchArgument('map_to_odom_x', default_value=map_to_odom_x_default)
    map_to_odom_y_arg = DeclareLaunchArgument('map_to_odom_y', default_value=map_to_odom_y_default)
    map_to_odom_yaw_arg = DeclareLaunchArgument('map_to_odom_yaw', default_value='0.1745329252')

    map_overlay_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_overlay_launch_path),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'odom_frame': 'odom_combined',
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
        }.items(),  
        condition=IfCondition(LaunchConfiguration('include_map_overlay')),
    )

    stage1_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage1_launch_path),
        launch_arguments={
            'include_depth': LaunchConfiguration('include_depth'),
            'include_bno055': LaunchConfiguration('include_bno055'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'standalone_map_overlay': 'false',
            'include_map_overlay': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_stage1')),
    )

    stage2_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage2_launch_path),
        launch_arguments={
            'include_bringup': 'false',
            'include_lidar': 'false',
            'include_bno055': 'false',
            'include_camera': 'false',
            'include_depth': 'false',
            'include_obstacle_markers': LaunchConfiguration('include_obstacle_markers'),
            'imu_topic': LaunchConfiguration('imu_topic'),
            'standalone_map_overlay': 'false',
            'include_map_overlay': 'false',
            'enable_test_publisher': 'false',
            'phase_topic': 'competition_phase',
            'task_topic': 'competition_qr_task',
            'enable_cmd_relay': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_stage2')),
    )

    stage3_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(stage3_launch_path),
        launch_arguments={
            'include_bringup': 'false',
            'include_lidar': 'false',
            'include_bno055': 'false',
            'include_camera': 'false',
            'test_direction': LaunchConfiguration('test_direction'),
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            'standalone_map_overlay': 'false',
            'include_map_overlay': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_stage3')),
    )

    vision_ai_node = Node(
        package='racing_vision_ai',
        executable='vision_ai_node',
        name='stage2_vision_ai',
        output='screen',
        parameters=[{
            'mode': 'stage2',
            'config_path': os.path.join(vision_ai_dir, 'config', 'vision_ai_config.yaml'),
            'trigger_topic': 'stage2_ai_capture',
            'image_topic': '/aurora/rgb/image_raw',
            'result_topic': 'ai_description',
            'status_topic': 'stage2_ai_status',
        }],
        condition=IfCondition(LaunchConfiguration('include_vision_ai')),
    )

    voice_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(voice_driver_dir, 'launch', 'voice_tts.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('include_voice')),
    )

    return LaunchDescription([
        include_map_overlay_arg,
        include_stage1_arg,
        include_stage2_arg,
        include_stage3_arg,
        include_vision_ai_arg,
        include_voice_arg,
        include_depth_arg,
        include_bno055_arg,
        include_obstacle_markers_arg,
        imu_topic_arg,
        test_direction_arg,
        map_yaml_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        map_overlay_stack,
        stage1_stack,
        stage2_stack,
        stage3_stack,
        vision_ai_node,
        voice_stack,
    ])
