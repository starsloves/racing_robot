import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _emergency_stop_action():
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'for _ in 1 2 3 4 5 6 7 8; do '
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" '
                '2>/dev/null & sleep 0.03; done; wait'
            ),
        ],
        output='log',
    )


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')

    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')
    inertial_config = os.path.join(stage2_dir, 'config', 'inertial_stage2.yaml')

    include_support_arg = DeclareLaunchArgument('include_support', default_value='true')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    test_direction_arg = DeclareLaunchArgument('test_direction', default_value='clockwise')
    test_start_mode_arg = DeclareLaunchArgument('test_start_mode', default_value='auto')
    field_track_yaml_arg = DeclareLaunchArgument('field_track_yaml', default_value='')
    enable_cmd_relay_arg = DeclareLaunchArgument('enable_cmd_relay', default_value='true')
    relay_input_topic_arg = DeclareLaunchArgument('relay_input_topic', default_value='/stage2_cmd_vel')
    relay_output_topic_arg = DeclareLaunchArgument('relay_output_topic', default_value='/cmd_vel')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')

    camera_topic_arg = DeclareLaunchArgument('camera_topic', default_value='/aurora/rgb/image_raw')
    use_compressed_arg = DeclareLaunchArgument('use_compressed', default_value='false')
    output_dir_arg = DeclareLaunchArgument('output_dir', default_value='/home/sunrise/dev_ws')
    output_prefix_arg = DeclareLaunchArgument('output_prefix', default_value='stage2_path')
    record_fps_arg = DeclareLaunchArgument('record_fps', default_value='15.0')
    max_duration_sec_arg = DeclareLaunchArgument('max_duration_sec', default_value='0.0')

    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': 'false',
            'include_camera': 'true',
            'include_depth': LaunchConfiguration('include_depth'),
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_support')),
    )

    tester_node = Node(
        package='racing_stage2_param_test',
        executable='direct_inertial_tester',
        name='stage2_inertial_navigator',
        parameters=[
            inertial_config,
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'test_direction': LaunchConfiguration('test_direction'),
                'test_start_mode': LaunchConfiguration('test_start_mode'),
                'field_track_yaml': LaunchConfiguration('field_track_yaml'),
            },
        ],
        output='screen',
    )

    cmd_relay_node = Node(
        package='racing_stage2_param_test',
        executable='twist_cmd_relay',
        name='stage2_test_cmd_relay',
        parameters=[{
            'input_topic': LaunchConfiguration('relay_input_topic'),
            'output_topic': LaunchConfiguration('relay_output_topic'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cmd_relay')),
    )

    video_recorder_node = Node(
        package='racing_stage2_param_test',
        executable='camera_video_recorder',
        name='camera_video_recorder',
        parameters=[{
            'camera_topic': LaunchConfiguration('camera_topic'),
            'use_compressed': LaunchConfiguration('use_compressed'),
            'output_dir': LaunchConfiguration('output_dir'),
            'output_prefix': LaunchConfiguration('output_prefix'),
            'fps': LaunchConfiguration('record_fps'),
            'max_duration_sec': LaunchConfiguration('max_duration_sec'),
        }],
        output='screen',
    )

    return LaunchDescription([
        include_support_arg,
        include_bringup_arg,
        include_lidar_arg,
        include_depth_arg,
        imu_topic_arg,
        test_direction_arg,
        test_start_mode_arg,
        field_track_yaml_arg,
        enable_cmd_relay_arg,
        relay_input_topic_arg,
        relay_output_topic_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        carto_slam_arg,
        camera_topic_arg,
        use_compressed_arg,
        output_dir_arg,
        output_prefix_arg,
        record_fps_arg,
        max_duration_sec_arg,
        support_stack,
        cmd_relay_node,
        tester_node,
        video_recorder_node,
        RegisterEventHandler(
            OnShutdown(on_shutdown=[_emergency_stop_action()]),
        ),
    ])
