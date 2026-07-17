import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')
    bringup_dir = get_package_share_directory('origincar_bringup')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')

    # 使用统一配置文件
    stage2_config = os.path.join(stage2_dir, 'config', 'stage2_controller.yaml')
    obstacle_marker_config = os.path.join(stage2_dir, 'config', 'obstacle_circle_markers.yaml')

    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='false')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='false')
    include_bno055_arg = DeclareLaunchArgument('include_bno055', default_value='false')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='false')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    include_obstacle_markers_arg = DeclareLaunchArgument('include_obstacle_markers', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    bno055_i2c_bus_arg = DeclareLaunchArgument('bno055_i2c_bus', default_value='5')
    bno055_i2c_addr_arg = DeclareLaunchArgument('bno055_i2c_addr', default_value='41')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    standalone_map_overlay_arg = DeclareLaunchArgument(
        'standalone_map_overlay',
        default_value='false',
        description='Include map server and map→odom TF for standalone testing',
    )
    include_map_overlay_arg = DeclareLaunchArgument(
        'include_map_overlay',
        default_value='false',
        description='Deprecated compatibility argument; use standalone_map_overlay',
    )
    enable_test_publisher_arg = DeclareLaunchArgument(
        'enable_test_publisher',
        default_value='false',
        description='Publish phase and direction topics for standalone testing',
    )
    map_to_odom_x_arg = DeclareLaunchArgument(
        'map_to_odom_x',
        default_value='2.80',
        description='Stage2 start X position in map frame (channel_entry)',
    )
    map_to_odom_y_arg = DeclareLaunchArgument(
        'map_to_odom_y',
        default_value='3.10',
        description='Stage2 start Y position in map frame (channel_entry)',
    )
    map_to_odom_yaw_arg = DeclareLaunchArgument(
        'map_to_odom_yaw',
        default_value='1.5707963268',
        description='Stage2 start yaw in map frame (rad, 90°)',
    )
    test_direction_arg = DeclareLaunchArgument(
        'test_direction',
        default_value='clockwise',
        description='Fallback direction for standalone testing (clockwise/counterclockwise)',
    )
    enable_cmd_relay_arg = DeclareLaunchArgument(
        'enable_cmd_relay',
        default_value='false',
        description='Relay Stage2 velocity commands to /cmd_vel (standalone only)',
    )
    relay_input_topic_arg = DeclareLaunchArgument('relay_input_topic', default_value='/stage2_cmd_vel')
    relay_output_topic_arg = DeclareLaunchArgument('relay_output_topic', default_value='/cmd_vel')

    # Optional support stack for standalone runs only
    support_actions = []
    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')
    if os.path.exists(support_launch_path):
        support_actions.append(
            IncludeLaunchDescription(
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
        )

    map_overlay_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_overlay_launch_path),
        launch_arguments={
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
            'odom_frame': 'odom_combined',
        }.items(),
        condition=IfCondition(LaunchConfiguration('standalone_map_overlay')),
    )

    test_publisher = Node(
        package='racing_stage2',
        executable='stage_test_publisher',
        name='stage_test_publisher',
        parameters=[{
            'stage_number': 2,
            'test_direction': LaunchConfiguration('test_direction'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_test_publisher')),
    )

    stage2_navigator = Node(
        package='racing_stage2',
        executable='stage2_inertial_navigator',
        name='stage2_inertial_navigator',
        parameters=[
            stage2_config,  # 统一配置文件（包含原 inertial + avoid 参数）
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'test_direction': LaunchConfiguration('test_direction'),
                'use_test_direction_fallback': LaunchConfiguration('enable_test_publisher'),
            },
        ],
        output='screen',
    )

    cmd_relay = Node(
        package='racing_stage2',
        executable='twist_cmd_relay',
        name='stage2_cmd_relay',
        parameters=[{
            'input_topic': LaunchConfiguration('relay_input_topic'),
            'output_topic': LaunchConfiguration('relay_output_topic'),
        }],
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_cmd_relay')),
    )

    obstacle_circle_markers = Node(
        package='racing_stage2',
        executable='lidar_obstacle_circle_markers',
        name='lidar_obstacle_circle_markers',
        parameters=[obstacle_marker_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('include_obstacle_markers')),
    )

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
        standalone_map_overlay_arg,
        include_map_overlay_arg,
        enable_test_publisher_arg,
        map_to_odom_x_arg,
        map_to_odom_y_arg,
        map_to_odom_yaw_arg,
        test_direction_arg,
        enable_cmd_relay_arg,
        relay_input_topic_arg,
        relay_output_topic_arg,
        *support_actions,
        map_overlay_stack,
        test_publisher,
        stage2_navigator,
        cmd_relay,
        obstacle_circle_markers,
    ])
