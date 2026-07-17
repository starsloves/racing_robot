from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def _emergency_stop_action():
    return ExecuteProcess(
        cmd=[
            'bash',
            '-c',
            (
                'set +e; '
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" '
                '2>/dev/null; '
                'true'
            ),
        ],
        output='log',
    )


def generate_launch_description():
    pkg_dir = get_package_share_directory('racing_stage2_seg_follow')
    bringup_dir = get_package_share_directory('origincar_bringup')
    default_config = os.path.join(pkg_dir, 'config', 'seg_line_follower.yaml')
    bringup_launch_path = os.path.join(bringup_dir, 'launch', 'origincar_bringup.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('config', default_value=default_config),
        DeclareLaunchArgument('include_bringup', default_value='true'),
        DeclareLaunchArgument('carto_slam', default_value='false'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('rgb_fps', default_value='15'),
        DeclareLaunchArgument('resolution_mode_index', default_value='2'),
        DeclareLaunchArgument('include_depth', default_value='false'),
        DeclareLaunchArgument('phase_gate_enabled', default_value='false'),
        DeclareLaunchArgument('cmd_vel_topic', default_value='/cmd_vel'),
        DeclareLaunchArgument('http_port', default_value='8092'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bringup_launch_path),
            launch_arguments={'carto_slam': LaunchConfiguration('carto_slam')}.items(),
            condition=IfCondition(LaunchConfiguration('include_bringup')),
        ),
        Node(
            package='deptrum-ros-driver-aurora930',
            executable='aurora930_node',
            namespace='aurora',
            name='aurora930_node',
            output='screen',
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
            condition=IfCondition(LaunchConfiguration('start_camera')),
        ),
        Node(
            package='racing_stage2_seg_follow',
            executable='seg_line_follower',
            name='seg_line_follower',
            output='screen',
            parameters=[
                LaunchConfiguration('config'),
                {
                    'phase_gate_enabled': LaunchConfiguration('phase_gate_enabled'),
                    'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                    'http_port': LaunchConfiguration('http_port'),
                },
            ],
        ),
        RegisterEventHandler(OnShutdown(on_shutdown=[_emergency_stop_action()])),
    ])
