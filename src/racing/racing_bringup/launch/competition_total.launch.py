"""Single production entry point: common base layer plus Supervisor."""

import math
import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def _map_defaults(config_path):
    with open(config_path, 'r', encoding='utf-8') as stream:
        params = yaml.safe_load(stream)['competition_controller']['ros__parameters']
    return (
        str(params['map_to_odom_x']),
        str(params['map_to_odom_y']),
        str(math.radians(float(params['imu_initial_map_yaw_deg']))),
    )


def _operator_tty():
    configured = os.environ.get('RACING_OPERATOR_TTY')
    if configured:
        return configured
    for fd in (1, 2):
        try:
            if os.isatty(fd):
                return os.ttyname(fd)
        except OSError:
            continue
    return '/dev/tty'


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_bringup')
    stage1_dir = get_package_share_directory('racing_stage1')
    voice_dir = get_package_share_directory('voice_driver')
    qr_dir = get_package_share_directory('qr_scanner')
    x_default, y_default, yaw_default = _map_defaults(
        os.path.join(stage1_dir, 'config', 'stage1_controller.yaml')
    )

    map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value=os.path.join(bringup_dir, 'map', 'map_restricted.yaml')
    )
    include_depth = DeclareLaunchArgument('include_depth', default_value='true')
    include_voice = DeclareLaunchArgument('include_voice', default_value='true')
    map_x = DeclareLaunchArgument('map_to_odom_x', default_value=x_default)
    map_y = DeclareLaunchArgument('map_to_odom_y', default_value=y_default)
    map_yaw = DeclareLaunchArgument('map_to_odom_yaw', default_value=yaw_default)

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'origincar_bringup.launch.py')),
        launch_arguments={'carto_slam': 'false'}.items(),
    )
    map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'odom_frame': 'odom_combined',
            'map_to_odom_x': LaunchConfiguration('map_to_odom_x'),
            'map_to_odom_y': LaunchConfiguration('map_to_odom_y'),
            'map_to_odom_yaw': LaunchConfiguration('map_to_odom_yaw'),
        }.items(),
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(qr_dir, 'launch', 'start_competition.launch.py')),
        launch_arguments={
            'include_camera': 'true', 'include_depth': LaunchConfiguration('include_depth'),
            'include_qr': 'false',
        }.items(),
    )
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('lslidar_driver'), 'launch', 'lsn10_launch.py')
        )
    )
    voice = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(voice_dir, 'launch', 'voice_tts.launch.py')),
        condition=IfCondition(LaunchConfiguration('include_voice')),
    )
    supervisor = Node(
        package='racing_bringup', executable='competition_supervisor',
        name='competition_supervisor', output='screen',
        parameters=[{'enable_stage2_vision_ai': True}],
    )

    def _supervisor_exit(event, context):
        if context.is_shutdown:
            return []
        return [Shutdown(reason='competition supervisor finished')]

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', '/home/sunrise/dev_ws/log/competition_runtime'),
        SetEnvironmentVariable('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'own_log'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        SetEnvironmentVariable('RMW_FASTRTPS_TRANSPORT', 'UDPv4'),
        SetEnvironmentVariable('RACING_OPERATOR_TTY', _operator_tty()),
        map_yaml, include_depth, include_voice, map_x, map_y, map_yaw,
        base, map_stack, camera, lidar,
        voice,
        supervisor,
        RegisterEventHandler(OnProcessExit(target_action=supervisor, on_exit=_supervisor_exit)),
    ])
