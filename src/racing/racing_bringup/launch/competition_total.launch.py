"""Single production entry point: common base layer plus Supervisor."""

import os
import socket
import threading

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
    SetEnvironmentVariable,
    Shutdown,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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


def _web_host_hint():
    """Pick a usable IPv4 address for the terminal browser hint."""
    try:
        probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        probe.connect(('8.8.8.8', 80))
        address = probe.getsockname()[0]
        probe.close()
        if not address.startswith('127.'):
            return address
    except OSError:
        pass
    try:
        addresses = socket.getaddrinfo(socket.gethostname(), None, socket.AF_INET)
        for _, _, _, _, sockaddr in addresses:
            address = sockaddr[0]
            if not address.startswith('127.'):
                return address
    except OSError:
        pass
    return '127.0.0.1'


_SUPERVISOR_SHUTDOWN_LOCK = threading.Lock()
_SUPERVISOR_SHUTDOWN_REQUESTED = False


def generate_launch_description():
    bringup_dir = get_package_share_directory('origincar_bringup')
    voice_dir = get_package_share_directory('voice_driver')
    qr_dir = get_package_share_directory('qr_scanner')
    tools_dir = get_package_share_directory('racing_tools')

    map_yaml = DeclareLaunchArgument(
        'map_yaml', default_value=os.path.join(bringup_dir, 'map', 'map_restricted.yaml')
    )
    include_depth = DeclareLaunchArgument('include_depth', default_value='true')
    include_voice = DeclareLaunchArgument('include_voice', default_value='true')
    enable_web_monitor = DeclareLaunchArgument('enable_web_monitor', default_value='true')
    web_monitor_host = DeclareLaunchArgument('web_monitor_host', default_value='0.0.0.0')
    web_monitor_port = DeclareLaunchArgument('web_monitor_port', default_value='8081')
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'origincar_bringup.launch.py')),
        launch_arguments={'carto_slam': 'false'}.items(),
    )
    map_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')),
        launch_arguments={
            'map_yaml': LaunchConfiguration('map_yaml'),
            'odom_frame': 'odom_combined',
            'publish_map_to_odom': 'false',
        }.items(),
    )
    start_localizer = Node(
        package='racing_tools',
        executable='start_corner_pose_diagnostic',
        name='start_corner_pose_localizer',
        parameters=[
            os.path.join(tools_dir, 'config', 'start_corner_pose_diagnostic.yaml'),
            {'odom_frame': 'odom_combined'},
        ],
        output='log',
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
    def _supervisor_exit(event, context):
        del event
        if context.is_shutdown:
            return []
        global _SUPERVISOR_SHUTDOWN_REQUESTED
        with _SUPERVISOR_SHUTDOWN_LOCK:
            if _SUPERVISOR_SHUTDOWN_REQUESTED:
                return []
            _SUPERVISOR_SHUTDOWN_REQUESTED = True
        return [
            LogInfo(msg='competition supervisor exited; shutting down top-level launch'),
            Shutdown(reason='competition supervisor finished'),
        ]

    # Bind the completion callback directly to the ExecuteProcess action
    # created by Node.  This keeps the callback attached to the exact process
    # action that emits ProcessExited, including when the Supervisor exits
    # through its normal rclpy shutdown path.
    supervisor = Node(
        package='racing_bringup', executable='competition_supervisor',
        name='competition_supervisor', output='screen',
        parameters=[{'enable_stage2_vision_ai': True}],
        on_exit=_supervisor_exit,
    )
    web_monitor = Node(
        package='racing_tools',
        executable='telemetry_web_monitor',
        name='telemetry_web_monitor',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'host': LaunchConfiguration('web_monitor_host'),
            'port': LaunchConfiguration('web_monitor_port'),
            'map_heading_topic': 'map_heading',
            'route_topic': 'stage1_route',
            'mission_route_topic': 'stage1_mission_route',
            'heading_motion_linear_threshold_mps': 0.015,
            'heading_motion_angular_threshold_rad_s': 0.03,
            'history_min_step_m': 0.06,
        }],
        condition=IfCondition(LaunchConfiguration('enable_web_monitor')),
    )
    web_monitor_hint = LogInfo(
        msg=[
            '[WEB] BROWSER_URL=http://', _web_host_hint(),
            ':', LaunchConfiguration('web_monitor_port'), '/ '
            '(默认地址；若端口占用，以监视器输出的实际 URL 为准；页面只读)',
        ],
        condition=IfCondition(LaunchConfiguration('enable_web_monitor')),
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROS_LOG_DIR', '/home/sunrise/dev_ws/log/competition_runtime'),
        SetEnvironmentVariable('OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'own_log'),
        SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0'),
        SetEnvironmentVariable('RMW_FASTRTPS_TRANSPORT', 'UDPv4'),
        SetEnvironmentVariable('RACING_OPERATOR_TTY', _operator_tty()),
        map_yaml, include_depth, include_voice,
        enable_web_monitor, web_monitor_host, web_monitor_port,
        base, map_stack, camera, lidar, start_localizer,
        voice,
        web_monitor,
        web_monitor_hint,
        supervisor,
    ])
