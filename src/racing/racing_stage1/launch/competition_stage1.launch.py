import os
import logging

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable, Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml
import launch.logging as launch_logging
from racing_common.launch_status import startup_status


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


def _shutdown_after_controller_exit(context):
    # Ctrl+C already put launch into shutdown; do not enqueue a second one.
    if context.is_shutdown:
        return []
    return [Shutdown(reason='S1 controller exited')]


def generate_launch_description():
    launch_logging.launch_config.level = logging.ERROR
    lidar_dir = get_package_share_directory('lslidar_driver')
    qr_dir = get_package_share_directory('qr_scanner')
    stage1_dir = get_package_share_directory('racing_stage1')
    bringup_dir = get_package_share_directory('origincar_bringup')
    nav2_dir = get_package_share_directory('nav2_bringup')
    lidar_launch_dir = os.path.join(lidar_dir, 'launch')
    qr_launch_dir = os.path.join(qr_dir, 'launch')
    stage1_config_dir = os.path.join(stage1_dir, 'config')
    stage1_config_path = os.path.join(stage1_config_dir, 'stage1_controller.yaml')
    bringup_launch_dir = os.path.join(bringup_dir, 'launch')
    map_overlay_launch_path = os.path.join(bringup_dir, 'launch', 'map_overlay.launch.py')
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_qr_arg = DeclareLaunchArgument('include_qr', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    imu_topic_arg = DeclareLaunchArgument('imu_topic', default_value='/imu/data')
    carto_slam_arg = DeclareLaunchArgument('carto_slam', default_value='false')
    standalone_map_overlay_arg = DeclareLaunchArgument(
        'standalone_map_overlay',
        default_value='false',
        description='Include map server and map→odom TF for standalone testing'
    )
    include_map_overlay_arg = DeclareLaunchArgument(
        'include_map_overlay',
        default_value='false',
        description='Deprecated compatibility argument; use standalone_map_overlay'
    )
    enable_test_publisher_arg = DeclareLaunchArgument(
        'enable_test_publisher',
        default_value='false',
        description='Publish fixed phase and direction topics for standalone testing'
    )
    test_direction_arg = DeclareLaunchArgument(
        'test_direction',
        default_value='clockwise',
        description='Test direction for standalone testing (clockwise/counterclockwise)'
    )
    standby_arg = DeclareLaunchArgument('standby', default_value='true')
    include_nav2_arg = DeclareLaunchArgument(
        'include_nav2', default_value='true',
        description='Start Nav2 navigation (map->odom remains owned by corner localizer)'
    )
    nav2_config_arg = DeclareLaunchArgument(
        'nav2_config',
        default_value=os.path.join(stage1_config_dir, 'stage1_nav2.yaml'),
    )
    nav2_params = RewrittenYaml(
        source_file=LaunchConfiguration('nav2_config'),
        root_key='',
        param_rewrites={
            'default_nav_to_pose_bt_xml': os.path.join(
                stage1_dir, 'config', 'navigate_to_pose_s1.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                stage1_dir, 'config', 'navigate_through_poses_s1.xml'),
        },
        convert_types=True,
    )

    bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(bringup_launch_dir, 'origincar_bringup.launch.py')),
        launch_arguments={
            'carto_slam': LaunchConfiguration('carto_slam'),
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_bringup')),
    )

    map_overlay_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(map_overlay_launch_path),
        launch_arguments={
            'odom_frame': 'odom_combined',
            'publish_map_to_odom': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('standalone_map_overlay')),
    )

    test_publisher = Node(
        package='racing_stage1',
        executable='stage_test_publisher_fixed',
        name='stage_test_publisher',
        parameters=[{
            'stage_number': 1,
            'test_direction': LaunchConfiguration('test_direction'),
            'publish_phase': False,
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('enable_test_publisher')),
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(qr_launch_dir, 'start_competition.launch.py')),
        launch_arguments={
            'device': LaunchConfiguration('device'),
            'include_camera': LaunchConfiguration('include_camera'),
            'include_qr': LaunchConfiguration('include_qr'),
            'include_depth': LaunchConfiguration('include_depth'),
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
        }.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(lidar_launch_dir, 'lsn10_launch.py')),
        condition=IfCondition(LaunchConfiguration('include_lidar')),
    )

    # Nav2 is the only normal-motion owner in S1.  Deliberately include only
    # navigation_launch.py: map->odom is already supplied by the shared
    # start-corner localizer, so AMCL/localization_launch must not be started.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_dir, 'launch', 'navigation_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'autostart': 'true',
            'params_file': nav2_params,
            # navigation_launch.py evaluates these two values as Python
            # expressions, so use its capitalized boolean spelling.
            'use_composition': 'False',
            'use_respawn': 'False',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_nav2')),
    )

    controller_node = Node(
        package='racing_stage1',
        executable='competition_controller',
        name='competition_controller',
        parameters=[
            stage1_config_path,  # 统一配置文件
            {
                'imu_topic': LaunchConfiguration('imu_topic'),
                'standby': LaunchConfiguration('standby'),
            },
        ],
        output='log',
    )

    # Standalone S1 tests run the startup localizer here.  Production runs it
    # once in competition_total.launch.py so the TF survives S1 release.
    corner_diagnostic_node = Node(
        package='racing_tools',
        executable='start_corner_pose_diagnostic',
        name='start_corner_pose_diagnostic',
        parameters=[
            os.path.join(
                get_package_share_directory('racing_tools'),
                'config', 'start_corner_pose_diagnostic.yaml',
            ),
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('standalone_map_overlay')),
    )

    # Process output remains off the terminal, while ROS keeps complete
    # startup and failure diagnostics in this stage's runtime log directory.
    session_root = os.environ.get('RACING_SESSION_ROOT', '').strip()
    runtime_ros_log_dir = SetEnvironmentVariable(
        'ROS_LOG_DIR', os.path.join(session_root, 'ros', 'stage1')
        if session_root else os.path.join(os.environ.get('DEV_WS', os.path.expanduser('~/dev_ws')), 'log', 'stage1', 'ros')
    )
    isolate_process_output = SetEnvironmentVariable(
        'OVERRIDE_LAUNCH_PROCESS_OUTPUT', 'own_log'
    )

    return LaunchDescription([
        runtime_ros_log_dir,
        isolate_process_output,
        SetEnvironmentVariable('RACING_OPERATOR_TTY', _operator_tty()),
        device_arg,
        include_camera_arg,
        include_qr_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        include_bringup_arg,
        include_lidar_arg,
        imu_topic_arg,
        carto_slam_arg,
        standalone_map_overlay_arg,
        include_map_overlay_arg,
        enable_test_publisher_arg,
        test_direction_arg,
        standby_arg,
        include_nav2_arg,
        nav2_config_arg,
        bringup_launch,
        map_overlay_stack,
        test_publisher,
        base_launch,
        lidar_launch,
        nav2_launch,
        corner_diagnostic_node,
        controller_node,
        RegisterEventHandler(OnProcessStart(
            target_action=controller_node,
            on_start=[startup_status('第一阶段控制器', '/competition_controller')],
        )),
        # The controller is the stage's lifecycle owner.  On normal release
        # it calls rclpy.shutdown() and exits, so the child launch wrapper
        # must tear down Nav2/QR resources as well; otherwise the Supervisor
        # keeps seeing a live S1 process after motion ownership has moved to
        # S2.  The same shutdown path also handles an unexpected controller
        # crash without leaving a second motion stack behind.
        RegisterEventHandler(OnProcessExit(
            target_action=controller_node,
            on_exit=[OpaqueFunction(function=_shutdown_after_controller_exit)],
        )),
    ])
