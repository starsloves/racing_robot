import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, SetEnvironmentVariable
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
                'set +e; '
                # 先停车
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" '
                '2>/dev/null & '
                # 杀掉所有相关进程（雷达驱动放在最前面）
                'pkill -15 -f lslidar_driver_node 2>/dev/null; '
                'sleep 0.3; '
                'pkill -9 -f lslidar_driver_node 2>/dev/null; '
                'pkill -9 -f lslidar 2>/dev/null; '
                'pkill -9 -f direct_inertial_tester 2>/dev/null; '
                'pkill -9 -f twist_cmd_relay 2>/dev/null; '
                'pkill -9 -f data_recorder 2>/dev/null; '
                'pkill -9 -f stage2_inertial_navigator 2>/dev/null; '
                'pkill -9 -f competition_support 2>/dev/null; '
                'pkill -9 -f origincar 2>/dev/null; '
                'pkill -9 -f ros2 2>/dev/null; '
                'pkill -9 -f fastrtps 2>/dev/null; '
                'pkill -9 -f carthographer 2>/dev/null; '
                'pkill -9 -f cartographer 2>/dev/null; '
                'pkill -9 -f robot_state_publisher 2>/dev/null; '
                'pkill -9 -f static_transform 2>/dev/null; '
                'pkill -9 -f imu_filter 2>/dev/null; '
                'pkill -9 -f robot_localization 2>/dev/null; '
                'pkill -9 -f ekf 2>/dev/null; '
                'pkill -9 -f realsense 2>/dev/null; '
                'pkill -9 -f camera 2>/dev/null; '
                'pkill -9 -f usb_cam 2>/dev/null; '
                'pkill -9 -f depthimage 2>/dev/null; '
                'pkill -9 -f vision_inertial 2>/dev/null; '
                'pkill -9 -f vision_record 2>/dev/null; '
                'pkill -9 -f simple_avoidance 2>/dev/null; '
                'pkill -9 -f qr_scanner 2>/dev/null; '
                'pkill -9 -f voice_driver 2>/dev/null; '
                'pkill -9 -f racing 2>/dev/null; '
                'pkill -9 -f stage2_cmd_vel 2>/dev/null; '
                'pkill -9 -f cmd_vel 2>/dev/null; '
                'pkill -9 -f pointcloud_to_laserscan 2>/dev/null; '
                'pkill -9 -f laser_filter 2>/dev/null; '
                'pkill -9 -f slam 2>/dev/null; '
                'pkill -9 -f rviz 2>/dev/null; '
                'pkill -9 -f odom 2>/dev/null; '
                'pkill -9 -f twist_mux 2>/dev/null; '
                # 确保串口释放（杀掉所有占用 /dev/ttyACM1 的进程）
                'for pid in $(lsof -t /dev/ttyACM1 2>/dev/null); do kill -9 $pid 2>/dev/null; done; '
                'true'
            ),
        ],
        output='log',
    )


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')
    param_test_dir = get_package_share_directory('racing_stage2_param_test')

    support_launch_path = os.path.join(stage2_dir, 'launch', 'competition_support.launch.py')
    inertial_config = os.path.join(param_test_dir, 'config', 'inertial_stage2.yaml')
    test_config = os.path.join(param_test_dir, 'config', 'direct_inertial_test.yaml')
    avoid_config = os.path.join(param_test_dir, 'config', 'avoidance_config.yaml')

    include_support_arg = DeclareLaunchArgument('include_support', default_value='true')
    include_bringup_arg = DeclareLaunchArgument('include_bringup', default_value='true')
    include_lidar_arg = DeclareLaunchArgument('include_lidar', default_value='true')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='false')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    include_recorder_arg = DeclareLaunchArgument('include_recorder', default_value='true')
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

    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': LaunchConfiguration('include_bringup'),
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': 'false',
            'include_camera': LaunchConfiguration('include_camera'),
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
            test_config,
            avoid_config,
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
        output='log',
        condition=IfCondition(LaunchConfiguration('enable_cmd_relay')),
    )

    recorder_node = Node(
        package='racing_stage2_param_test',
        executable='data_recorder',
        name='data_recorder',
        parameters=[{
            'record_rate_hz': 1.0,
            'record_subdir': 'data_records',
            'record_filename': 'latest_record.csv',
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('include_recorder')),
    )

    # Disable Fast DDS shared-memory transport to avoid /dev/shm init failures.
    disable_shm = SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0')
    force_udp = SetEnvironmentVariable('RMW_FASTRTPS_TRANSPORT', 'UDPv4')

    # Redirect ROS 2 built-in logs into our log/ directory so they don't
    # clutter ~/.ros/log/.
    ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR',
        os.path.join(os.path.expanduser('~'), 'dev_ws', 'log', 'ros'))

    return LaunchDescription([
        disable_shm,
        force_udp,
        ros_log_dir,

        include_support_arg,
        include_bringup_arg,
        include_lidar_arg,
        include_camera_arg,
        include_depth_arg,
        include_recorder_arg,
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
        support_stack,
        cmd_relay_node,
        tester_node,
        recorder_node,
        RegisterEventHandler(
            OnShutdown(on_shutdown=[_emergency_stop_action()]),
        ),
    ])
