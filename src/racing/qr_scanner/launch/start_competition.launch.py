import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessStart
from racing_common.launch_status import startup_status


def generate_launch_description():
    # 保留 device 参数以兼容上层 launch 透传，但 Aurora 方案不再使用它。
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    include_qr_arg = DeclareLaunchArgument('include_qr', default_value='true')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')

    aurora_node = Node(
        package='deptrum-ros-driver-aurora930',
        executable='aurora930_node',
        namespace='aurora',
        parameters=[{
            'rgb_enable': True,
            # Aurora RGB streaming is paired with the IR timing stream; the
            # vendor driver requires IR FPS to be at least the RGB FPS.
            'ir_enable': True,
            'ir_fps': LaunchConfiguration('rgb_fps'),
            'depth_enable': LaunchConfiguration('include_depth'),
            'rgbd_enable': False,
            'point_cloud_enable': False,
            'boot_order': 1,
            'rgb_fps': LaunchConfiguration('rgb_fps'),
            'resolution_mode_index': LaunchConfiguration('resolution_mode_index'),
            'align_mode': LaunchConfiguration('include_depth'),
            'log_dir': '/tmp/',
            'stream_sdk_log_enable': False,
            # Keep the device heartbeat/watchdog enabled so a stalled USB
            # frame stream can recover instead of leaving a live node with no
            # camera messages.
            'heart_enable': True,
        }],
        output='log',
        condition=IfCondition(LaunchConfiguration('include_camera')),
    )

    qr_node = Node(
        package='qr_scanner',
        executable='qr_scanner',
        name='qr_scanner',
        parameters=[
            {'camera_topic': '/aurora/rgb/image_raw'},
            {'use_compressed': False},
            {'result_topic': 'qr_scan_result'},
            {'odom_topic': '/odom_combined'},
            {'scan_task_phase': 1},
            # Keep QR startup evidence separate from the Stage1 controller
            # session, which truncates latest.log when it starts.
            {'diagnostics_log_subdir': 'competition_stage1'},
            {'diagnostics_log_filename': 'qr_scanner.log'},
            {'crop_top_ratio': 0.25},
            {'crop_top_px': 80},
            {'upscale_factor': 1.0},
            {'detection_order': 'crop_only'},
        ],
        output='log',
        condition=IfCondition(LaunchConfiguration('include_qr')),
    )

    return LaunchDescription([
        device_arg,
        include_camera_arg,
        include_depth_arg,
        include_qr_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        aurora_node,
        qr_node,
        RegisterEventHandler(OnProcessStart(
            target_action=aurora_node,
            on_start=[startup_status('相机', '/aurora/aurora')],
        )),
        RegisterEventHandler(OnProcessStart(
            target_action=qr_node,
            on_start=[startup_status('二维码节点', '/qr_scanner')],
        )),
    ])
