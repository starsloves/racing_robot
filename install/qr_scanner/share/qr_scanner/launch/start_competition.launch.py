import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 保留 device 参数以兼容上层 launch 透传，但 Aurora 方案不再使用它。
    device_arg = DeclareLaunchArgument('device', default_value='/dev/video0')
    include_camera_arg = DeclareLaunchArgument('include_camera', default_value='true')
    include_depth_arg = DeclareLaunchArgument('include_depth', default_value='false')
    rgb_fps_arg = DeclareLaunchArgument('rgb_fps', default_value='15')
    resolution_mode_index_arg = DeclareLaunchArgument('resolution_mode_index', default_value='2')

    aurora_node = Node(
        package='deptrum-ros-driver-aurora930',
        executable='aurora930_node',
        namespace='aurora',
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
            {'phase_topic': 'competition_phase'},
            {'odom_topic': '/odom_combined'},
            {'scan_task_phase': 1},
            {'scan_start_x_m': 2.0},
            {'scan_rate_hz': 4.0},
            {'crop_top_ratio': 0.25},
            {'crop_top_px': 155},
            {'process_every_frame': True},
            {'backend': 'wechat'},
            {'allow_backend_fallback': True},
        ],
        output='screen'
    )

    return LaunchDescription([
        device_arg,
        include_camera_arg,
        include_depth_arg,
        rgb_fps_arg,
        resolution_mode_index_arg,
        aurora_node,
        qr_node
    ])