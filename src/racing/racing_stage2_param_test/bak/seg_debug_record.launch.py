# seg_debug_record.launch.py
# 纯惯导驱动 + 分割模型输出录制（合并启动）

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    racing_stage2_share = get_package_share_directory('racing_stage2')
    stage2_dir = get_package_share_directory('racing_stage2')
    param_test_dir = get_package_share_directory('racing_stage2_param_test')
    inertial_config = os.path.join(stage2_dir, 'config', 'inertial_stage2.yaml')
    test_config = os.path.join(param_test_dir, 'config', 'direct_inertial_test.yaml')

    args = [
        ('camera_topic',        '/aurora/rgb/image_raw', '相机话题'),
        ('test_direction',      'clockwise', '顺时针/逆时针'),
        ('save_raw',            'false',     '同时保存原始相机图'),
    ]
    decl = [DeclareLaunchArgument(name, default_value=dfl, description=desc)
            for name, dfl, desc in args]

    support = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            racing_stage2_share, 'launch', 'competition_support.launch.py')),
        launch_arguments={
            'include_camera':  'true',
            'include_bringup': 'true',
            'include_lidar':   'true',
            'include_bno055':  'false',
            'rgb_fps':         '5',
        }.items(),
    )

    # 纯惯导 — 负责开车
    inertial = Node(
        package='racing_stage2_param_test',
        executable='direct_inertial_tester',
        name='stage2_inertial_navigator',
        output='screen',
        parameters=[
            inertial_config,
            test_config,
            {
                'test_direction': LaunchConfiguration('test_direction'),
            },
        ],
    )

    # 分割模型查看器 — 只录不控
    viewer = Node(
        package='racing_stage2_param_test',
        executable='seg_debug_viewer',
        name='seg_debug_viewer',
        output='screen',
        parameters=[{
            'camera_topic':  LaunchConfiguration('camera_topic'),
            'save_raw':      LaunchConfiguration('save_raw'),
        }],
    )

    # cmd_vel 转发
    relay = Node(
        package='racing_stage2_param_test',
        executable='twist_cmd_relay',
        name='test_cmd_relay',
        parameters=[{
            'input_topic':  '/stage2_cmd_vel',
            'output_topic': '/cmd_vel',
        }],
        output='screen',
    )

    return LaunchDescription(decl + [support, inertial, viewer, relay])