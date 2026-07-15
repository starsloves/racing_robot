# seg_debug_viewer.launch.py
# 分割模型调试查看�?�?仅启动底�?摄像�?模型推理并保存结果图

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    racing_stage2_share = get_package_share_directory('racing_stage2')

    args = [
        ('camera_topic',        '/aurora/rgb/image_raw', '相机话题'),
        ('save_raw',            'false',  '同时保存原始相机�?),
    ]
    decl = [DeclareLaunchArgument(name, default_value=dfl, description=desc)
            for name, dfl, desc in args]

    support = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            racing_stage2_share, 'launch', 'competition_support.launch.py')),
        launch_arguments={
            'include_camera':  'true',
            'include_bringup': 'true',
            'include_lidar':   'false',
            'include_bno055':  'false',
            'rgb_fps':         '5',
        }.items(),
    )

    viewer = Node(
        package='racing_stage2',
        executable='seg_debug_viewer',
        name='seg_debug_viewer',
        output='screen',
        parameters=[{
            'camera_topic':  LaunchConfiguration('camera_topic'),
            'save_raw':      LaunchConfiguration('save_raw'),
        }],
    )

    return LaunchDescription(decl + [support, viewer])
