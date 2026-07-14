#!/usr/bin/env python3
"""一键启动：相机 → BPU 推理 → OpenCV 窗口"""
import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    racing_stage2_share = get_package_share_directory('racing_stage2')

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            racing_stage2_share, 'launch', 'competition_support.launch.py')),
        launch_arguments={
            'include_camera':  'true',
            'include_bringup': 'false',
            'include_lidar':   'false',
            'include_bno055':  'false',
            'rgb_fps':         '5',
        }.items(),
    )

    tester = Node(
        package='racing_stage2_param_test',
        executable='bpu_direct_test',
        name='bpu_direct_test',
        output='screen',
    )

    return LaunchDescription([camera, tester])
