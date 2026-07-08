"""极简直行避障测试 Launch 文件

启动：
- competition_support.launch.py（激光雷达 + IMU + 底盘）
- simple_avoid_tester 节点（加载 avoidance_config.yaml）
- 退出时急停

用法：
    ros2 launch racing_stage2_param_test simple_avoid_test.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _emergency_stop_action():
    """退出时急停命令"""
    return ExecuteProcess(
        cmd=[
            'bash', '-c',
            (
                'for _ in 1 2 3 4 5 6 7 8; do '
                'ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '
                '"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" '
                '2>/dev/null & sleep 0.03; done; wait'
            ),
        ],
        output='log',
    )


def generate_launch_description():
    stage2_dir = get_package_share_directory('racing_stage2')
    param_test_dir = get_package_share_directory('racing_stage2_param_test')
    
    support_launch_path = os.path.join(
        stage2_dir, 'launch', 'competition_support.launch.py'
    )
    
    # DDS 和日志配置（与 direct_inertial_test.launch.py 一致）
    disable_shm = SetEnvironmentVariable('RMW_FASTRTPS_USE_SHM', '0')
    force_udp = SetEnvironmentVariable('RMW_FASTRTPS_TRANSPORT', 'UDPv4')
    ros_log_dir = SetEnvironmentVariable('ROS_LOG_DIR',
        os.path.join(os.path.expanduser('~'), 'dev_ws', 'log', 'ros'))
    
    # 加载原配置（与 direct_inertial_test.launch.py 一致）
    inertial_config = os.path.join(param_test_dir, 'config', 'inertial_stage2.yaml')
    test_config = os.path.join(param_test_dir, 'config', 'direct_inertial_test.yaml')
    avoid_config = os.path.join(param_test_dir, 'config', 'avoidance_config.yaml')
    
    # Launch 参数
    include_support_arg = DeclareLaunchArgument(
        'include_support',
        default_value='true',
        description='是否启动支持栈（激光雷达 + IMU + 底盘）'
    )
    
    include_lidar_arg = DeclareLaunchArgument(
        'include_lidar',
        default_value='true',
        description='是否启动激光雷达'
    )
    
    # 支持栈（激光雷达 + IMU + 底盘）
    support_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(support_launch_path),
        launch_arguments={
            'include_bringup': 'true',
            'include_lidar': LaunchConfiguration('include_lidar'),
            'include_bno055': 'false',
            'include_camera': 'false',
            'include_depth': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('include_support')),
    )
    
    # 测试节点（加载完整原配置）
    tester_node = Node(
        package='racing_stage2_param_test',
        executable='simple_avoid_tester',
        name='stage2_inertial_navigator',
        parameters=[
            inertial_config,   # 基础参数（速度、转弯、通道导航等）
            test_config,       # 测试覆盖（位姿源=wheel、轮速 topic 等）
            avoid_config,      # 避障参数
            {
                'cmd_topic': '/cmd_vel',  # 直接发布到 /cmd_vel（原默认 /stage2_cmd_vel）
                'session_log_subdir': 'simple_avoid_test',  # 独立日志目录
            },
        ],
        output='screen',
    )
    
    # 退出时急停
    emergency_stop_handler = RegisterEventHandler(
        OnShutdown(on_shutdown=[_emergency_stop_action()])
    )
    
    return LaunchDescription([
        disable_shm,
        force_udp,
        ros_log_dir,
        
        include_support_arg,
        include_lidar_arg,
        support_stack,
        tester_node,
        emergency_stop_handler,
    ])
