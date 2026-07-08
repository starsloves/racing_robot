from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """雷达测试 launch 文件
    
    只启动：
    1. 雷达驱动（LSLIDAR N10，串口模式）
    2. 雷达测试节点
    
    使用方法：
        ros2 launch racing_stage2_param_test lidar_test.launch.py
    """
    
    # 方案1：直接引用官方 launch（与之前成功配置一致）
    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lslidar_driver'),
                'launch', 'lsn10_launch.py'
            )
        )
    )
    
    return LaunchDescription([
        # 启动雷达驱动（使用官方 launch）
        lidar_launch,
        
        # 启动雷达测试节点
        Node(
            package='racing_stage2_param_test',
            executable='lidar_test',
            name='lidar_test',
            output='screen',
        ),
    ])
