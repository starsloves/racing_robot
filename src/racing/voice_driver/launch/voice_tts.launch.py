from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """Start TTS-only node: subscribe ai_description and speak API text."""
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='tts_only'),
        Node(
            package='voice_driver',
            executable='voice_broadcast_node',
            name='voice_broadcast_node',
            output='screen',
            parameters=[{'mode': LaunchConfiguration('mode')}],
        ),
    ])
