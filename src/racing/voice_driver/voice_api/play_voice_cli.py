#!/usr/bin/env python3
"""CLI helper: ros2 run voice_driver play_voice -- 4 | go_ahead"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from voice_api.env_config import VoiceEnvConfig
from voice_api.module_player import ModuleVoicePlayer
from voice_api.voice_client import VoiceClient
from voice_api.voice_ids import voice_id_for_name, voice_name_for_id


class PlayVoiceCli(Node):
    def __init__(self, target: str, *, direct_hardware: bool = True) -> None:
        super().__init__('play_voice_cli')
        voice_id = voice_id_for_name(target)
        if voice_id is None:
            self.get_logger().error(f'Unknown voice target: {target}')
            self._ok = False
            return

        self._ok = False
        if direct_hardware:
            player = ModuleVoicePlayer.from_config(
                VoiceEnvConfig.from_env(),
                logger=self.get_logger(),
            )
            self._ok = player.play_preset(voice_id)

        client = VoiceClient(self, use_service=False)
        client.play(voice_id)
        if direct_hardware and self._ok:
            self.get_logger().info(
                f'Module preset id={voice_id} ({voice_name_for_id(voice_id)})'
            )
        elif not direct_hardware:
            self._ok = True
            self.get_logger().info(
                f'Published play_voice_id={voice_id} (start voice_node for I2C)'
            )

    @property
    def ok(self) -> bool:
        return self._ok


def main(args=None) -> None:
    argv = list(rclpy.utilities.remove_ros_args(args=sys.argv)[1:])
    if not argv:
        print('Usage: ros2 run voice_driver play_voice -- <voice_id|name>')
        print('Example: ros2 run voice_driver play_voice -- go_ahead')
        raise SystemExit(1)

    rclpy.init(args=args)
    node = PlayVoiceCli(argv[0])
    try:
        rclpy.spin_once(node, timeout_sec=1.0)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    raise SystemExit(0 if node.ok else 2)


if __name__ == '__main__':
    main()
