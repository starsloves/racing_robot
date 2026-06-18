#!/usr/bin/env python3
"""CLI helper: ros2 run voice_driver play_voice -- 4 | go_ahead"""

from __future__ import annotations

import sys

import rclpy
from rclpy.node import Node

from .voice_client import VoiceClient
from .voice_ids import voice_id_for_name, voice_name_for_id


class PlayVoiceCli(Node):
    def __init__(self, target: str) -> None:
        super().__init__('play_voice_cli')
        self._client = VoiceClient(self, use_service=True)
        voice_id = voice_id_for_name(target)
        if voice_id is None:
            self.get_logger().error(f'Unknown voice target: {target}')
            self._ok = False
            return
        self._ok = self._client.play(voice_id)
        if self._ok:
            self.get_logger().info(
                f'Requested voice id={voice_id} ({voice_name_for_id(voice_id)})'
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
