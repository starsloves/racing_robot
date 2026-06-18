"""ROS 2 client API for Yahboom voice broadcast."""

from __future__ import annotations

from rclpy.node import Node
from std_msgs.msg import Int32

from .voice_ids import VoiceId, voice_id_for_name, voice_name_for_id

try:
    from voice_driver.srv import PlayVoice
except ImportError:  # pragma: no cover - before colcon build
    PlayVoice = None


class VoiceClient:
    """Publish or service-call voice broadcast requests."""

    def __init__(
        self,
        node: Node,
        *,
        topic: str = 'play_voice_id',
        service: str = 'play_voice',
        use_service: bool = True,
    ) -> None:
        self._node = node
        self._topic = topic
        self._service_name = service
        self._use_service = use_service and PlayVoice is not None
        self._publisher = node.create_publisher(Int32, topic, 10)
        self._service_client = None
        if self._use_service:
            self._service_client = node.create_client(PlayVoice, service)
        elif use_service:
            node.get_logger().warn(
                'voice_driver PlayVoice service type unavailable; using topic publish only'
            )

    def play(self, voice_id: int) -> bool:
        """Broadcast by numeric ID. Returns True when request was sent or succeeded."""
        voice_id = int(voice_id) & 0xFF
        if self._service_client is not None and self._service_client.service_is_ready():
            request = PlayVoice.Request()
            request.voice_id = voice_id
            try:
                response = self._service_client.call(request)
            except Exception as exc:  # noqa: BLE001 - surface service failure to caller logs
                self._node.get_logger().error(f'play_voice service failed: {exc}')
                return self._publish(voice_id)
            if response.success:
                self._node.get_logger().info(
                    f'Voice broadcast ok id={voice_id} ({voice_name_for_id(voice_id)})'
                )
                return True
            self._node.get_logger().error(
                f'Voice broadcast failed id={voice_id}: {response.message}'
            )
            return False
        if self._service_client is not None:
            self._node.get_logger().debug(
                f'play_voice service not ready, publishing topic id={voice_id}'
            )
        return self._publish(voice_id)

    def play_name(self, name: str) -> bool:
        """Broadcast using a preset name such as ``go_ahead``."""
        voice_id = voice_id_for_name(name)
        if voice_id is None:
            self._node.get_logger().error(f'Unknown voice name: {name}')
            return False
        return self.play(voice_id)

    def stop(self) -> bool:
        return self.play(VoiceId.STOP)

    def go_ahead(self) -> bool:
        return self.play(VoiceId.GO_AHEAD)

    def back(self) -> bool:
        return self.play(VoiceId.BACK)

    def turn_left(self) -> bool:
        return self.play(VoiceId.TURN_LEFT)

    def turn_right(self) -> bool:
        return self.play(VoiceId.TURN_RIGHT)

    def _publish(self, voice_id: int) -> bool:
        msg = Int32()
        msg.data = voice_id
        self._publisher.publish(msg)
        self._node.get_logger().info(
            f'Published play_voice_id={voice_id} ({voice_name_for_id(voice_id)})'
        )
        return True
