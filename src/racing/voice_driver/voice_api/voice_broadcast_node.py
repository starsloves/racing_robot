#!/usr/bin/env python3
"""ROS 2 node: image -> vision LLM -> TTS voice broadcast."""

from __future__ import annotations

import threading
from queue import Empty, Queue

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Int32, String

from voice_api.env_config import VoiceEnvConfig
from voice_api.voice_broadcast import VoiceBroadcastService


class VoiceBroadcastNode(Node):
    """Subscribe to trigger/image or ai_description and speak the result."""

    def __init__(self) -> None:
        super().__init__('voice_broadcast_node')
        self.declare_parameter('mode', '')
        self.declare_parameter('target_sign', -1)

        self._config = VoiceEnvConfig.from_env()
        mode = self.get_parameter('mode').get_parameter_value().string_value.strip()
        self._mode = mode or self._config.broadcast_mode

        target_sign = self.get_parameter('target_sign').get_parameter_value().integer_value
        self._target_sign = target_sign if target_sign >= 0 else self._config.target_sign

        self._service = VoiceBroadcastService(self._config, logger=self.get_logger())
        self._bridge = CvBridge()
        self._speech_queue: Queue[tuple[str, str] | None] = Queue()
        self._shutdown_requested = threading.Event()
        self._waiting_for_image = False
        self._image_sub = None

        qos = QoSProfile(depth=10)
        self._status_pub = self.create_publisher(String, 'voice_broadcast_status', qos)
        self._speech_worker = threading.Thread(target=self._speech_worker_loop, daemon=True)
        self._speech_worker.start()

        if self._mode == 'tts_only':
            self.create_subscription(
                String,
                self._config.ai_description_topic,
                self._ai_description_callback,
                qos,
            )
            self.get_logger().info(
                f'TTS-only mode: listening on {self._config.ai_description_topic}'
            )
        else:
            self.create_subscription(
                Int32,
                self._config.sign_topic,
                self._sign_callback,
                qos,
            )
            self.get_logger().info(
                f'Full mode: waiting for {self._config.sign_topic}={self._target_sign}, '
                f'image={self._config.image_topic}'
            )

        self.get_logger().info(
            f'Vision ready={self._config.vision_ready}, '
            f'TTS provider={self._config.resolved_tts_provider()}'
        )

    def _sign_callback(self, msg: Int32) -> None:
        if msg.data != self._target_sign:
            return
        if self._waiting_for_image:
            return
        self._waiting_for_image = True
        if self._image_sub is None:
            self._image_sub = self.create_subscription(
                CompressedImage,
                self._config.image_topic,
                self._image_callback,
                QoSProfile(depth=10),
            )
            self.get_logger().info(f'Subscribed to {self._config.image_topic}')

    def _image_callback(self, msg: CompressedImage) -> None:
        if not self._waiting_for_image:
            return
        self._waiting_for_image = False
        self._teardown_image_sub()

        try:
            cv_image = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001
            self._publish_status(f'image_decode_failed:{exc}')
            self.get_logger().error(f'Image decode failed: {exc}')
            return

        self._run_async('image', lambda: self._service.describe_and_speak_bgr(cv_image))

    def _ai_description_callback(self, msg: String) -> None:
        text = msg.data.strip()
        if not text:
            return
        self.get_logger().info(
            f'[VOICE_BROADCAST] ai_description received chars={len(text)}; '
            'queueing background speech task'
        )
        self._enqueue_speech('text', text)

    def _run_async(self, kind: str, task) -> None:
        def _worker() -> None:
            try:
                self.get_logger().info(f'[VOICE_BROADCAST] worker started kind={kind}')
                result = task()
                if result:
                    if isinstance(result, str):
                        self._publish_status(f'{kind}_ok:{result[:120]}')
                    else:
                        self._publish_status(f'{kind}_ok')
                else:
                    self._publish_status(f'{kind}_failed')
            finally:
                self.get_logger().info(f'[VOICE_BROADCAST] worker finished kind={kind}')

        threading.Thread(target=_worker, daemon=True).start()

    def _enqueue_speech(self, kind: str, text: str) -> None:
        self._speech_queue.put((kind, text))
        self.get_logger().info(
            f'[VOICE_BROADCAST] queued {kind} speech; pending={self._speech_queue.qsize()}'
        )

    def _speech_worker_loop(self) -> None:
        while not self._shutdown_requested.is_set():
            try:
                item = self._speech_queue.get(timeout=0.2)
            except Empty:
                continue
            if item is None:
                return
            kind, text = item
            try:
                self.get_logger().info(f'[VOICE_BROADCAST] speaking queued {kind}')
                if self._service.speak_text(text):
                    self._publish_status(f'{kind}_ok:{text[:120]}')
                else:
                    self._publish_status(f'{kind}_failed')
            finally:
                self._speech_queue.task_done()

    def _teardown_image_sub(self) -> None:
        if self._image_sub is not None:
            self.destroy_subscription(self._image_sub)
            self._image_sub = None

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f'voice_broadcast_status: {status}')

    def destroy_node(self) -> bool:
        self._shutdown_requested.set()
        self._speech_queue.put(None)
        self._speech_worker.join(timeout=1.0)
        self._teardown_image_sub()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VoiceBroadcastNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
