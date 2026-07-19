#!/usr/bin/env python3
"""Asynchronous one-frame vision analysis for the racing robot."""

from __future__ import annotations

import base64
import io
import os
import threading
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String

try:
    from volcenginesdkarkruntime import Ark
except ImportError:  # pragma: no cover - deployment dependency
    Ark = None


class VisionAINode(Node):
    """Cache camera frames and run cloud analysis outside the ROS callbacks."""

    def __init__(self) -> None:
        super().__init__('vision_ai_node')
        self.declare_parameter('mode', 'stage2')
        self.declare_parameter('config_path', '')
        self.declare_parameter('trigger_topic', 'stage2_ai_capture')
        self.declare_parameter('image_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('result_topic', 'ai_description')
        self.declare_parameter('status_topic', 'stage2_ai_status')
        self.declare_parameter('frame_max_age_sec', 1.0)
        self.declare_parameter('request_timeout_sec', 30.0)

        self._config = self._load_config()
        volc = self._config.get('volcengine', {})
        self._api_key = str(volc.get('api_key') or os.environ.get('ARK_API_KEY', '')).strip()
        self._model_id = str(volc.get('model_id') or os.environ.get(
            'ARK_MODEL_ID', 'doubao-seed-1-6-250615'
        )).strip()
        self._prompt = str(volc.get(
            'prompt', '请用一小段话简单描述这张图片中的内容，包括物体、场景、颜色等信息。'
        ))
        self._jpeg_quality = int(self._config.get('image', {}).get('jpeg_quality', 85))
        self._frame_max_age_sec = max(0.1, float(self.get_parameter('frame_max_age_sec').value))
        self._request_timeout_sec = max(1.0, float(self.get_parameter('request_timeout_sec').value))
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame: Any | None = None
        self._latest_frame_time = 0.0
        self._busy = False
        self._busy_lock = threading.Lock()
        self._executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='vision-ai')
        self._result_topic = str(self.get_parameter('result_topic').value)

        self._result_pub = self.create_publisher(
            String, self._result_topic, QoSProfile(depth=10)
        )
        self._status_pub = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), QoSProfile(depth=10)
        )
        image_topic = str(self.get_parameter('image_topic').value)
        self._image_sub = self.create_subscription(
            Image, image_topic, self._image_callback, qos_profile_sensor_data
        )

        mode = str(self.get_parameter('mode').value).strip().lower()
        if mode == 'full':
            target_sign = int(self._config.get('detection', {}).get('target_sign', 9))
            sign_topic = str(self._config.get('detection', {}).get('sign_topic', 'sign4return'))
            self._sign_sub = self.create_subscription(
                Int32, sign_topic, lambda msg: self._trigger_callback(target_sign, msg.data), 10
            )
            self.get_logger().info(
                f'[VISION_AI] mode=full sign_topic={sign_topic} target_sign={target_sign}'
            )
        else:
            trigger_topic = str(self.get_parameter('trigger_topic').value)
            self._trigger_sub = self.create_subscription(
                Empty, trigger_topic, self._stage2_trigger_callback, 10
            )
            self.get_logger().info(
                f'[VISION_AI] mode=stage2 trigger_topic={trigger_topic} '
                f'image_topic={image_topic} max_age={self._frame_max_age_sec:.1f}s'
            )

        if Ark is None:
            self.get_logger().error('[VISION_AI] Ark SDK unavailable; install volcengine-python-sdk[ark]')
        elif not self._api_key:
            self.get_logger().error('[VISION_AI] ARK_API_KEY is empty; cloud analysis disabled')
        else:
            self.get_logger().info(
                f'[VISION_AI] Ark ready model={self._model_id} '
                f'prompt_len={len(self._prompt)}'
            )

    def _load_config(self) -> dict:
        configured = str(self.get_parameter('config_path').value).strip()
        candidates = []
        if configured:
            candidates.append(Path(configured))
        try:
            from ament_index_python.packages import get_package_share_directory
            candidates.append(Path(get_package_share_directory('racing_vision_ai'))
                             / 'config' / 'vision_ai_config.yaml')
        except Exception:
            pass
        candidates.append(Path(__file__).resolve().parents[1] / 'config' / 'vision_ai_config.yaml')
        for path in candidates:
            try:
                with path.open(encoding='utf-8') as stream:
                    config = yaml.safe_load(stream) or {}
                self.get_logger().info(f'[VISION_AI] config loaded: {path}')
                return config
            except (OSError, yaml.YAMLError) as exc:
                self.get_logger().debug(f'[VISION_AI] config unavailable path={path}: {exc}')
        self.get_logger().warn('[VISION_AI] no config file found; using environment/defaults')
        return {}

    def _image_callback(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = frame.copy()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'[VISION_AI] frame decode failed: {exc}')
            return
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
        now = self.get_clock().now().nanoseconds / 1e9
        with self._frame_lock:
            self._latest_frame = frame
            self._latest_frame_time = stamp if stamp > 0.0 else now

    def _stage2_trigger_callback(self, _msg: Empty) -> None:
        self._trigger_callback(None, None)

    def _trigger_callback(self, target_sign: int | None, value: int | None) -> None:
        if target_sign is not None and value != target_sign:
            return
        now = self.get_clock().now().nanoseconds / 1e9
        with self._busy_lock:
            if self._busy:
                self.get_logger().warn('[VISION_AI] trigger ignored: previous request still running')
                self._publish_status('busy')
                return
            self._busy = True
        with self._frame_lock:
            frame = None if self._latest_frame is None else self._latest_frame.copy()
            frame_time = self._latest_frame_time
        age = now - frame_time if frame is not None else float('inf')
        if frame is None or age > self._frame_max_age_sec:
            self.get_logger().error(
                f'[VISION_AI] trigger failed: no fresh frame age={age:.3f}s '
                f'max={self._frame_max_age_sec:.3f}s'
            )
            self._publish_status('capture_failed:no_fresh_frame')
            with self._busy_lock:
                self._busy = False
            return
        self.get_logger().info(
            f'[VISION_AI] frame captured age={age:.3f}s shape={frame.shape}; '
            f'queueing cloud request timeout={self._request_timeout_sec:.1f}s'
        )
        self._publish_status(f'captured:age={age:.3f}s')
        self._executor.submit(self._analyze_worker, frame)

    def _analyze_worker(self, frame: Any) -> None:
        try:
            if Ark is None or not self._api_key:
                self._publish_status('analysis_failed:ark_not_ready')
                return
            started = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info('[VISION_AI] cloud request started')
            encoded = self._encode_image(frame)
            client = Ark(api_key=self._api_key)
            response = client.chat.completions.create(
                model=self._model_id,
                messages=[{'role': 'user', 'content': [
                    {'type': 'image_url', 'image_url': {'url': f'data:image/jpeg;base64,{encoded}'}},
                    {'type': 'text', 'text': self._prompt},
                ]}],
                timeout=self._request_timeout_sec,
            )
            content = response.choices[0].message.content if response.choices else ''
            if not isinstance(content, str):
                content = str(content)
            content = content.strip()
            elapsed = self.get_clock().now().nanoseconds / 1e9 - started
            if not content:
                self.get_logger().error(f'[VISION_AI] cloud response empty elapsed={elapsed:.3f}s')
                self._publish_status('analysis_failed:empty_response')
                return
            result = String()
            result.data = content[:1000]
            self._result_pub.publish(result)
            self.get_logger().info(
                f'[VISION_AI] result published topic={self._result_topic} '
                f'elapsed={elapsed:.3f}s chars={len(result.data)}'
            )
            self._publish_status(f'analysis_ok:elapsed={elapsed:.3f}s:chars={len(result.data)}')
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[VISION_AI] cloud request failed: {type(exc).__name__}: {exc}')
            self._publish_status(f'analysis_failed:{type(exc).__name__}')
        finally:
            with self._busy_lock:
                self._busy = False
            self.get_logger().info('[VISION_AI] background task finished; ready for next trigger')

    def _encode_image(self, frame: Any) -> str:
        ok, encoded = cv2.imencode(
            '.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
        )
        if not ok:
            raise RuntimeError('jpeg_encode_failed')
        return base64.b64encode(encoded.tobytes()).decode('ascii')

    def _publish_status(self, status: str) -> None:
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def destroy_node(self) -> bool:
        self.get_logger().info('[VISION_AI] shutting down executor')
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VisionAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
