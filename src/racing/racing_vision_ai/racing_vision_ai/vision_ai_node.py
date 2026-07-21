#!/usr/bin/env python3
"""Asynchronous one-frame vision analysis for the racing robot."""

from __future__ import annotations

import os
import re
import threading
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, wait
from pathlib import Path
from typing import Any

import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String

from voice_api.env_config import VoiceEnvConfig
from voice_api.vision_analyzer import VisionAnalyzer


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
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('active_phase', 2)
        self.declare_parameter('phase_gated', True)
        self.declare_parameter('streaming_enabled', True)
        self.declare_parameter('stream_sentence_min_chars', 8)

        self._config = self._load_config()
        volc = self._config.get('volcengine', {})
        self._voice_env = VoiceEnvConfig.from_env()
        self._vision_provider, self._api_key, self._model_id = self._resolve_vision_api(volc)
        self._prompt = str(volc.get(
            'prompt', '请用一小段话简单描述这张图片中的内容，包括物体、场景、颜色等信息。'
        ))
        self._max_description_chars = max(1, int(volc.get('max_description_chars', 20)))
        self._streaming_enabled = bool(self.get_parameter('streaming_enabled').value)
        self._stream_sentence_min_chars = max(
            1, int(self.get_parameter('stream_sentence_min_chars').value)
        )
        self._jpeg_quality = int(self._config.get('image', {}).get('jpeg_quality', 85))
        self._image_max_edge_px = max(0, int(self._config.get('image', {}).get('max_edge_px', 448)))
        self._cloud_vision = VisionAnalyzer(
            provider=self._vision_provider,
            api_key=self._api_key,
            model_id=self._model_id,
            prompt=self._prompt,
            jpeg_quality=self._jpeg_quality,
            image_max_edge_px=self._image_max_edge_px,
            request_timeout_sec=float(self.get_parameter('request_timeout_sec').value),
            logger=self.get_logger(),
        )
        self._dashscope_vision: VisionAnalyzer | None = None
        dashscope_key = os.environ.get('DASHSCOPE_API_KEY', '').strip()
        if self._vision_provider != 'dashscope' and dashscope_key:
            self._dashscope_vision = VisionAnalyzer(
                provider='dashscope',
                api_key=dashscope_key,
                model_id=self._voice_env.dashscope_model_id,
                prompt=self._prompt,
                jpeg_quality=self._jpeg_quality,
                image_max_edge_px=self._image_max_edge_px,
                request_timeout_sec=float(self.get_parameter('request_timeout_sec').value),
                logger=self.get_logger(),
            )
        local = self._config.get('local_llama', {})
        self._local_vision: VisionAnalyzer | None = None
        self._local_require_chinese = bool(local.get('require_chinese', True))
        if bool(local.get('enabled', False)):
            self._local_vision = VisionAnalyzer(
                provider='openai_compatible',
                api_key='',
                model_id=str(local.get('model_id', 'SmolVLM-500M-Instruct')),
                prompt=str(local.get('prompt', self._prompt)),
                jpeg_quality=self._jpeg_quality,
                image_max_edge_px=self._image_max_edge_px,
                base_url=str(local.get('base_url', 'http://127.0.0.1:8080/v1/chat/completions')),
                request_timeout_sec=float(local.get('timeout_sec', 12.0)),
                max_tokens=int(local.get('max_tokens', 48)),
                logger=self.get_logger(),
            )
        self._frame_max_age_sec = max(0.1, float(self.get_parameter('frame_max_age_sec').value))
        self._request_timeout_sec = max(1.0, float(self.get_parameter('request_timeout_sec').value))
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame: Any | None = None
        self._latest_frame_time = 0.0
        self._busy = False
        self._busy_lock = threading.Lock()
        self._phase = 0
        self._capture_active = not bool(self.get_parameter('phase_gated').value)
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
        if bool(self.get_parameter('phase_gated').value):
            qos_latched = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.create_subscription(
                Int32, str(self.get_parameter('phase_topic').value), self._phase_callback, qos_latched
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

        if not self._cloud_vision.ready:
            self.get_logger().error(
                f'[VISION_AI] {self._vision_provider} API is not ready; '
                'check the shared voice_driver/.env credentials'
            )
        else:
            self.get_logger().info(
                f'[VISION_AI] provider={self._vision_provider} ready model={self._model_id} '
                f'prompt_len={len(self._prompt)} streaming={self._streaming_enabled}'
            )
        if self._local_vision is not None:
            self.get_logger().info(
                f'[VISION_AI] local race enabled endpoint={local.get("base_url")} '
                f'model={local.get("model_id")} image_max_edge={self._image_max_edge_px}px'
            )

    def _resolve_vision_api(self, volc: dict) -> tuple[str, str, str]:
        """Prefer the shared .env provider, retaining legacy Ark YAML fallback."""
        provider = os.environ.get('VISION_PROVIDER', '').strip().lower()
        if not provider:
            provider = 'ark' if (volc.get('api_key') or os.environ.get('ARK_API_KEY')) else self._voice_env.resolved_vision_provider()

        if provider in {'dashscope', 'bailian', '百炼'}:
            return (
                'dashscope',
                self._voice_env.dashscope_api_key,
                self._voice_env.dashscope_model_id,
            )

        return (
            'ark',
            str(volc.get('api_key') or self._voice_env.ark_api_key).strip(),
            str(volc.get('model_id') or self._voice_env.ark_model_id).strip(),
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
        if not self._capture_active:
            return
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

    def _phase_callback(self, msg: Int32) -> None:
        previous = self._phase
        self._phase = int(msg.data)
        self._capture_active = self._phase == int(self.get_parameter('active_phase').value)
        if previous == self._phase:
            return
        if not self._capture_active:
            with self._frame_lock:
                self._latest_frame = None
                self._latest_frame_time = 0.0
        self.get_logger().info(
            f'[RESOURCE] vision_ai phase={self._phase} capture_active={self._capture_active}'
        )

    def _stage2_trigger_callback(self, _msg: Empty) -> None:
        self._trigger_callback(None, None)

    def _trigger_callback(self, target_sign: int | None, value: int | None) -> None:
        if target_sign is not None and value != target_sign:
            return
        if not self._capture_active:
            self.get_logger().warning(
                f'[VISION_AI] trigger ignored: inactive phase={self._phase}'
            )
            self._publish_status(f'ignored:inactive_phase={self._phase}')
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
            if not self._stream_candidates():
                self._publish_status('analysis_failed:no_vision_provider_ready')
                return
            started = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f'[VISION_AI] stream race candidates={", ".join(name for name, _ in self._stream_candidates())}'
            )
            if self._streaming_enabled:
                content, winner = self._analyze_stream_race(frame)
            else:
                content, winner = self._analyze_race(frame)
            elapsed = self.get_clock().now().nanoseconds / 1e9 - started
            if not content:
                self.get_logger().error(f'[VISION_AI] all vision responses failed elapsed={elapsed:.3f}s')
                self._publish_status('analysis_failed:empty_response')
                return
            description = content.strip()[:self._max_description_chars]
            if not winner.endswith('_stream'):
                self._publish_result(description)
            self.get_logger().info(
                f'[VISION_AI] result published topic={self._result_topic} '
                f'winner={winner} elapsed={elapsed:.3f}s chars={len(description)} '
                f'max_chars={self._max_description_chars}'
            )
            self._publish_status(
                f'analysis_ok:winner={winner}:elapsed={elapsed:.3f}s:chars={len(description)}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[VISION_AI] cloud request failed: {type(exc).__name__}: {exc}')
            self._publish_status(f'analysis_failed:{type(exc).__name__}')
        finally:
            with self._busy_lock:
                self._busy = False
            self.get_logger().info('[VISION_AI] background task finished; ready for next trigger')

    def _stream_candidates(self) -> list[tuple[str, VisionAnalyzer]]:
        candidates: list[tuple[str, VisionAnalyzer]] = []
        if self._cloud_vision.ready:
            candidates.append((self._vision_provider, self._cloud_vision))
        if self._dashscope_vision is not None and self._dashscope_vision.ready:
            candidates.append(('qwen', self._dashscope_vision))
        if self._local_vision is not None and self._local_vision.ready:
            candidates.append(('local', self._local_vision))
        return candidates

    def _analyze_stream_race(self, frame: Any) -> tuple[str | None, str]:
        """Use the first usable stream, then ignore all slower model responses."""
        candidates = self._stream_candidates()
        selected: list[str] = []
        buffers = {name: '' for name, _ in candidates}
        selection_lock = threading.Lock()
        selection_ready = threading.Event()
        pending = ''
        published = 0

        def publish_delta(delta: str) -> None:
            nonlocal pending, published
            pending += delta
            while pending:
                match = re.search(r'[。！？!?；;\n]', pending)
                if match:
                    end = match.end()
                elif len(pending) >= self._stream_sentence_min_chars:
                    end = self._stream_sentence_min_chars
                else:
                    return
                phrase = pending[:end].strip()
                pending = pending[end:]
                if phrase:
                    published += len(phrase)
                    self._publish_result(phrase[:self._max_description_chars])

        def callback_for(name: str):
            def on_delta(delta: str) -> None:
                with selection_lock:
                    if selected and selected[0] != name:
                        return
                    buffers[name] += delta
                    if not selected:
                        if name == 'local' and self._local_require_chinese and not re.search(
                            r'[\u4e00-\u9fff]', buffers[name]
                        ):
                            return
                        selected.append(name)
                        selection_ready.set()
                        self.get_logger().info(f'[VISION_AI] stream race winner={name}')
                    outgoing = buffers[name]
                    buffers[name] = ''
                publish_delta(outgoing)
            return on_delta

        race_executor = ThreadPoolExecutor(max_workers=len(candidates), thread_name_prefix='vision-stream')
        futures = {
            name: race_executor.submit(analyzer.analyze_bgr_stream, frame, callback_for(name))
            for name, analyzer in candidates
        }
        try:
            if not selection_ready.wait(timeout=self._request_timeout_sec):
                self.get_logger().error('[VISION_AI] stream race timed out before first usable delta')
                return None, 'none'
            winner = selected[0]
            content = futures[winner].result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(f'[VISION_AI] {selected[0] if selected else "stream"} failed: {exc}')
            return None, selected[0] if selected else 'none'
        finally:
            for name, future in futures.items():
                if not selected or name != selected[0]:
                    future.cancel()
            race_executor.shutdown(wait=False, cancel_futures=True)
        final_text = (content or '').strip()[:self._max_description_chars]
        remainder = pending.strip()
        if remainder and published < len(final_text):
            self._publish_result(remainder[:self._max_description_chars])
        return final_text or None, f'{selected[0]}_stream'

    def _publish_result(self, text: str) -> None:
        result = String()
        result.data = text
        self._result_pub.publish(result)
        self.get_logger().info(
            f'[VISION_AI] stream phrase published topic={self._result_topic} chars={len(text)}'
        )

    def _analyze_race(self, frame: Any) -> tuple[str | None, str]:
        """Return the first non-empty result from concurrent local/cloud requests."""
        candidates = self._stream_candidates()
        if not candidates:
            return None, 'none'

        race_executor = ThreadPoolExecutor(max_workers=len(candidates), thread_name_prefix='vision-race')
        futures: dict[Future, str] = {
            race_executor.submit(analyzer.analyze_bgr, frame): name
            for name, analyzer in candidates
        }
        pending = set(futures)
        try:
            while pending:
                completed, pending = wait(pending, return_when=FIRST_COMPLETED)
                for future in completed:
                    name = futures[future]
                    try:
                        content = future.result()
                    except Exception as exc:  # noqa: BLE001
                        self.get_logger().error(f'[VISION_AI] {name} race task failed: {exc}')
                        continue
                    if content and content.strip():
                        if (
                            name == 'local'
                            and self._local_require_chinese
                            and not re.search(r'[\u4e00-\u9fff]', content)
                        ):
                            self.get_logger().warn(
                                '[VISION_AI] local result has no Chinese characters; waiting for cloud'
                            )
                            continue
                        for loser in pending:
                            loser.cancel()
                        if pending:
                            self.get_logger().info(
                                f'[VISION_AI] winner={name}; cancellation requested for '
                                f'{", ".join(futures[loser] for loser in pending)}'
                            )
                        return content, name
                    self.get_logger().warn(f'[VISION_AI] {name} returned empty; waiting for peer')
            return None, 'none'
        finally:
            # A running HTTP request cannot be forcibly killed by Python futures. Its result is ignored.
            race_executor.shutdown(wait=False, cancel_futures=True)

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
