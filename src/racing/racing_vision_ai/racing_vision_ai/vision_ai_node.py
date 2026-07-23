#!/usr/bin/env python3
"""Asynchronous one-frame vision analysis for the racing robot."""

from __future__ import annotations

from datetime import datetime
import fcntl
import re
import subprocess
import threading
import time
import urllib.error
import urllib.request
from concurrent.futures import FIRST_COMPLETED, Future, ThreadPoolExecutor, wait
from pathlib import Path
from typing import Any

import cv2
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Empty, Int32, String

from voice_api.vision_analyzer import VisionAnalyzer


class VisionAINode(Node):
    """Cache camera frames and run cloud analysis outside the ROS callbacks."""

    def __init__(self) -> None:
        super().__init__('vision_ai_node')
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
        self.declare_parameter('mission_state_topic', 'stage3_state')
        self.declare_parameter('streaming_enabled', True)
        self.declare_parameter('stream_sentence_min_chars', 8)

        self._config = self._load_config()
        models = self._config.get('vision_models', {})
        self._max_description_chars = max(
            1, int(self._config.get('response', {}).get('max_description_chars', 20))
        )
        self._streaming_enabled = bool(self.get_parameter('streaming_enabled').value)
        self._stream_sentence_min_chars = max(
            1, int(self.get_parameter('stream_sentence_min_chars').value)
        )
        self._jpeg_quality = int(self._config.get('image', {}).get('jpeg_quality', 85))
        self._image_max_edge_px = max(0, int(self._config.get('image', {}).get('max_edge_px', 448)))
        capture_config = self._config.get('capture', {})
        if not isinstance(capture_config, dict):
            capture_config = {}
        self._capture_image_path = Path(str(capture_config.get(
            'save_path', '/home/sunrise/dev_ws/log/competition_stage2/ai_capture.jpg'
        ))).expanduser()
        self._stage2_log_path = self._capture_image_path.with_name('latest.log')
        self._stage2_log_active = False
        self._vision_models: dict[str, VisionAnalyzer] = {}
        for name, model in models.items():
            if name == 'local' or not isinstance(model, dict) or not bool(model.get('enabled', False)):
                continue
            self._vision_models[name] = self._create_vision_analyzer(model)
        local = models.get('local', {})
        if not isinstance(local, dict):
            local = {}
        self._local_config = local
        self._local_no_timeout = float(local.get(
            'timeout_sec', self.get_parameter('request_timeout_sec').value
        )) <= 0.0
        self._local_vision: VisionAnalyzer | None = None
        self._local_require_chinese = bool(local.get('require_chinese', True))
        self._local_server: subprocess.Popen | None = None
        self._local_server_log = None
        self._local_server_lock = threading.Lock()
        self._local_server_ready = threading.Event()
        if bool(local.get('enabled', False)):
            self._local_vision = self._create_vision_analyzer(local)
        self._frame_max_age_sec = max(0.1, float(self.get_parameter('frame_max_age_sec').value))
        self._request_timeout_sec = max(1.0, float(self.get_parameter('request_timeout_sec').value))
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame: Any | None = None
        self._latest_frame_time = 0.0
        self._busy = False
        self._busy_lock = threading.Lock()
        self._mission_complete = False
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
            self.create_subscription(
                String, str(self.get_parameter('mission_state_topic').value),
                self._mission_state_callback, qos_latched
            )

        trigger_topic = str(self.get_parameter('trigger_topic').value)
        self._trigger_sub = self.create_subscription(
            Empty, trigger_topic, self._stage2_trigger_callback, 10
        )
        self.get_logger().info(
            f'[VISION_AI] trigger_topic={trigger_topic} '
            f'image_topic={image_topic} max_age={self._frame_max_age_sec:.1f}s '
            f'capture_path={self._capture_image_path}'
        )

        for name, analyzer in self._vision_models.items():
            self.get_logger().info(
                f'[VISION_AI] model={name} provider={analyzer.provider} ready={analyzer.ready} '
                f'streaming={self._streaming_enabled}'
            )
        if self._local_vision is not None:
            self.get_logger().info(
                f'[VISION_AI] local race enabled endpoint={local.get("base_url")} '
                f'model={local.get("model_id")} image_max_edge={self._image_max_edge_px}px'
            )

    def _create_vision_analyzer(self, model: dict) -> VisionAnalyzer:
        """Build one contender from the uniform YAML model schema."""
        api_key = str(model.get('api_key', '')).strip()
        thinking_enabled = model.get('thinking_enabled')
        if not isinstance(thinking_enabled, bool):
            thinking_enabled = None
        return VisionAnalyzer(
            provider=str(model.get('provider', '')).strip(),
            api_key=api_key,
            model_id=str(model.get('model_id', '')).strip(),
            prompt=str(model.get('prompt', '')).strip(),
            jpeg_quality=self._jpeg_quality,
            image_max_edge_px=self._image_max_edge_px,
            base_url=str(model.get('base_url', '')).strip(),
            request_timeout_sec=float(model.get('timeout_sec', self.get_parameter('request_timeout_sec').value)),
            max_tokens=int(model.get('max_tokens', 0)) or None,
            thinking_enabled=thinking_enabled,
            logger=self.get_logger(),
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
        if self._capture_active:
            self._stage2_log_active = True
        if previous == self._phase:
            return
        if self._capture_active:
            self._start_local_server()
        if not self._capture_active:
            with self._frame_lock:
                self._latest_frame = None
                self._latest_frame_time = 0.0
        self.get_logger().info(
            f'[RESOURCE] vision_ai phase={self._phase} capture_active={self._capture_active}'
        )
        self._write_stage2_log(
            f'[PHASE] phase={self._phase} capture_active={self._capture_active}'
        )

    def _mission_state_callback(self, msg: String) -> None:
        if msg.data.strip().lower() != 'complete' or self._mission_complete:
            return
        self._mission_complete = True
        self.get_logger().info(
            '[RESOURCE] competition complete; local VLM will stop after pending analysis'
        )
        self._stop_local_server_if_idle()

    def _stop_local_server_if_idle(self) -> None:
        with self._busy_lock:
            busy = self._busy
        if busy:
            return
        self._stop_local_server()

    def _start_local_server(self) -> None:
        if self._local_vision is None or not bool(
                self._local_config.get('manage_process', False)):
            return
        with self._local_server_lock:
            if self._local_server is not None and self._local_server.poll() is None:
                return
            server_path = Path(str(self._local_config.get('server_path', '')).strip())
            model_path = Path(str(self._local_config.get('model_path', '')).strip())
            mmproj_path = Path(str(self._local_config.get('mmproj_path', '')).strip())
            missing = [str(path) for path in (server_path, model_path, mmproj_path)
                       if not path.is_file()]
            if missing:
                self.get_logger().error(
                    f'[RESOURCE] local VLM not started; missing: {", ".join(missing)}'
                )
                self._publish_status('local_vlm_failed:missing_files')
                return
            log_path = Path(str(self._local_config.get('log_path', '')).strip())
            try:
                log_path.parent.mkdir(parents=True, exist_ok=True)
                self._local_server_log = log_path.open('a', encoding='utf-8')
                command = [
                    str(server_path),
                    '--host', str(self._local_config.get('host', '127.0.0.1')),
                    '--port', str(int(self._local_config.get('port', 8080))),
                    '--model', str(model_path),
                    '--mmproj', str(mmproj_path),
                    '--threads', str(max(1, int(self._local_config.get('threads', 4)))),
                    '--ctx-size', str(max(512, int(self._local_config.get('context_size', 2048)))),
                    '--n-gpu-layers', '0',
                ]
                self._local_server = subprocess.Popen(
                    command,
                    stdout=self._local_server_log,
                    stderr=subprocess.STDOUT,
                    start_new_session=True,
                )
            except (OSError, ValueError) as exc:
                self.get_logger().error(f'[RESOURCE] local VLM start failed: {exc}')
                self._publish_status('local_vlm_failed:start')
                if self._local_server_log is not None:
                    self._local_server_log.close()
                    self._local_server_log = None
                return
            self._local_server_ready.clear()
            self.get_logger().info('[RESOURCE] local VLM starting for Phase 2 prewarm')
            self._write_stage2_log('[RESOURCE] local VLM starting for Phase 2 prewarm')
            threading.Thread(target=self._wait_for_local_server_ready, daemon=True).start()

    def _wait_for_local_server_ready(self) -> None:
        timeout_sec = max(1.0, float(self._local_config.get('warmup_timeout_sec', 25.0)))
        host = str(self._local_config.get('host', '127.0.0.1'))
        port = int(self._local_config.get('port', 8080))
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._local_server_lock:
                process = self._local_server
            if process is None or process.poll() is not None:
                self.get_logger().error('[RESOURCE] local VLM exited during Phase 2 prewarm')
                self._publish_status('local_vlm_failed:exited')
                return
            try:
                with urllib.request.urlopen(
                    f'http://{host}:{port}/health', timeout=1.0
                ) as response:
                    if 200 <= response.status < 300:
                        self._local_server_ready.set()
                        self.get_logger().info('[RESOURCE] local VLM ready for Phase 2 capture')
                        self._write_stage2_log('[RESOURCE] local VLM ready for capture')
                        self._publish_status('local_vlm_ready')
                        return
            except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError):
                pass
            time.sleep(0.25)
        self.get_logger().error('[RESOURCE] local VLM Phase 2 prewarm timed out')
        self._publish_status('local_vlm_failed:warmup_timeout')

    def _stop_local_server(self) -> None:
        self._local_server_ready.clear()
        with self._local_server_lock:
            process = self._local_server
            self._local_server = None
        if process is None:
            return
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        if self._local_server_log is not None:
            self._local_server_log.close()
            self._local_server_log = None
        self.get_logger().info('[RESOURCE] local VLM stopped outside Phase 2')
        self._write_stage2_log('[RESOURCE] local VLM stopped')

    def _stage2_trigger_callback(self, _msg: Empty) -> None:
        self._trigger_callback()

    def _trigger_callback(self) -> None:
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
        self._save_capture_image(frame)
        self._publish_status(f'captured:age={age:.3f}s')
        self._executor.submit(self._analyze_worker, frame)

    def _save_capture_image(self, frame: Any) -> None:
        """Persist the exact frame sent to the vision providers for inspection."""
        try:
            self._capture_image_path.parent.mkdir(parents=True, exist_ok=True)
            encoded, data = cv2.imencode(
                '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, self._jpeg_quality]
            )
            if not encoded:
                raise RuntimeError('cv2.imencode returned false')
            temporary_path = self._capture_image_path.with_suffix('.jpg.tmp')
            temporary_path.write_bytes(data.tobytes())
            temporary_path.replace(self._capture_image_path)
            self.get_logger().info(
                f'[VISION_AI] capture image saved: {self._capture_image_path}'
            )
            self._write_stage2_log(f'[CAPTURE] image_saved={self._capture_image_path}')
        except (OSError, RuntimeError, cv2.error) as exc:
            self.get_logger().warning(f'[VISION_AI] capture image save failed: {exc}')
            self._write_stage2_log(f'[CAPTURE] image_save_failed={type(exc).__name__}: {exc}')

    def _analyze_worker(self, frame: Any) -> None:
        try:
            if not self._stream_candidates():
                self._publish_status('analysis_failed:no_vision_provider_ready')
                return
            started = self.get_clock().now().nanoseconds / 1e9
            self.get_logger().info(
                f'[VISION_AI] stream race candidates={", ".join(name for name, _ in self._stream_candidates())}'
            )
            self._write_stage2_log(
                f'[RACE] candidates={", ".join(name for name, _ in self._stream_candidates())}'
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
            self._write_stage2_log('[RACE] background task finished')
            if self._mission_complete:
                self._stop_local_server_if_idle()

    def _stream_candidates(self) -> list[tuple[str, VisionAnalyzer]]:
        candidates: list[tuple[str, VisionAnalyzer]] = []
        candidates.extend(
            (name, analyzer) for name, analyzer in self._vision_models.items() if analyzer.ready
        )
        if (self._local_vision is not None and self._local_vision.ready
                and self._local_server_ready.is_set()):
            candidates.append(('local', self._local_vision))
        return candidates

    def _analyze_stream_race(self, frame: Any) -> tuple[str | None, str]:
        """Publish the first usable stream, then cancel the losing requests."""
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
                        self._write_stage2_log(f'[RACE] stream_winner={name}')
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
            wait_timeout = (
                None if self._local_no_timeout and any(name == 'local' for name, _ in candidates)
                else self._request_timeout_sec
            )
            if not selection_ready.wait(timeout=wait_timeout):
                self.get_logger().error('[VISION_AI] stream race timed out before first usable delta')
                return None, 'none'
            winner = selected[0]
            for name, future in futures.items():
                if name != winner:
                    future.cancel()
            if winner != 'local':
                self._stop_local_server()
            self.get_logger().info(
                f'[VISION_AI] winner={winner}; cancellation requested for '
                f'{", ".join(name for name in futures if name != winner)}'
            )
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
        self._write_stage2_log(f'[STATUS] {status}')
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)

    def _write_stage2_log(self, message: str) -> None:
        """Append AI diagnostics without competing with Stage2's session owner."""
        if not self._stage2_log_active:
            return
        try:
            self._stage2_log_path.parent.mkdir(parents=True, exist_ok=True)
            with self._stage2_log_path.open('a', encoding='utf-8') as stream:
                fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
                try:
                    stamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3]
                    stream.write(f'{stamp} [VISION_AI] {message}\n')
                    stream.flush()
                finally:
                    fcntl.flock(stream.fileno(), fcntl.LOCK_UN)
        except OSError as exc:
            self.get_logger().warning(f'[VISION_AI] stage2 log append failed: {exc}')

    def destroy_node(self) -> bool:
        self.get_logger().info('[VISION_AI] shutting down executor')
        self._executor.shutdown(wait=False, cancel_futures=True)
        self._stop_local_server()
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
