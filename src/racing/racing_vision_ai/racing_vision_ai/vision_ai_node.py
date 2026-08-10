#!/usr/bin/env python3
"""Asynchronous one-frame vision analysis for the racing robot."""

from __future__ import annotations

from datetime import datetime
import fcntl
import os
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

from racing_common.process_lifecycle import install_parent_death_signal
from voice_api.vision_analyzer import VisionAnalyzer


def _resolve_stage2_dir():
    session_root = os.environ.get('RACING_SESSION_ROOT', '').strip()
    if session_root:
        return os.path.join(session_root, 'stage2')
    dev_ws = os.environ.get('DEV_WS', '').strip()
    if dev_ws:
        return os.path.join(dev_ws, 'log', 'stage2')
    cwd = os.getcwd()
    if os.path.isdir(os.path.join(cwd, 'src', 'racing')):
        return os.path.join(cwd, 'log', 'stage2')
    return os.path.join(os.path.expanduser('~'), 'dev_ws', 'log', 'stage2')


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
        self.declare_parameter('prewarm_task_topic', 'competition_qr_task')
        self.declare_parameter('stream_sentence_min_chars', 4)
        self.declare_parameter(
            'env_path', '/home/sunrise/dev_ws/src/racing/racing_vision_ai/config/.env'
        )

        self._config = self._load_config()
        self._env_values = self._load_env_values()
        models = self._config.get('vision_models', {})
        response_config = self._config.get('response', {})
        if not isinstance(response_config, dict):
            response_config = {}
        self._max_description_chars = max(
            1, int(response_config.get('max_description_chars', 20))
        )
        self._streaming_enabled = self._config_bool(
            response_config.get('streaming_enabled', False)
        )
        self._stream_sentence_min_chars = max(
            1, int(self.get_parameter('stream_sentence_min_chars').value)
        )
        self._jpeg_quality = int(self._config.get('image', {}).get('jpeg_quality', 85))
        self._image_max_edge_px = max(0, int(self._config.get('image', {}).get('max_edge_px', 448)))
        self._image_crop_top_ratio = min(
            0.20, max(0.0, float(self._config.get('image', {}).get('crop_top_ratio', 0.0)))
        )
        capture_config = self._config.get('capture', {})
        if not isinstance(capture_config, dict):
            capture_config = {}
        self._stage2_dir = _resolve_stage2_dir()
        self._capture_image_path = Path(
            os.path.join(self._stage2_dir, 'ai_capture.jpg')
        )
        self._stage2_log_path = Path(os.path.join(self._stage2_dir, 'latest.log'))
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
        self._local_warmup_vision: VisionAnalyzer | None = None
        self._local_require_chinese = bool(local.get('require_chinese', True))
        self._local_image_warmup_enabled = bool(local.get('image_warmup_enabled', False))
        self._local_image_warmup_max_edge_px = max(
            16, int(local.get('image_warmup_max_edge_px', 96))
        )
        self._local_image_warmup_timeout_sec = max(
            0.1, float(local.get('image_warmup_request_timeout_sec', 45.0))
        )
        self._local_image_warmup_frame_wait_sec = max(
            0.1, float(local.get('image_warmup_frame_wait_sec', 8.0))
        )
        self._local_image_warmup_max_tokens = max(
            1, int(local.get('image_warmup_max_tokens', 1))
        )
        self._local_server_prewarm_on_startup = bool(
            local.get('server_prewarm_on_startup', False)
        )
        self._local_image_warmup_on_task = bool(local.get('image_warmup_on_task', False))
        self._local_warmup_lock = threading.Lock()
        self._local_warmup_requested = False
        self._local_warmup_reason = ''
        self._local_warmup_started = False
        self._local_warmup_complete = False
        self._local_server: subprocess.Popen | None = None
        self._local_server_log = None
        self._local_server_lock = threading.Lock()
        self._local_server_ready = threading.Event()
        if bool(local.get('enabled', False)):
            self._local_vision = self._create_vision_analyzer(local)
            warmup_model = dict(local)
            warmup_model.update({
                'prompt': '请只回答好。',
                'max_tokens': self._local_image_warmup_max_tokens,
                'timeout_sec': self._local_image_warmup_timeout_sec,
            })
            self._local_warmup_vision = self._create_vision_analyzer(warmup_model)
        self._frame_max_age_sec = max(0.1, float(self.get_parameter('frame_max_age_sec').value))
        self._request_timeout_sec = max(1.0, float(self.get_parameter('request_timeout_sec').value))
        self._bridge = CvBridge()
        self._frame_lock = threading.Lock()
        self._latest_frame: Any | None = None
        self._latest_frame_time = 0.0
        self._busy = False
        self._busy_lock = threading.Lock()
        self._shutdown_requested = threading.Event()
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
            self.create_subscription(
                String, str(self.get_parameter('prewarm_task_topic').value),
                self._prewarm_task_callback, qos_latched
            )

        trigger_topic = str(self.get_parameter('trigger_topic').value)
        self._trigger_sub = self.create_subscription(
            Empty, trigger_topic, self._stage2_trigger_callback, 10
        )
        self.get_logger().info(
            f'[VISION_AI] trigger_topic={trigger_topic} '
            f'image_topic={image_topic} max_age={self._frame_max_age_sec:.1f}s '
            f'capture_path={self._capture_image_path} '
            f'streaming_enabled={self._streaming_enabled}'
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
            if self._local_server_prewarm_on_startup:
                self._start_local_server('node_startup')

    def _create_vision_analyzer(self, model: dict) -> VisionAnalyzer:
        """Build one contender from the uniform YAML model schema."""
        api_key = str(model.get('api_key', '')).strip()
        api_key_env = str(model.get('api_key_env', '')).strip()
        if not api_key and api_key_env:
            api_key = os.environ.get(api_key_env, '').strip()
            if not api_key:
                api_key = self._env_values.get(api_key_env, '').strip()
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

    def _load_env_values(self) -> dict[str, str]:
        """Load private credentials without placing them in the installed YAML."""
        path = Path(str(self.get_parameter('env_path').value)).expanduser()
        try:
            values: dict[str, str] = {}
            with path.open(encoding='utf-8') as stream:
                for raw_line in stream:
                    line = raw_line.strip()
                    if not line or line.startswith('#'):
                        continue
                    if line.startswith('export '):
                        line = line[7:].lstrip()
                    key, separator, value = line.partition('=')
                    key = key.strip()
                    if not separator or not key:
                        continue
                    value = value.strip()
                    if len(value) >= 2 and value[0] == value[-1] and value[0] in {'\"', "'"}:
                        value = value[1:-1]
                    values[key] = value
            self.get_logger().info(
                f'[VISION_AI] credentials loaded from {path}: entries={len(values)}'
            )
            return values
        except OSError as exc:
            self.get_logger().warning(
                f'[VISION_AI] credentials unavailable path={path}: {type(exc).__name__}'
            )
            return {}

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

    @staticmethod
    def _config_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {'1', 'true', 'yes', 'on'}
        return bool(value)

    def _image_callback(self, msg: Image) -> None:
        if not self._should_cache_frame():
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

    def _prepare_model_frame(self, frame: Any) -> Any:
        """Apply the deliberately conservative crop shared by warmup and inference."""
        if self._image_crop_top_ratio <= 0.0:
            return frame
        height = int(frame.shape[0])
        crop_rows = min(height - 1, round(height * self._image_crop_top_ratio))
        if crop_rows <= 0:
            return frame
        return frame[crop_rows:, :].copy()

    def _phase_callback(self, msg: Int32) -> None:
        previous = self._phase
        self._phase = int(msg.data)
        self._capture_active = self._phase == int(self.get_parameter('active_phase').value)
        if self._capture_active:
            self._stage2_log_active = True
        if previous == self._phase:
            return
        if self._capture_active:
            self._start_local_server('phase2')
            self._request_local_image_warmup('phase2_fallback')
        if not self._capture_active and not self._should_cache_frame():
            with self._frame_lock:
                self._latest_frame = None
                self._latest_frame_time = 0.0
        self.get_logger().info(
            f'[RESOURCE] vision_ai phase={self._phase} capture_active={self._capture_active}'
        )
        self._write_stage2_log(
            f'[PHASE] phase={self._phase} capture_active={self._capture_active}'
        )

    def _prewarm_task_callback(self, msg: String) -> None:
        """Start visual prefill as soon as Stage1 has resolved the QR task."""
        task = msg.data.strip()
        if not task or not self._local_image_warmup_on_task:
            return
        self._request_local_image_warmup(f'qr_task:{task}')

    def _should_cache_frame(self) -> bool:
        if self._capture_active:
            return True
        with self._local_warmup_lock:
            return self._local_warmup_requested and not self._local_warmup_complete

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

    def _start_local_server(self, reason: str) -> None:
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
            log_path = Path(os.path.join(self._stage2_dir, 'local-smolvlm.log'))
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
            self.get_logger().info(f'[RESOURCE] local VLM starting prewarm reason={reason}')
            self._write_stage2_log(f'[RESOURCE] local VLM starting prewarm reason={reason}')
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
                        self._start_local_image_warmup()
                        return
            except (urllib.error.HTTPError, urllib.error.URLError, TimeoutError):
                pass
            time.sleep(0.25)
        self.get_logger().error('[RESOURCE] local VLM Phase 2 prewarm timed out')
        self._publish_status('local_vlm_failed:warmup_timeout')

    def _start_local_image_warmup(self) -> None:
        """Warm the local VLM's multimodal path without publishing a user result."""
        if (
            not self._local_image_warmup_enabled
            or self._local_warmup_vision is None
        ):
            return
        with self._local_warmup_lock:
            if (not self._local_warmup_requested or self._local_warmup_started
                    or self._local_warmup_complete):
                return
            self._local_warmup_started = True
        threading.Thread(target=self._run_local_image_warmup, daemon=True).start()

    def _request_local_image_warmup(self, reason: str) -> None:
        if self._local_vision is None:
            return
        with self._local_warmup_lock:
            if self._local_warmup_complete:
                return
            self._local_warmup_requested = True
            self._local_warmup_reason = reason
        self.get_logger().info(f'[WARMUP] local image prefill requested reason={reason}')
        self._write_stage2_log(f'[WARMUP] requested reason={reason}')
        self._start_local_server(reason)
        if self._local_server_ready.is_set():
            self._start_local_image_warmup()

    def _run_local_image_warmup(self) -> None:
        deadline = time.monotonic() + self._local_image_warmup_frame_wait_sec
        frame = None
        while time.monotonic() < deadline:
            with self._frame_lock:
                if self._latest_frame is not None:
                    frame = self._latest_frame.copy()
            if frame is not None:
                break
            time.sleep(0.03)
        if frame is None:
            with self._local_warmup_lock:
                self._local_warmup_started = False
            self._write_stage2_log('[WARMUP] skipped: no camera frame before deadline')
            return

        model_frame = self._prepare_model_frame(frame)
        height, width = model_frame.shape[:2]
        longest_edge = max(height, width)
        if longest_edge > self._local_image_warmup_max_edge_px:
            scale = self._local_image_warmup_max_edge_px / longest_edge
            model_frame = cv2.resize(
                model_frame,
                (max(1, round(width * scale)), max(1, round(height * scale))),
                interpolation=cv2.INTER_AREA,
            )
        started = time.monotonic()
        try:
            content = self._local_warmup_vision.analyze_bgr(model_frame)
            elapsed = time.monotonic() - started
            if not content:
                self.get_logger().warning(
                    f'[WARMUP] local image preflight returned no result elapsed={elapsed:.3f}s'
                )
                self._write_stage2_log(
                    f'[WARMUP] empty_result elapsed={elapsed:.3f}s shape={model_frame.shape}'
                )
                return
            with self._local_warmup_lock:
                self._local_warmup_complete = True
            self.get_logger().info(
                f'[WARMUP] local image preflight complete elapsed={elapsed:.3f}s '
                f'shape={model_frame.shape}'
            )
            self._write_stage2_log(
                f'[WARMUP] complete elapsed={elapsed:.3f}s shape={model_frame.shape}'
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warning(f'[WARMUP] local image preflight failed: {exc}')
            self._write_stage2_log(f'[WARMUP] failed={type(exc).__name__}')

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
        self.get_logger().info('[RESOURCE] local VLM stopped')
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
        model_frame = self._prepare_model_frame(frame)
        with self._local_warmup_lock:
            warmup_state = (
                'complete' if self._local_warmup_complete
                else 'running' if self._local_warmup_started
                else 'disabled'
            )
        self.get_logger().info(
            f'[VISION_AI] frame captured age={age:.3f}s source_shape={frame.shape} '
            f'model_shape={model_frame.shape} local_warmup={warmup_state}; '
            f'queueing vision request timeout={self._request_timeout_sec:.1f}s'
        )
        self._save_capture_image(model_frame)
        self._publish_status(f'captured:age={age:.3f}s')
        self._executor.submit(self._analyze_worker, model_frame, time.monotonic())

    def _save_capture_image(self, frame: Any) -> None:
        """Persist the cropped source frame used to build provider requests."""
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

    def _analyze_worker(self, frame: Any, captured_at: float) -> None:
        try:
            if not self._stream_candidates():
                self._publish_status('analysis_failed:no_vision_provider_ready')
                return
            started = time.monotonic()
            mode = 'stream' if self._streaming_enabled else 'complete'
            self.get_logger().info(
                f'[VISION_AI] {mode} race candidates='
                f'{", ".join(name for name, _ in self._stream_candidates())}'
            )
            self._write_stage2_log(
                f'[RACE] mode={mode} '
                f'candidates={", ".join(name for name, _ in self._stream_candidates())} '
                f'capture_to_race={started - captured_at:.3f}s'
            )
            if self._streaming_enabled:
                content, winner = self._analyze_stream_race(frame, captured_at)
            else:
                content, winner = self._analyze_race(frame, captured_at)
            if self._shutdown_requested.is_set():
                self.get_logger().info('[VISION_AI] analysis cancelled by node shutdown')
                return
            elapsed = time.monotonic() - started
            if not content:
                self.get_logger().error(f'[VISION_AI] all vision responses failed elapsed={elapsed:.3f}s')
                self._publish_status('analysis_failed:empty_response')
                return
            description = content.strip()[:self._max_description_chars]
            if not winner.endswith('_stream'):
                self._publish_result(description)
            self._write_stage2_log(
                f'[RESULT] winner={winner} elapsed={elapsed:.3f}s '
                f'chars={len(description)} raw_chars={len(content.strip())} '
                f'text="{self._log_safe_text(description)}"'
            )
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

    def _analyze_stream_race(self, frame: Any, captured_at: float) -> tuple[str | None, str]:
        """Publish the first usable stream, then cancel the losing requests."""
        candidates = self._stream_candidates()
        selected: list[str] = []
        buffers = {name: '' for name, _ in candidates}
        selection_lock = threading.Lock()
        selection_ready = threading.Event()
        pending = ''
        published = 0
        local_timing_lock = threading.Lock()
        local_request_started = 0.0
        local_first_delta_logged = False

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
                nonlocal local_first_delta_logged
                if name == 'local':
                    with local_timing_lock:
                        if not local_first_delta_logged:
                            local_first_delta_logged = True
                            now = time.monotonic()
                            self._write_stage2_log(
                                '[LOCAL_TIMING] first_delta '
                                f'capture_to_first={now - captured_at:.3f}s '
                                f'request_to_first={now - local_request_started:.3f}s'
                            )
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

        def run_candidate(name: str, analyzer: VisionAnalyzer) -> str | None:
            nonlocal local_request_started
            if name != 'local':
                return analyzer.analyze_bgr_stream(frame, callback_for(name))
            with local_timing_lock:
                local_request_started = time.monotonic()
            self._write_stage2_log(
                '[LOCAL_TIMING] request_started '
                f'capture_to_request={local_request_started - captured_at:.3f}s'
            )
            content = analyzer.analyze_bgr_stream(frame, callback_for(name))
            completed = time.monotonic()
            self._write_stage2_log(
                '[LOCAL_TIMING] request_complete '
                f'capture_to_complete={completed - captured_at:.3f}s '
                f'request_elapsed={completed - local_request_started:.3f}s '
                f'chars={len((content or "").strip())}'
            )
            return content

        race_executor = ThreadPoolExecutor(max_workers=len(candidates), thread_name_prefix='vision-stream')
        futures = {
            name: race_executor.submit(run_candidate, name, analyzer)
            for name, analyzer in candidates
        }
        try:
            wait_timeout = (
                None if self._local_no_timeout and any(name == 'local' for name, _ in candidates)
                else self._request_timeout_sec
            )
            while not selection_ready.wait(timeout=0.1):
                if self._shutdown_requested.is_set():
                    return None, 'shutdown'
                if wait_timeout is not None:
                    wait_timeout -= 0.1
                    if wait_timeout <= 0.0:
                        break
            if not selection_ready.is_set():
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
            f'[VISION_AI] result published topic={self._result_topic} chars={len(text)}'
        )

    @staticmethod
    def _log_safe_text(text: str) -> str:
        return (
            text.replace('\\', '\\\\')
            .replace('\r', '\\r')
            .replace('\n', '\\n')
            .replace('\t', '\\t')
            .replace('"', '\\"')
        )

    def _analyze_race(self, frame: Any, captured_at: float) -> tuple[str | None, str]:
        """Return the first non-empty result from concurrent local/cloud requests."""
        candidates = self._stream_candidates()
        if not candidates:
            return None, 'none'

        def run_candidate(name: str, analyzer: VisionAnalyzer) -> str | None:
            if name != 'local':
                return analyzer.analyze_bgr(frame)
            started = time.monotonic()
            self._write_stage2_log(
                '[LOCAL_TIMING] request_started '
                f'capture_to_request={started - captured_at:.3f}s'
            )
            content = analyzer.analyze_bgr(frame)
            completed = time.monotonic()
            self._write_stage2_log(
                '[LOCAL_TIMING] request_complete '
                f'capture_to_complete={completed - captured_at:.3f}s '
                f'request_elapsed={completed - started:.3f}s '
                f'chars={len((content or "").strip())}'
            )
            return content

        race_executor = ThreadPoolExecutor(max_workers=len(candidates), thread_name_prefix='vision-race')
        futures: dict[Future, str] = {
            race_executor.submit(run_candidate, name, analyzer): name
            for name, analyzer in candidates
        }
        pending = set(futures)
        try:
            while pending:
                completed, pending = wait(pending, return_when=FIRST_COMPLETED)
                if self._shutdown_requested.is_set():
                    return None, 'shutdown'
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
        self._shutdown_requested.set()
        with self._busy_lock:
            busy = self._busy
        if busy:
            self.get_logger().info(
                '[VISION_AI] shutdown cancels pending image analysis and stops local VLM'
            )
            self._write_stage2_log('[RESOURCE] node shutdown cancelled pending analysis')
        # Local inference may intentionally have no request timeout. Stopping
        # its server first releases the blocked HTTP call before executor tear-down.
        self._stop_local_server()
        self._executor.shutdown(wait=False, cancel_futures=True)
        return super().destroy_node()


def main(args=None) -> None:
    install_parent_death_signal()
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
