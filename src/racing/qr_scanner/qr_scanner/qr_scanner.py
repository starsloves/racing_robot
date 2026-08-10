import os
import time
import threading

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from racing_common.process_lifecycle import install_parent_death_signal
from racing_common.session_file_log import SessionFileLog
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String
from std_srvs.srv import Trigger

class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner')

        self.declare_parameter('camera_topic', '/image')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('result_topic', 'qr_scan_result')
        self.declare_parameter('stage1_state_topic', 'stage1_state')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('scan_task_phase', 1)
        self.declare_parameter('crop_top_ratio', 0.25)
        self.declare_parameter('crop_top_px', 80)
        self.declare_parameter('upscale_factor', 1.0)
        self.declare_parameter('detection_order', 'crop_only')
        self.declare_parameter('min_publish_interval', 1.0)
        self.declare_parameter('diagnostics_enabled', True)
        self.declare_parameter('diagnostics_interval_sec', 1.0)
        self.declare_parameter('diagnostics_log_subdir', 'stage1')
        self.declare_parameter('diagnostics_log_filename', 'latest.log')
        self.declare_parameter('debug_image_enabled', True)
        self.declare_parameter('debug_image_filename', 'qr_latest.jpg')
        self.declare_parameter('wechat_detect_prototxt', '')
        self.declare_parameter('wechat_detect_caffemodel', '')
        self.declare_parameter('wechat_sr_prototxt', '')
        self.declare_parameter('wechat_sr_caffemodel', '')

        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        self.result_topic = self.get_parameter('result_topic').value
        self.stage1_state_topic = self.get_parameter('stage1_state_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_task_phase = int(self.get_parameter('scan_task_phase').value)
        self.crop_top_ratio = float(self.get_parameter('crop_top_ratio').value)
        self.crop_top_px = int(self.get_parameter('crop_top_px').value)
        self.upscale_factor = max(1.0, float(self.get_parameter('upscale_factor').value))
        self.detection_order = str(self.get_parameter('detection_order').value).strip().lower()
        if self.detection_order not in ('crop_only', 'full_only', 'full_then_crop', 'crop_then_full'):
            self.get_logger().warn(
                f'unsupported detection_order={self.detection_order!r}; using crop_only'
            )
            self.detection_order = 'crop_only'
        self.min_publish_interval = float(self.get_parameter('min_publish_interval').value)
        self.diagnostics_enabled = bool(self.get_parameter('diagnostics_enabled').value)
        self.diagnostics_interval_sec = max(
            0.2, float(self.get_parameter('diagnostics_interval_sec').value)
        )
        self.diagnostics_log_subdir = str(
            self.get_parameter('diagnostics_log_subdir').value
        ).strip()
        self.diagnostics_log_filename = str(
            self.get_parameter('diagnostics_log_filename').value
        ).strip()
self.debug_image_enabled = bool(self.get_parameter('debug_image_enabled').value)
        self.debug_image_filename = str(
            self.get_parameter('debug_image_filename').value
        ).strip() or 'qr_latest.jpg'

        self._diag_log = None
        if self.diagnostics_enabled:
            self._diag_log = SessionFileLog(
                self.diagnostics_log_subdir or 'stage1',
                filename=self.diagnostics_log_filename or 'latest.log',
                session_title='QR scanner diagnostic',
            )
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        package_share = get_package_share_directory('qr_scanner')
        config_dir = os.path.join(package_share, 'config')
        self.wechat_detect_prototxt = self.resolve_model_path(
            self.get_parameter('wechat_detect_prototxt').value,
            os.path.join(config_dir, 'detect.prototxt'),
        )
        self.wechat_detect_caffemodel = self.resolve_model_path(
            self.get_parameter('wechat_detect_caffemodel').value,
            os.path.join(config_dir, 'detect.caffemodel'),
        )
        self.wechat_sr_prototxt = self.resolve_model_path(
            self.get_parameter('wechat_sr_prototxt').value,
            os.path.join(config_dir, 'sr.prototxt'),
        )
        self.wechat_sr_caffemodel = self.resolve_model_path(
            self.get_parameter('wechat_sr_caffemodel').value,
            os.path.join(config_dir, 'sr.caffemodel'),
        )

        self.bridge = CvBridge()
        self.wechat_detector = None
        self.active_backend = self.initialize_backend()
        self.publisher_ = self.create_publisher(String, self.result_topic, 10)
        self._activated = False
        self._mission_search_enabled = False
        self._released = False
        self.current_x = None
        self.latest_image_msg = None
        self.scan_armed = False
        self.scan_completed = False
        self.scan_activation_logged = False
        self.last_qr_content = ''
        self.last_publish_time = None
        self.diag_window_started_at = None
        self.diag_last_frame_stamp_ns = None
        self.diag_attempt_count = 0
        self.diag_candidate_count = 0
        self.diag_total_decode_ms = 0.0
        self.diag_max_decode_ms = 0.0
        self.diag_source_gap_total_ms = 0.0
        self.diag_source_gap_count = 0
        self.diag_backend_error_count = 0
        self.diag_last_image_shape = None
        self.diag_last_candidates = []
        self.diag_variant_stats = {}
        self.frame_lock = threading.Lock()
        self.frame_event = threading.Event()
        self.shutdown_event = threading.Event()

        state_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, self.stage1_state_topic, self.stage1_state_callback, state_qos
        )
        self._activate_srv = self.create_service(
            Trigger, '/competition/stage1/qr_activate', self._activate_cb
        )
        self._release_srv = self.create_service(
            Trigger, '/competition/stage1/qr_release', self._release_cb
        )
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)

        if self.use_compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                self.camera_topic,
                self.compressed_callback,
                qos,
            )
        else:
            self.subscription = self.create_subscription(
                Image,
                self.camera_topic,
                self.raw_callback,
                qos,
            )

        self.decode_worker = threading.Thread(
            target=self.decode_worker_loop,
            name='qr_decode_worker',
            daemon=True,
        )
        self.decode_worker.start()

        image_mode = 'compressed' if self.use_compressed else 'raw'
        self.get_logger().info(
            f'qr scanner ready, topic={self.camera_topic}, mode={image_mode}, result={self.result_topic}, '
                f'backend={self.active_backend}, gate=stage1 SEARCH_QR, '
            f'crop_px={self.crop_top_px}, upscale={self.upscale_factor:.1f}x, '
            f'order={self.detection_order}, path={self.active_backend}_raw, '
            'latest_frame_worker=true'
        )
        self.reset_diagnostic_log()
        self.write_diagnostic(
            'ready '
            f'backend={self.active_backend} topic={self.camera_topic} '
            'gate=stage1 SEARCH_QR '
            f'crop_px={self.crop_top_px} '
            f'upscale={self.upscale_factor:.1f}x order={self.detection_order} '
            f'path={self.active_backend}_raw latest_frame_worker=true '
            f'debug_image={self.debug_image_path() or "disabled"}'
        )

def diagnostic_log_path(self):
        if self._diag_log is None:
            return ''
        return self._diag_log.path

    def debug_image_path(self):
        if not self.debug_image_enabled:
            return ''

        log_path = self.diagnostic_log_path()
        if not log_path:
            return ''
        return os.path.join(os.path.dirname(log_path), self.debug_image_filename)

    def write_diagnostic(self, message):
        if not self.diagnostics_enabled or self._diag_log is None:
            return

        self._diag_log.write(f'[QR_DIAG] {message}')

    def reset_diagnostic_log(self):
        if not self.diagnostics_enabled:
            return

        if self._diag_log is not None:
            self._diag_log.close()
        self._diag_log = SessionFileLog(
            self.diagnostics_log_subdir or 'stage1',
            filename=self.diagnostics_log_filename or 'latest.log',
            session_title='QR scanner diagnostic',
        )

    def write_debug_image(self, gray_image, candidate_descriptions, results):
        path = self.debug_image_path()
        if not path:
            return

        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            vis_image = cv2.cvtColor(gray_image, cv2.COLOR_GRAY2BGR)
            height, width = gray_image.shape[:2]
            crop_top = self.compute_crop_top(height)
            if 0 < crop_top < height:
                cv2.line(vis_image, (0, crop_top), (width - 1, crop_top), (0, 255, 255), 2)
                cv2.putText(
                    vis_image,
                    f'detect area below y={crop_top}',
                    (8, min(height - 8, crop_top + 24)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )

            status = 'decoded' if results else 'no_decode'
            candidate_text = ','.join(candidate_descriptions) or 'none'
            x_text = 'nan' if self.current_x is None else f'{self.current_x:.2f}'
            overlay_lines = [
                f'QR {status} x={x_text}m active={int(self._activated)}',
                f'order={self.detection_order} candidate={candidate_text}',
            ]
            if results:
                overlay_lines.append(f'content={results[0][:48]}')

            for index, text in enumerate(overlay_lines):
                y = 24 + index * 24
                cv2.putText(
                    vis_image,
                    text,
                    (8, y),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.55,
                    (0, 255, 0) if results else (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            path_root, path_ext = os.path.splitext(path)
            tmp_path = f'{path_root}.tmp{path_ext or ".jpg"}'
            if cv2.imwrite(tmp_path, vis_image, [cv2.IMWRITE_JPEG_QUALITY, 85]):
                os.replace(tmp_path, path)
        except Exception as exc:
            self.get_logger().warn(f'failed to write QR debug image: {exc}')

    def reset_diagnostic_window(self):
        self.diag_window_started_at = time.monotonic()
        self.diag_attempt_count = 0
        self.diag_candidate_count = 0
        self.diag_total_decode_ms = 0.0
        self.diag_max_decode_ms = 0.0
        self.diag_source_gap_total_ms = 0.0
        self.diag_source_gap_count = 0
        self.diag_backend_error_count = 0
        self.diag_variant_stats = {
            'wechat_raw': {'attempts': 0, 'total_ms': 0.0, 'max_ms': 0.0}
        }

    def record_frame_stamp(self, image_msg):
        stamp = image_msg.header.stamp
        stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        if stamp_ns <= 0:
            return
        if self.diag_last_frame_stamp_ns is not None and stamp_ns > self.diag_last_frame_stamp_ns:
            self.diag_source_gap_total_ms += (stamp_ns - self.diag_last_frame_stamp_ns) / 1e6
            self.diag_source_gap_count += 1
        self.diag_last_frame_stamp_ns = stamp_ns

    def log_diagnostic_summary(self, force=False):
        if not self.diagnostics_enabled or self.diag_window_started_at is None:
            return

        elapsed_sec = time.monotonic() - self.diag_window_started_at
        if not force and elapsed_sec < self.diagnostics_interval_sec:
            return
        if self.diag_attempt_count == 0:
            self.reset_diagnostic_window()
            return

        average_decode_ms = self.diag_total_decode_ms / self.diag_attempt_count
        source_fps = 0.0
        if self.diag_source_gap_total_ms > 0.0:
            source_fps = 1000.0 * self.diag_source_gap_count / self.diag_source_gap_total_ms
        height, width = self.diag_last_image_shape or (0, 0)
        candidates = ','.join(self.diag_last_candidates) or 'none'
        variant_stats = ';'.join(
            f'{name}(n={stats["attempts"]},avg='
            f'{stats["total_ms"] / stats["attempts"]:.1f}ms,max={stats["max_ms"]:.1f}ms)'
            for name, stats in self.diag_variant_stats.items()
            if stats['attempts'] > 0
        ) or 'none'
        self.write_diagnostic(
            f'summary x={self.current_x if self.current_x is not None else float("nan"):.2f}m '
            f'attempts={self.diag_attempt_count} rate={self.diag_attempt_count / elapsed_sec:.1f}Hz '
            f'decode_avg={average_decode_ms:.1f}ms decode_max={self.diag_max_decode_ms:.1f}ms '
            f'source_fps={source_fps:.1f} image={width}x{height} '
            f'candidates={candidates} variant_stats={variant_stats} '
            f'backend_errors={self.diag_backend_error_count}'
        )
        self.reset_diagnostic_window()

    def resolve_model_path(self, configured_path, default_path):
        candidate = str(configured_path).strip()
        return candidate if candidate else default_path

    def initialize_backend(self):
        detector = self.create_wechat_detector()
        if detector is None:
            self.get_logger().error('wechat QR backend unavailable')
            return 'disabled'

        self.wechat_detector = detector
        return 'wechat'

    def create_wechat_detector(self):
        if not hasattr(cv2, 'wechat_qrcode_WeChatQRCode'):
            self.get_logger().warn('OpenCV build does not provide wechat_qrcode_WeChatQRCode')
            return None

        model_paths = [
            self.wechat_detect_prototxt,
            self.wechat_detect_caffemodel,
            self.wechat_sr_prototxt,
            self.wechat_sr_caffemodel,
        ]
        missing = [path for path in model_paths if not os.path.exists(path)]
        if missing:
            self.get_logger().warn(f'wechat QR model files missing: {missing}')
            return None

        try:
            return cv2.wechat_qrcode_WeChatQRCode(
                self.wechat_detect_prototxt,
                self.wechat_detect_caffemodel,
                self.wechat_sr_prototxt,
                self.wechat_sr_caffemodel,
            )
        except Exception as exc:
            self.get_logger().warn(f'failed to initialize wechat QR detector: {exc}')
            return None

    def should_scan(self):
        return (
            self.active_backend != 'disabled'
            and not self.scan_completed
            and self._activated
            and self._mission_search_enabled
        )

    def stage1_state_callback(self, msg):
        state = msg.data.strip()
        if state == 'search_qr' and not self._released:
            self._mission_search_enabled = True
            self.scan_completed = False
            self.scan_activation_logged = False
            self.scan_armed = self.should_scan()
        elif state == 'running' and not self._released:
            # Supervisor grants the service activation separately.  Running
            # only announces motion ownership; SEARCH_QR is the actual QR
            # capture gate.
            self._mission_search_enabled = False
            self.scan_armed = False
        elif state in ('qr_locked', 'return_to_entry', 'handoff_wait',
                       'handoff_ready', 'complete', 'failed'):
            self._mission_search_enabled = False
            self._activated = False
            self.scan_armed = False
            self.clear_latest_frame()
            if state == 'complete' and not self._released:
                self._released = True
                self.create_timer(0.10, self._shutdown_after_release)

    def _activate_cb(self, _request, response):
        if self._released:
            response.success = False
            response.message = 'QR scanner already released'
            return response
        self._activated = True
        self.scan_armed = self.should_scan()
        response.success = True
        response.message = 'QR scanner activated'
        return response

    def _release_cb(self, _request, response):
        self._released = True
        self._activated = False
        self.scan_armed = False
        self.clear_latest_frame()
        response.success = True
        response.message = 'QR scanner released'
        self.create_timer(0.10, self._shutdown_after_release)
        return response

    def _shutdown_after_release(self):
        if rclpy.ok():
            rclpy.shutdown()

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.scan_armed = self.should_scan()
        if self.scan_armed and not self.scan_activation_logged:
            self.scan_activation_logged = True
            self.reset_diagnostic_window()
            self.get_logger().info(f'qr scan armed at x={self.current_x:.2f} m')
            self.write_diagnostic(f'armed x={self.current_x:.2f}m')
        if not self.scan_armed:
            self.clear_latest_frame()

    def compressed_callback(self, msg):
        self.enqueue_latest_frame(msg)

    def raw_callback(self, msg):
        self.enqueue_latest_frame(msg)

    def enqueue_latest_frame(self, image_msg):
        """Keep only the newest camera frame while a decode is in progress."""
        if not self.scan_armed:
            return

        with self.frame_lock:
            self.latest_image_msg = image_msg
        self.frame_event.set()

    def clear_latest_frame(self):
        with self.frame_lock:
            self.latest_image_msg = None
        self.frame_event.clear()

    def decode_worker_loop(self):
        while not self.shutdown_event.is_set():
            if not self.frame_event.wait(timeout=0.2):
                continue

            self.frame_event.clear()
            with self.frame_lock:
                image_msg = self.latest_image_msg
                self.latest_image_msg = None

            if image_msg is not None:
                self.process_image_msg(image_msg)

    def compute_crop_top(self, image_height):
        if image_height <= 1:
            return 0

        if self.crop_top_px > 0:
            return min(self.crop_top_px, image_height - 1)

        return int(max(0.0, min(self.crop_top_ratio, 0.95)) * image_height)

    def iter_detection_images(self, gray_image):
        crop_top = self.compute_crop_top(gray_image.shape[0])
        cropped = gray_image[crop_top:, :] if crop_top > 0 else None

        if self.detection_order == 'crop_only':
            if cropped is not None and cropped.size != 0:
                yield 'crop', cropped
            return

        if self.detection_order in ('full_only', 'full_then_crop'):
            yield 'full', gray_image

        if self.detection_order != 'full_only' and cropped is not None and cropped.size != 0:
            yield 'crop', cropped

        if self.detection_order == 'crop_then_full':
            yield 'full', gray_image

    def upscale_for_detection(self, gray_image):
        if self.upscale_factor <= 1.0:
            return gray_image

        return cv2.resize(
            gray_image,
            None,
            fx=self.upscale_factor,
            fy=self.upscale_factor,
            interpolation=cv2.INTER_CUBIC,
        )

    def record_variant_attempt(self, variant, decode_ms):
        stats = self.diag_variant_stats.setdefault(
            variant, {'attempts': 0, 'total_ms': 0.0, 'max_ms': 0.0}
        )
        stats['attempts'] += 1
        stats['total_ms'] += decode_ms
        stats['max_ms'] = max(stats['max_ms'], decode_ms)

    def process_image_msg(self, image_msg):
        if not self.scan_armed:
            return

        try:
            if self.use_compressed:
                gray_image = self.bridge.compressed_imgmsg_to_cv2(image_msg, 'mono8')
            else:
                gray_image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='mono8')
        except Exception:
            return

        if gray_image is None or gray_image.size == 0:
            return

        self.record_frame_stamp(image_msg)
        self.diag_last_image_shape = gray_image.shape[:2]
        results = []
        decoded_path = ''
        candidate_descriptions = []
        decode_started_at = time.monotonic()
        for candidate_name, detection_image in self.iter_detection_images(gray_image):
            candidate_descriptions = [
                f'{candidate_name}:{detection_image.shape[1]}x{detection_image.shape[0]}'
            ]
            variant_results, variant_ms = self.decode_variant(
                detection_image
            )
            self.record_variant_attempt('wechat_raw', variant_ms)
            if variant_results:
                results = variant_results
                decoded_path = f'{candidate_name}/wechat_raw'
            break

        decode_ms = (time.monotonic() - decode_started_at) * 1000.0
        self.diag_attempt_count += 1
        self.diag_candidate_count += len(candidate_descriptions)
        self.diag_total_decode_ms += decode_ms
        self.diag_max_decode_ms = max(self.diag_max_decode_ms, decode_ms)
        self.diag_last_candidates = candidate_descriptions
        self.write_debug_image(gray_image, candidate_descriptions, results)

        if not results:
            self.log_diagnostic_summary()
            return

        now = self.get_clock().now()
        for qr_content in results:
            if not qr_content:
                continue
            if self.last_publish_time is not None and qr_content == self.last_qr_content:
                elapsed = (now - self.last_publish_time).nanoseconds / 1e9
                if elapsed < self.min_publish_interval:
                    continue

            self.last_qr_content = qr_content
            self.last_publish_time = now
            self.scan_completed = True
            self.scan_armed = False
            self.write_diagnostic(
                f'detected x={self.current_x if self.current_x is not None else float("nan"):.2f}m '
                f'content={qr_content!r} path={decoded_path} decode={decode_ms:.1f}ms '
                f'attempts_since_last_summary={self.diag_attempt_count} '
                f'candidates={";".join(candidate_descriptions)} '
                f'backend_errors={self.diag_backend_error_count}'
            )
            self.log_diagnostic_summary(force=True)
            self.get_logger().warn(f'qr detected via {self.active_backend}: {qr_content}')
            self.publisher_.publish(String(data=qr_content))
            return

    def decode_variant(self, gray_image):
        started_at = time.monotonic()
        results = self.detect_and_decode(self.upscale_for_detection(gray_image))
        return results, (time.monotonic() - started_at) * 1000.0

    def detect_and_decode(self, gray_image):
        if self.wechat_detector is None:
            return []

        try:
            decoded = self.wechat_detector.detectAndDecode(gray_image)
        except Exception as exc:
            self.diag_backend_error_count += 1
            self.get_logger().warn(f'wechat QR decode failed: {exc}')
            return []

        if isinstance(decoded, tuple):
            decoded = decoded[0]

        if isinstance(decoded, str):
            decoded = [decoded]

        if not isinstance(decoded, (list, tuple)):
            return []

        return [str(item).strip() for item in decoded if str(item).strip()]

    def destroy_node(self):
        self.shutdown_event.set()
        self.frame_event.set()
        if self.decode_worker.is_alive():
            self.decode_worker.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    install_parent_death_signal()
    rclpy.init(args=args)
    node = QRScannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
