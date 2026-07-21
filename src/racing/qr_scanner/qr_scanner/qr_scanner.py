import os
import time
import fcntl

import cv2
import rclpy
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, String

class QRScannerNode(Node):
    def __init__(self):
        super().__init__('qr_scanner')

        self.declare_parameter('camera_topic', '/image')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('result_topic', 'qr_scan_result')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('odom_topic', '/odom_combined')
        self.declare_parameter('scan_task_phase', 1)
        self.declare_parameter('scan_start_x_m', 1.0)
        self.declare_parameter('scan_rate_hz', 4.0)
        self.declare_parameter('crop_top_ratio', 0.25)
        self.declare_parameter('crop_top_px', 80)
        self.declare_parameter('upscale_factor', 1.0)
        self.declare_parameter('detection_order', 'crop_only')
        self.declare_parameter('process_every_frame', True)
        self.declare_parameter('min_publish_interval', 1.0)
        self.declare_parameter('diagnostics_enabled', True)
        self.declare_parameter('diagnostics_interval_sec', 1.0)
        self.declare_parameter('diagnostics_log_subdir', 'competition_stage1')
        self.declare_parameter('diagnostics_log_filename', 'latest.log')
        self.declare_parameter('backend', 'wechat')
        self.declare_parameter('allow_backend_fallback', True)
        self.declare_parameter('wechat_detect_prototxt', '')
        self.declare_parameter('wechat_detect_caffemodel', '')
        self.declare_parameter('wechat_sr_prototxt', '')
        self.declare_parameter('wechat_sr_caffemodel', '')

        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        self.result_topic = self.get_parameter('result_topic').value
        self.phase_topic = self.get_parameter('phase_topic').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.scan_task_phase = int(self.get_parameter('scan_task_phase').value)
        self.scan_start_x_m = float(self.get_parameter('scan_start_x_m').value)
        self.scan_rate_hz = float(self.get_parameter('scan_rate_hz').value)
        self.crop_top_ratio = float(self.get_parameter('crop_top_ratio').value)
        self.crop_top_px = int(self.get_parameter('crop_top_px').value)
        self.upscale_factor = max(1.0, float(self.get_parameter('upscale_factor').value))
        self.detection_order = str(self.get_parameter('detection_order').value).strip().lower()
        if self.detection_order not in ('crop_only', 'full_only', 'full_then_crop', 'crop_then_full'):
            self.get_logger().warn(
                f'unsupported detection_order={self.detection_order!r}; using crop_only'
            )
            self.detection_order = 'crop_only'
        self.process_every_frame = bool(self.get_parameter('process_every_frame').value)
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
        self.requested_backend = str(self.get_parameter('backend').value).strip().lower()
        self.allow_backend_fallback = bool(self.get_parameter('allow_backend_fallback').value)

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
        self.opencv_detector = None
        self.active_backend = self.initialize_backend()
        self.publisher_ = self.create_publisher(String, self.result_topic, 10)
        self.phase = self.scan_task_phase
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

        self.create_subscription(Int32, self.phase_topic, self.phase_callback, 10)
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

        if not self.process_every_frame:
            self.create_timer(1.0 / max(self.scan_rate_hz, 1.0), self.process_latest_frame)

        image_mode = 'compressed' if self.use_compressed else 'raw'
        self.get_logger().info(
            f'qr scanner ready, topic={self.camera_topic}, mode={image_mode}, result={self.result_topic}, '
            f'backend={self.active_backend}, arm_phase={self.scan_task_phase}, arm_x>{self.scan_start_x_m:.2f}m, '
            f'crop_px={self.crop_top_px}, upscale={self.upscale_factor:.1f}x, '
            f'order={self.detection_order}, path=wechat_raw, '
            f'per_frame={self.process_every_frame}'
        )
        self.write_diagnostic(
            'ready '
            f'backend={self.active_backend} topic={self.camera_topic} '
            f'arm_x>{self.scan_start_x_m:.2f}m crop_px={self.crop_top_px} '
            f'upscale={self.upscale_factor:.1f}x order={self.detection_order} '
            f'path=wechat_raw per_frame={self.process_every_frame}'
        )

    def diagnostic_log_path(self):
        workspace_root = os.environ.get('DEV_WS', '').strip()
        if not workspace_root:
            workspace_root = os.getcwd()
        if not os.path.isdir(os.path.join(workspace_root, 'src', 'racing')):
            return ''
        subdir = self.diagnostics_log_subdir or 'competition_stage1'
        filename = self.diagnostics_log_filename or 'latest.log'
        return os.path.join(workspace_root, 'log', subdir, filename)

    def write_diagnostic(self, message):
        """Append QR processing evidence without taking ownership of the Stage1 log."""
        if not self.diagnostics_enabled:
            return

        path = self.diagnostic_log_path()
        if not path:
            return
        try:
            os.makedirs(os.path.dirname(path), exist_ok=True)
            with open(path, 'a', encoding='utf-8') as log_file:
                fcntl.flock(log_file.fileno(), fcntl.LOCK_EX)
                try:
                    log_file.write(f'[QR_DIAG] {message}\n')
                    log_file.flush()
                finally:
                    fcntl.flock(log_file.fileno(), fcntl.LOCK_UN)
        except OSError as exc:
            self.get_logger().warn(f'failed to write QR diagnostic log: {exc}')

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
        requested = self.requested_backend if self.requested_backend in ('wechat', 'opencv') else 'wechat'

        if requested == 'wechat':
            detector = self.create_wechat_detector()
            if detector is not None:
                self.wechat_detector = detector
                return 'wechat'
            if not self.allow_backend_fallback:
                self.get_logger().error('wechat backend unavailable and fallback disabled')
                return 'disabled'
            self.get_logger().warn('wechat backend unavailable, falling back to opencv QRCodeDetector')

        self.opencv_detector = cv2.QRCodeDetector()
        return 'opencv'

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
            and self.phase == self.scan_task_phase
            and self.current_x is not None
            and self.current_x > self.scan_start_x_m
        )

    def phase_callback(self, msg):
        previous_phase = self.phase
        self.phase = int(msg.data)
        if self.phase != self.scan_task_phase:
            self.scan_armed = False
            self.scan_activation_logged = False
            self.latest_image_msg = None
        elif previous_phase != self.phase:
            self.scan_completed = False
            self.scan_activation_logged = False

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.scan_armed = self.should_scan()
        if self.scan_armed and not self.scan_activation_logged:
            self.scan_activation_logged = True
            self.reset_diagnostic_window()
            self.get_logger().info(f'qr scan armed at x={self.current_x:.2f} m')
            self.write_diagnostic(f'armed x={self.current_x:.2f}m phase={self.phase}')
        if not self.scan_armed:
            self.latest_image_msg = None

    def compressed_callback(self, msg):
        if self.scan_armed and self.process_every_frame:
            self.process_image_msg(msg)
        elif self.scan_armed:
            self.latest_image_msg = msg

    def raw_callback(self, msg):
        if self.scan_armed and self.process_every_frame:
            self.process_image_msg(msg)
        elif self.scan_armed:
            self.latest_image_msg = msg

    def process_latest_frame(self):
        if not self.scan_armed or self.latest_image_msg is None:
            return

        image_msg = self.latest_image_msg
        self.latest_image_msg = None

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
                f'{candidate_name}/wechat_raw:{detection_image.shape[1]}x{detection_image.shape[0]}'
            ]
            variant_results, variant_ms = self.decode_variant(detection_image)
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
        if self.active_backend == 'wechat' and self.wechat_detector is not None:
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

        if self.opencv_detector is None:
            return []

        try:
            decoded, _, _ = self.opencv_detector.detectAndDecode(gray_image)
        except Exception as exc:
            self.diag_backend_error_count += 1
            self.get_logger().warn(f'OpenCV QR decode failed: {exc}')
            return []

        decoded = decoded.strip() if isinstance(decoded, str) else ''
        return [decoded] if decoded else []

def main(args=None):
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
