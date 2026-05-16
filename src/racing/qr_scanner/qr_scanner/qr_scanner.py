import os

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
        self.declare_parameter('scan_start_x_m', 2.0)
        self.declare_parameter('scan_rate_hz', 4.0)
        self.declare_parameter('crop_top_ratio', 0.25)
        self.declare_parameter('crop_top_px', 155)
        self.declare_parameter('process_every_frame', True)
        self.declare_parameter('min_publish_interval', 1.0)
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
        self.process_every_frame = bool(self.get_parameter('process_every_frame').value)
        self.min_publish_interval = float(self.get_parameter('min_publish_interval').value)
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
        self.scan_activation_logged = False
        self.last_qr_content = ''
        self.last_publish_time = None

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
            f'crop_px={self.crop_top_px}, per_frame={self.process_every_frame}'
        )

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
            self.scan_activation_logged = False

    def odom_callback(self, msg):
        self.current_x = float(msg.pose.pose.position.x)
        self.scan_armed = self.should_scan()
        if self.scan_armed and not self.scan_activation_logged:
            self.scan_activation_logged = True
            self.get_logger().info(f'qr scan armed at x={self.current_x:.2f} m')
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
        if crop_top > 0:
            cropped = gray_image[crop_top:, :]
            if cropped.size != 0:
                yield cropped

        yield gray_image

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

        results = []
        for detection_image in self.iter_detection_images(gray_image):
            results = self.detect_and_decode(detection_image)
            if results:
                break

        if not results:
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
            self.scan_armed = False
            self.get_logger().warn(f'qr detected via {self.active_backend}: {qr_content}')
            self.publisher_.publish(String(data=qr_content))
            return

    def detect_and_decode(self, gray_image):
        if self.active_backend == 'wechat' and self.wechat_detector is not None:
            try:
                decoded = self.wechat_detector.detectAndDecode(gray_image)
            except Exception:
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
        except Exception:
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