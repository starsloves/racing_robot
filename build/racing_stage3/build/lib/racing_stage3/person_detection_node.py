import json
from math import hypot

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Int32, String


class PersonDetectionNode(Node):
    def __init__(self):
        super().__init__('person_detection_node')

        self.declare_parameter('camera_topic', '/image')
        self.declare_parameter('use_compressed', True)
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('detection_topic', 'person_detection')
        self.declare_parameter('crop_topic', 'person_detection_crop')
        self.declare_parameter('detector_rate_hz', 4.0)
        self.declare_parameter('stage2_only', True)
        self.declare_parameter('min_detection_interval_sec', 4.0)
        self.declare_parameter('report_once', True)
        self.declare_parameter('confirm_frames', 2)
        self.declare_parameter('min_bbox_height', 90)
        self.declare_parameter('min_bbox_area_ratio', 0.02)
        self.declare_parameter('max_bbox_area_ratio', 0.55)
        self.declare_parameter('weight_threshold', 0.35)

        self.camera_topic = self.get_parameter('camera_topic').value
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        self.phase_topic = self.get_parameter('phase_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.crop_topic = self.get_parameter('crop_topic').value
        self.detector_rate_hz = float(self.get_parameter('detector_rate_hz').value)
        self.stage2_only = bool(self.get_parameter('stage2_only').value)
        self.min_detection_interval_sec = float(self.get_parameter('min_detection_interval_sec').value)
        self.report_once = bool(self.get_parameter('report_once').value)
        self.confirm_frames = int(self.get_parameter('confirm_frames').value)
        self.min_bbox_height = int(self.get_parameter('min_bbox_height').value)
        self.min_bbox_area_ratio = float(self.get_parameter('min_bbox_area_ratio').value)
        self.max_bbox_area_ratio = float(self.get_parameter('max_bbox_area_ratio').value)
        self.weight_threshold = float(self.get_parameter('weight_threshold').value)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        self.bridge = CvBridge()
        self.phase = 1
        self.latest_frame = None
        self.latest_frame_stamp = None
        self.last_detection_time = 0.0
        self.reported = False
        self.pending_bbox = None
        self.pending_count = 0

        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.detection_pub = self.create_publisher(String, self.detection_topic, 10)
        self.crop_pub = self.create_publisher(CompressedImage, self.crop_topic, 10)

        self.create_subscription(Int32, self.phase_topic, self.phase_callback, 10)
        if self.use_compressed:
            self.create_subscription(CompressedImage, self.camera_topic, self.compressed_callback, qos)
        else:
            self.create_subscription(Image, self.camera_topic, self.raw_callback, qos)

        self.create_timer(1.0 / max(self.detector_rate_hz, 1.0), self.process_latest_frame)
        self.get_logger().info('person detection node ready')

    def phase_callback(self, msg):
        new_phase = int(msg.data)
        if new_phase != self.phase and new_phase != 2:
            self.reported = False
            self.pending_bbox = None
            self.pending_count = 0
        self.phase = new_phase

    def compressed_callback(self, msg):
        try:
            self.latest_frame = self.bridge.compressed_imgmsg_to_cv2(msg, 'bgr8')
            self.latest_frame_stamp = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            pass

    def raw_callback(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_frame_stamp = self.get_clock().now().nanoseconds / 1e9
        except Exception:
            pass

    def similar_bbox(self, bbox_a, bbox_b):
        if bbox_a is None or bbox_b is None:
            return False
        center_a = (bbox_a[0] + bbox_a[2] / 2.0, bbox_a[1] + bbox_a[3] / 2.0)
        center_b = (bbox_b[0] + bbox_b[2] / 2.0, bbox_b[1] + bbox_b[3] / 2.0)
        return hypot(center_a[0] - center_b[0], center_a[1] - center_b[1]) < 50.0

    def bbox_tags(self, bbox, width, height):
        x, y, w, h = bbox
        center_x = x + w / 2.0
        position_ratio = center_x / max(width, 1)
        if position_ratio < 0.33:
            position_tag = 'left'
        elif position_ratio > 0.66:
            position_tag = 'right'
        else:
            position_tag = 'center'

        size_ratio = (w * h) / float(max(width * height, 1))
        if size_ratio > 0.15:
            size_tag = 'near'
        elif size_ratio > 0.05:
            size_tag = 'medium'
        else:
            size_tag = 'far'
        return position_tag, size_tag

    def publish_crop(self, frame, bbox):
        x, y, w, h = bbox
        x0 = max(int(x), 0)
        y0 = max(int(y), 0)
        x1 = min(int(x + w), frame.shape[1])
        y1 = min(int(y + h), frame.shape[0])
        crop = frame[y0:y1, x0:x1]
        if crop.size == 0:
            return

        success, encoded = cv2.imencode('.jpg', crop)
        if not success:
            return

        msg = CompressedImage()
        msg.format = 'jpeg'
        msg.data = encoded.tobytes()
        self.crop_pub.publish(msg)

    def process_latest_frame(self):
        if self.latest_frame is None:
            return
        if self.stage2_only and self.phase != 2:
            return
        if self.report_once and self.reported:
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_detection_time < self.min_detection_interval_sec:
            return

        frame = self.latest_frame.copy()
        height, width = frame.shape[:2]
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        rects, weights = self.hog.detectMultiScale(
            gray,
            winStride=(8, 8),
            padding=(8, 8),
            scale=1.05,
        )

        best_bbox = None
        best_weight = 0.0
        image_area = float(max(width * height, 1))
        for index, rect in enumerate(rects):
            x, y, w, h = rect
            weight = float(weights[index]) if len(weights) > index else 0.0
            area_ratio = (w * h) / image_area
            if weight < self.weight_threshold:
                continue
            if h < self.min_bbox_height:
                continue
            if area_ratio < self.min_bbox_area_ratio or area_ratio > self.max_bbox_area_ratio:
                continue
            aspect_ratio = h / float(max(w, 1))
            if aspect_ratio < 1.1 or aspect_ratio > 4.0:
                continue
            if weight > best_weight:
                best_weight = weight
                best_bbox = (int(x), int(y), int(w), int(h))

        if best_bbox is None:
            self.pending_bbox = None
            self.pending_count = 0
            return

        if self.similar_bbox(best_bbox, self.pending_bbox):
            self.pending_count += 1
        else:
            self.pending_bbox = best_bbox
            self.pending_count = 1

        if self.pending_count < max(self.confirm_frames, 1):
            return

        position_tag, size_tag = self.bbox_tags(best_bbox, width, height)
        payload = {
            'bbox': {
                'x': best_bbox[0],
                'y': best_bbox[1],
                'w': best_bbox[2],
                'h': best_bbox[3],
            },
            'position_tag': position_tag,
            'size_tag': size_tag,
            'confidence': round(best_weight, 3),
            'image_width': width,
            'image_height': height,
            'stamp': now_sec,
        }
        self.publish_crop(frame, best_bbox)
        self.detection_pub.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.last_detection_time = now_sec
        self.pending_bbox = best_bbox
        self.pending_count = 0
        if self.report_once:
            self.reported = True
        self.get_logger().info(f'person-like target detected: {payload}')


def main(args=None):
    rclpy.init(args=args)
    node = PersonDetectionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()