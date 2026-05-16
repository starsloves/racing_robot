import base64
import json
from urllib import error, request

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


class ImageCaptionNode(Node):
    def __init__(self):
        super().__init__('image_caption_node')

        self.declare_parameter('detection_topic', 'person_detection')
        self.declare_parameter('crop_topic', 'person_detection_crop')
        self.declare_parameter('caption_topic', 'human_caption_result')
        self.declare_parameter('feedback_topic', 'competition_feedback')
        self.declare_parameter('caption_mode', 'template')
        self.declare_parameter('caption_api_url', '')
        self.declare_parameter('request_timeout_sec', 3.0)
        self.declare_parameter('cooldown_sec', 5.0)

        self.detection_topic = self.get_parameter('detection_topic').value
        self.crop_topic = self.get_parameter('crop_topic').value
        self.caption_topic = self.get_parameter('caption_topic').value
        self.feedback_topic = self.get_parameter('feedback_topic').value
        self.caption_mode = self.get_parameter('caption_mode').value
        self.caption_api_url = self.get_parameter('caption_api_url').value
        self.request_timeout_sec = float(self.get_parameter('request_timeout_sec').value)
        self.cooldown_sec = float(self.get_parameter('cooldown_sec').value)

        self.latest_crop = None
        self.last_publish_time = 0.0

        self.caption_pub = self.create_publisher(String, self.caption_topic, 10)
        self.feedback_pub = self.create_publisher(String, self.feedback_topic, 10)

        self.create_subscription(String, self.detection_topic, self.detection_callback, 10)
        self.create_subscription(CompressedImage, self.crop_topic, self.crop_callback, 10)
        self.get_logger().info('image caption node ready')

    def crop_callback(self, msg):
        self.latest_crop = msg.data

    def template_caption(self, payload):
        position_map = {
            'left': '画面左侧',
            'center': '画面中间',
            'right': '画面右侧',
        }
        size_map = {
            'near': '距离较近',
            'medium': '距离中等',
            'far': '距离较远',
        }
        position_text = position_map.get(payload.get('position_tag', 'center'), '画面中间')
        size_text = size_map.get(payload.get('size_tag', 'medium'), '距离中等')
        confidence = payload.get('confidence', 0.0)
        return f'图生文结果：发现一块人形立牌，位于{position_text}，{size_text}，检测置信度约为 {confidence:.2f}。'

    def http_caption(self, payload):
        if not self.caption_api_url or not self.latest_crop:
            return None

        request_body = json.dumps({
            'image_base64': base64.b64encode(self.latest_crop).decode('utf-8'),
            'metadata': payload,
        }).encode('utf-8')
        http_request = request.Request(
            self.caption_api_url,
            data=request_body,
            headers={'Content-Type': 'application/json'},
            method='POST',
        )
        try:
            with request.urlopen(http_request, timeout=self.request_timeout_sec) as response:
                response_data = json.loads(response.read().decode('utf-8'))
        except (error.URLError, TimeoutError, json.JSONDecodeError):
            return None

        if isinstance(response_data, dict):
            caption = response_data.get('caption') or response_data.get('text')
            if isinstance(caption, str) and caption.strip():
                return caption.strip()
        return None

    def detection_callback(self, msg):
        now_sec = self.get_clock().now().nanoseconds / 1e9
        if now_sec - self.last_publish_time < self.cooldown_sec:
            return

        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        caption = None
        if self.caption_mode == 'http_api':
            caption = self.http_caption(payload)
        if not caption:
            caption = self.template_caption(payload)

        self.caption_pub.publish(String(data=caption))
        self.feedback_pub.publish(String(data=caption))
        self.last_publish_time = now_sec
        self.get_logger().info(caption)


def main(args=None):
    rclpy.init(args=args)
    node = ImageCaptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()