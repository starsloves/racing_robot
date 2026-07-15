import os
import threading
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraVideoRecorder(Node):
    """订阅相机话题，持续写入视频到 dev_ws 根目录�?""

    def __init__(self):
        super().__init__('camera_video_recorder')

        self.declare_parameter('camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('output_dir', 'log/video')
        self.declare_parameter('output_prefix', 'stage2_path')
        self.declare_parameter('fps', 15.0)
        self.declare_parameter('fourcc', 'MJPG')
        self.declare_parameter('file_ext', '.avi')
        self.declare_parameter('max_duration_sec', 0.0)

        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.output_prefix = str(self.get_parameter('output_prefix').value)
        self.target_fps = max(1.0, float(self.get_parameter('fps').value))
        self.fourcc = str(self.get_parameter('fourcc').value).ljust(4)[:4]
        self.file_ext = str(self.get_parameter('file_ext').value)
        self.max_duration_sec = max(0.0, float(self.get_parameter('max_duration_sec').value))

        self._bridge = CvBridge()
        self._writer = None
        self._writer_lock = threading.Lock()
        self._frame_count = 0
        self._started_at = None
        self._output_path = ''
        self._stopped = False

        os.makedirs(self.output_dir, exist_ok=True)
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        if self.use_compressed:
            self.create_subscription(
                CompressedImage,
                self.camera_topic,
                self._compressed_callback,
                qos,
            )
        else:
            self.create_subscription(
                Image,
                self.camera_topic,
                self._image_callback,
                qos,
            )

        self.get_logger().info(
            f'录像节点就绪：topic={self.camera_topic}, '
            f'compressed={self.use_compressed}, '
            f'output_dir={self.output_dir}, '
            f'fps={self.target_fps:.1f}'
        )

    def _build_output_path(self):
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        filename = f'{self.output_prefix}_{timestamp}{self.file_ext}'
        return os.path.join(self.output_dir, filename)

    def _ensure_writer(self, frame):
        if self._writer is not None:
            return True

        height, width = frame.shape[:2]
        if width <= 0 or height <= 0:
            return False

        self._output_path = self._build_output_path()
        fourcc = cv2.VideoWriter_fourcc(*self.fourcc)
        writer = cv2.VideoWriter(
            self._output_path,
            fourcc,
            self.target_fps,
            (width, height),
        )
        if not writer.isOpened():
            self.get_logger().error(
                f'无法创建视频文件: {self._output_path} '
                f'(fourcc={self.fourcc})'
            )
            return False

        self._writer = writer
        self._started_at = self.get_clock().now()
        self.get_logger().info(f'开始录�? {self._output_path} ({width}x{height})')
        return True

    def _write_frame(self, frame):
        if self._stopped or frame is None or frame.size == 0:
            return

        with self._writer_lock:
            if not self._ensure_writer(frame):
                return

            if frame.ndim == 2:
                frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
            elif frame.shape[2] == 4:
                frame = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)

            self._writer.write(frame)
            self._frame_count += 1

            if self._frame_count == 1 or self._frame_count % 150 == 0:
                self.get_logger().info(
                    f'已写�?{self._frame_count} �?-> {self._output_path}'
                )

            if (
                self.max_duration_sec > 0.0
                and self._started_at is not None
                and (self.get_clock().now() - self._started_at).nanoseconds / 1e9
                >= self.max_duration_sec
            ):
                self.get_logger().info(
                    f'达到最大录像时�?{self.max_duration_sec:.0f}s，停止写�?
                )
                self._release_writer_locked()

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'解码 Image 失败: {exc}')
            return
        self._write_frame(frame)

    def _compressed_callback(self, msg: CompressedImage):
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'解码 CompressedImage 失败: {exc}')
            return
        self._write_frame(frame)

    def _release_writer_locked(self):
        if self._writer is None:
            return

        self._writer.release()
        self._writer = None
        self._stopped = True
        self.get_logger().info(
            f'录像已保�? {self._output_path} (�?{self._frame_count} �?'
        )

    def destroy_node(self):
        with self._writer_lock:
            self._release_writer_locked()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraVideoRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
