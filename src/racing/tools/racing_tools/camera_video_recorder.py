import os
import subprocess
from datetime import datetime

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


class CameraVideoRecorder(Node):
    """启动相机驱动 + 订阅相机话题，逐帧保存为 JPG 图片。"""

    def __init__(self):
        super().__init__('camera_video_recorder')

        self.declare_parameter('camera_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('use_compressed', False)
        self.declare_parameter('output_dir', 'dev_ws/log/video')
        self.declare_parameter('output_prefix', 'stage2_path')
        self.declare_parameter('max_duration_sec', 0.0)
        self.declare_parameter('rgb_fps', 15)
        self.declare_parameter('resolution_mode_index', 2)

        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.use_compressed = bool(self.get_parameter('use_compressed').value)
        raw_dir = str(self.get_parameter('output_dir').value)
        self.output_dir = raw_dir if os.path.isabs(raw_dir) else os.path.join(os.path.expanduser('~'), raw_dir)
        self.output_prefix = str(self.get_parameter('output_prefix').value)
        self.max_duration_sec = max(0.0, float(self.get_parameter('max_duration_sec').value))
        self.rgb_fps = int(self.get_parameter('rgb_fps').value)
        self.resolution_mode_index = int(self.get_parameter('resolution_mode_index').value)

        self._bridge = CvBridge()
        self._frame_count = 0
        self._started_at = None
        self._stopped = False
        self._session_ts = datetime.now().strftime('%Y%m%d_%H%M%S')
        self._save_dir = os.path.join(self.output_dir, f'{self.output_prefix}_{self._session_ts}')
        self._camera_proc = None

        os.makedirs(self._save_dir, exist_ok=True)
        self.get_logger().info(f'保存目录: {self._save_dir}')

        # 启动相机驱动
        self._start_camera_driver()

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
            f'save_dir={self._save_dir}'
        )

    def _start_camera_driver(self):
        """启动 Aurora 930 相机驱动进程。"""
        try:
            self._camera_proc = subprocess.Popen(
                [
                    'ros2', 'run', 'deptrum-ros-driver-aurora930', 'aurora930_node',
                    '--ros-args', '--log-level', 'warn',
                    '-r', '__ns:=/aurora',
                    '-p', f'rgb_enable:=True',
                    '-p', f'ir_enable:=False',
                    '-p', f'depth_enable:=False',
                    '-p', f'rgbd_enable:=False',
                    '-p', f'point_cloud_enable:=False',
                    '-p', f'boot_order:=1',
                    '-p', f'rgb_fps:={self.rgb_fps}',
                    '-p', f'resolution_mode_index:={self.resolution_mode_index}',
                    '-p', f'align_mode:=False',
                    '-p', f'log_dir:=/tmp/',
                    '-p', f'stream_sdk_log_enable:=False',
                    '-p', f'heart_enable:=False',
                ],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            self.get_logger().info('相机驱动已启动 (aurora930_node)')
        except Exception as e:
            self.get_logger().error(f'启动相机驱动失败: {e}')

    def _save_frame(self, frame):
        if self._stopped or frame is None or frame.size == 0:
            return

        self._frame_count += 1
        fname = f'frame_{self._frame_count:06d}.jpg'
        path = os.path.join(self._save_dir, fname)
        cv2.imwrite(path, frame, [cv2.IMWRITE_JPEG_QUALITY, 90])

        if self._frame_count == 1:
            self._started_at = self.get_clock().now()
            self.get_logger().info(
                f'开始逐帧保存: {self._save_dir} ({frame.shape[1]}x{frame.shape[0]})'
            )

        if self._frame_count % 150 == 0:
            self.get_logger().info(
                f'已保存 {self._frame_count} 帧 -> {self._save_dir}'
            )

        if (
            self.max_duration_sec > 0.0
            and self._started_at is not None
            and (self.get_clock().now() - self._started_at).nanoseconds / 1e9
            >= self.max_duration_sec
        ):
            self.get_logger().info(
                f'达到最大录像时长 {self.max_duration_sec:.0f}s，停止保存'
            )
            self._stopped = True

    def _image_callback(self, msg: Image):
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'解码 Image 失败: {exc}')
            return
        self._save_frame(frame)

    def _compressed_callback(self, msg: CompressedImage):
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warning(f'解码 CompressedImage 失败: {exc}')
            return
        self._save_frame(frame)

    def _stop_camera_driver(self):
        if self._camera_proc is not None:
            self._camera_proc.terminate()
            try:
                self._camera_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self._camera_proc.kill()
            self._camera_proc = None
            self.get_logger().info('相机驱动已停止')

    def destroy_node(self):
        self._stopped = True
        total = self._frame_count
        self._stop_camera_driver()
        super().destroy_node()
        if total > 0:
            self.get_logger().info(
                f'逐帧保存结束: {self._save_dir} (共 {total} 帧)'
            )


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