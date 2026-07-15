#!/usr/bin/env python3
"""
seg_debug_viewer.py �?分割模型调试查看�?
纯观察模式：启动底盘 + 摄像�?+ 分割模型推理�?将模型输出叠加图保存到磁盘供赛后分析。不发出任何运动指令�?
保存目录：~/dev_ws/log/debug/seg_debug_frames_<时间�?/
"""

import os
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from racing_stage2.lane_follow import VisionLaneEngine


class SegDebugViewer(Node):
    def __init__(self):
        super().__init__('seg_debug_viewer')

        for name, default in [
            ('camera_topic',        '/aurora/rgb/image_raw'),
            ('model_path',          ''),
            ('conf_threshold',      0.3),
            ('mask_threshold',      0.5),
            ('roi_bottom',          0.35),
            ('save_dir',            'dev_ws/log/debug'),
            ('save_raw',            True),
        ]:
            self.declare_parameter(name, default)

        g = self.get_parameter
        self._bridge = CvBridge()
        mp = str(g('model_path').value) or self._default_model_path()
        self._engine = VisionLaneEngine(
            mp,
            conf_thr=float(g('conf_threshold').value),
            mask_thr=float(g('mask_threshold').value),
            roi_bottom=float(g('roi_bottom').value),
            logger=self.get_logger(),
        )
        self._save_raw = bool(g('save_raw').value)
        self._frame_count = 0

        # 保存目录
        base = str(g('save_dir').value)
        d = base if os.path.isabs(base) else os.path.join(os.path.expanduser('~'), base)
        ts = time.strftime('%Y%m%d_%H%M%S')
        self._save_dir = os.path.join(d, f'seg_debug_frames_{ts}')
        os.makedirs(self._save_dir, exist_ok=True)
        self.get_logger().info(f'[SegDebug] 保存目录: {self._save_dir}')

        # 订阅相机
        cam_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )
        self.create_subscription(
            Image, str(g('camera_topic').value),
            self._cam_cb, cam_qos,
        )

        # 发布可视化话题（可在 rviz 中实时查看）
        self._viz_pub = self.create_publisher(Image, '/seg_debug_viz', 10)
        self._mask_pub = self.create_publisher(Image, '/seg_debug_mask', 10)

        self.get_logger().info(
            f'[SegDebug] 就绪 engine={"OK" if self._engine.ready else "FAIL"}'
        )

    @staticmethod
    def _default_model_path():
        return os.path.normpath(os.path.join(
            os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
            'models', 'saidao_seg_model_quant.bin'
        ))

    def _cam_cb(self, msg):
        if not self._engine or not self._engine.ready:
            return
        try:
            cv_img = self._bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return

        t0 = time.time()
        r = self._engine.process(cv_img)
        dt = time.time() - t0
        if dt > 0.1:
            self.get_logger().warn(
                f'inference {dt*1000:.0f}ms', throttle_duration_sec=2.0)

        det = r.get('has_detection', False)
        off = r.get('center_offset')
        self.get_logger().info(
            f'[SegDebug] det={det} offset={off if off is not None else "N/A":>6} '
            f'infer={dt*1000:.0f}ms',
            throttle_duration_sec=1.0,
        )

        # 保存图片
        self._frame_count += 1
        try:
            cv2 = __import__('cv2')
            if self._save_raw:
                cv2.imwrite(
                    os.path.join(self._save_dir, f'raw_{self._frame_count:06d}.jpg'),
                    cv_img,
                )
            ov = r.get('viz_overlay')
            if ov is not None:
                cv2.imwrite(
                    os.path.join(self._save_dir, f'overlay_{self._frame_count:06d}.jpg'),
                    ov,
                )
        except Exception:
            pass

        # 发布可视�?        try:
            ts = self.get_clock().now().to_msg()
            for pub, key in [(self._viz_pub, 'viz_overlay'),
                             (self._mask_pub, 'viz_mask')]:
                m = self._bridge.cv2_to_imgmsg(r[key], 'bgr8')
                m.header.stamp = ts
                m.header.frame_id = 'camera'
                pub.publish(m)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = SegDebugViewer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
