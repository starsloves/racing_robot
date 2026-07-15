#!/usr/bin/env python3
"""
bpu_direct_test.py �?BPU 推理磁盘记录版本

订阅相机 �?BPU 推理 �?保存 JPG + MP4 + CSV �?~/dev_ws/log/debug/bpu_test/
�?OpenCV 窗口依赖，可在纯 SSH 下运行�?"""
import rclpy, cv2, numpy as np, time, sys, os
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy

from racing_stage2.lane_follow import _compute_seg_mask, _center_offset

try:
    from hobot_dnn import pyeasy_dnn as dnn
except ImportError:
    dnn = None

LOG_DIR = os.path.expanduser('~/dev_ws/log/debug/bpu_test')


class BpuDirectTest(Node):
    def __init__(self):
        super().__init__('bpu_direct_test')
        self.bridge = CvBridge()
        self._count = 0

        from ament_index_python import get_package_share_directory
        pkg_dir = get_package_share_directory('racing_stage2')
        mp = os.path.join(pkg_dir, 'models', 'saidao_seg_model_quant.bin')
        self.model = dnn.load(mp)[0]
        self.get_logger().info(f'model loaded: {self.model.name}')

        os.makedirs(LOG_DIR, exist_ok=True)
        self.csv_path = os.path.join(LOG_DIR, 'bpu_test.csv')
        with open(self.csv_path, 'w') as f:
            f.write('ts,frame_id,infer_ms,detected,offset\n')
        self.get_logger().info(f'csv: {self.csv_path}')

        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)
        self.sub = self.create_subscription(Image, '/aurora/rgb/image_raw', self.cb, qos)
        self.get_logger().info('waiting for camera...')

    def cb(self, msg):
        self._count += 1
        t0 = time.time()

        cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        rgb = cv2.cvtColor(cv_img, cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(cv2.resize(rgb, (640, 640)).astype(np.uint8), axis=0)

        outs = self.model.forward([inp])
        infer_ms = (time.time() - t0) * 1000

        binary = _compute_seg_mask(outs[0].buffer, outs[1].buffer, 0.3, 0.5)
        offset = _center_offset(binary)

        viz = cv_img.copy()
        h, w = viz.shape[:2]
        if binary is not None:
            mask_big = cv2.resize(binary.astype(np.uint8) * 255, (w, h))
            green = np.zeros_like(viz)
            green[:] = [0, 255, 0]
            viz = cv2.addWeighted(viz, 0.6, green, 0.4, 0)
            viz[mask_big == 0] = cv_img[mask_big == 0]

        # 每帧�?CSV
        ts = time.time()
        with open(self.csv_path, 'a') as f:
            f.write(f'{ts:.3f},{self._count},{infer_ms:.0f},{binary is not None},{offset}\n')

        # �?10 �?+ 之后每秒一帧保存图�?        save_interval = max(1, int(1.0 / max(0.001, (time.time() - t0))))
        if self._count <= 10 or self._count % save_interval == 0:
            fname = os.path.join(LOG_DIR, f'frame_{self._count:04d}.jpg')
            cv2.imwrite(fname, viz)

        self.get_logger().info(
            f'#{self._count} infer:{infer_ms:.0f}ms det={binary is not None} ofs={offset}'
        )


def main(args=None):
    if dnn is None:
        print('ERROR: hobot_dnn not available'); sys.exit(1)
    rclpy.init(args=args)
    node = BpuDirectTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
