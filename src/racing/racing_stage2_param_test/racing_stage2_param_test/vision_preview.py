#!/usr/bin/env python3
"""
vision_preview.py — 视觉预览（纯观察，不动车）

Step 1 of visual system implementation:
  - Load BPU segmentation model
  - Subscribe to camera topic
  - Run inference and publish visualization topics
  - Does NOT publish /cmd_vel
"""

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


# ═══════════════════════════════════════════════════════════
# YOLOv8-Seg 后处理（纯函数，与 ROS 无关）
# 来源: bak/lane_follow.py (原样复制)
# ═══════════════════════════════════════════════════════════


def _compute_seg_mask(output0, output1, conf_thr=0.25, mask_thr=0.5):
    """
    YOLOv8-Seg 标准后处理：argmax objectness → mc @ proto → sigmoid → 阈值化。
    """
    raw = output0.reshape(-1, 8400).T       # (8400, 37)
    scores = 1.0 / (1.0 + np.exp(-np.clip(raw[:, 4], -20, 20)))
    best = np.argmax(scores)
    if scores[best] < conf_thr:
        return None
    proto = output1.reshape(32, 160, 160)
    mc = raw[best, 5:37]                    # (32,) mask coeffs
    flat = 1.0 / (1.0 + np.exp(-np.clip(mc @ proto.reshape(32, -1), -20, 20)))
    return flat.reshape(160, 160) > mask_thr


def _center_offset(binary, roi_bottom=0.35):
    if binary is None:
        return None
    h, w = binary.shape
    rh = max(5, int(h * roi_bottom))
    roi = binary[-rh:, :]
    offs = []
    for r in range(roi.shape[0] - 1, max(0, roi.shape[0] - 15), -2):
        nz = np.where(roi[r, :])[0]
        if len(nz) > 5:
            offs.append((nz[0] + nz[-1]) / 2.0 - w / 2.0)
    if not offs:
        return None
    return float(np.clip(np.median(offs) / (w / 2.0), -1.0, 1.0))


# ═══════════════════════════════════════════════════════════
# 可复用推理引擎 — VisionLaneEngine
# 来源: bak/lane_follow.py (原样复制)
# ═══════════════════════════════════════════════════════════

class VisionLaneEngine:
    def __init__(self, model_path, mask_thr=0.7,
                 roi_bottom=0.35, logger=None):
        self.mask_thr = mask_thr
        self.roi_bottom = roi_bottom
        self.model = None
        self._model_path = model_path
        self._log = (lambda msg: logger.info(msg)) if logger else (lambda msg: None)
        self._err = (lambda msg: logger.error(msg)) if logger else (lambda msg: print(msg))
        self._init()

    def _init(self):
        try:
            from hobot_dnn import pyeasy_dnn as dnn
            models = dnn.load(self._model_path)
            self.model = models[0]
            self._log(f'[VisionEngine] loaded: {self.model.name}')
        except Exception as e:
            self._err(f'[VisionEngine] load failed: {e}')

    @property
    def ready(self):
        return self.model is not None

    def process(self, bgr_image):
        if not self.ready:
            return _no_det(bgr_image)

        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(
            cv2.resize(rgb, (640, 640)).astype(np.uint8), axis=0
        )

        try:
            outs = self.model.forward([inp])
        except Exception as e:
            self._err(f'[VisionEngine] infer: {e}')
            return _no_det(bgr_image)

        if len(outs) < 2:
            return _no_det(bgr_image)

        binary = _compute_seg_mask(outs[0].buffer, outs[1].buffer,
                                   mask_thr=self.mask_thr)
        offset = _center_offset(binary, self.roi_bottom)

        return {
            'binary': binary,
            'center_offset': offset,
            'has_detection': binary is not None and offset is not None,
            'viz_overlay': _viz_overlay(bgr_image, binary, offset, self.roi_bottom),
            'viz_mask': _viz_mask(binary),
            'proto': outs[1].buffer.copy(),
            'det': outs[0].buffer.copy(),
        }


# ═══════════════════════════════════════════════════════════
# 可视化辅助
# 来源: bak/lane_follow.py (原样复制)
# ═══════════════════════════════════════════════════════════

def _no_det(bgr):
    img = bgr.copy()
    cv2.putText(img, 'NO DETECTION', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return {'binary': None, 'center_offset': None, 'has_detection': False,
            'viz_overlay': img, 'viz_mask': np.full((160, 160, 3), 80, dtype=np.uint8)}


def _viz_mask(binary):
    viz = np.full((160, 160, 3), 80, dtype=np.uint8)
    if binary is not None:
        viz[binary > 0] = [0, 255, 0]
    return viz


def _viz_overlay(bgr, binary, offset, roi_bottom):
    h, w = bgr.shape[:2]
    if binary is None:
        img = bgr.copy()
        cv2.putText(img, 'NO TRACK', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        return img

    # Coverage sanity: if >70% filled, likely false positive
    coverage = binary.sum() / binary.size
    show_as_bad = coverage > 0.70

    mr = cv2.resize(binary.astype(np.uint8) * 255, (w, h))
    mc = np.zeros_like(bgr)
    mc[:, :, 1] = mr
    ov = cv2.addWeighted(bgr, 0.6, mc, 0.4, 0)

    if show_as_bad:
        cv2.putText(ov, f'COVERAGE {coverage*100:.0f}% (BAD)', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    rh = max(5, int(h * roi_bottom))
    rb = cv2.resize(binary[-rh:, :].astype(np.uint8) * 255, (w, rh))
    ls, rs = [], []
    for row in range(rh - 1, max(0, rh - 30), -3):
        nz = np.where(rb[row, :] > 0)[0]
        if len(nz) > 10:
            ls.append(nz[0])
            rs.append(nz[-1])
    if ls and rs and offset is not None:
        al, ar = int(np.median(ls)), int(np.median(rs))
        dy = h - rh // 2
        cv2.line(ov, (al, dy - 20), (al, dy + 20), (255, 0, 0), 2)
        cv2.line(ov, (ar, dy - 20), (ar, dy + 20), (255, 0, 0), 2)
        cv2.line(ov, (int((al + ar) / 2), dy), (w // 2, dy), (0, 0, 255), 3)
        cv2.putText(ov, f'offset: {offset:+.3f} cov:{coverage*100:.0f}%', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    elif offset is None:
        cv2.putText(ov, f'NO BOUNDARY cov:{coverage*100:.0f}%', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return ov


# ═══════════════════════════════════════════════════════════
# ROS 节点 — VisionPreview（纯观察，不动车）
# ═══════════════════════════════════════════════════════════

class VisionPreview(Node):
    """
    视觉预览节点。

    订阅相机话题 → BPU 推理 → 发布可视化话题。
    不发出任何 /cmd_vel 运动指令。
    """

    def __init__(self):
        super().__init__('vision_preview')
        self._declare_params()
        self._read_params()
        self._resolve_model_path()
        self._resolve_save_dir()
        self.bridge = CvBridge()
        self._engine = None
        self._frame_count = 0

        self.get_logger().info(f'[VisionPreview] saving raw→{self.raw_dir} / viz→{self.viz_dir}')

        qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            durability=rclpy.qos.DurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            Image, self.camera_topic, self._cam_cb, qos
        )
        self._viz_pub = self.create_publisher(Image, '/lane_seg_viz', 10)
        self._mask_pub = self.create_publisher(Image, '/lane_seg_mask', 10)

        self.get_logger().info(
            f'[VisionPreview] cam={self.camera_topic} 就绪'
        )

    def _declare_params(self):
        for p in [
            ('camera_topic',   '/aurora/rgb/image_raw'),
            ('model_path',     ''),
            ('mask_threshold', 0.7),
            ('roi_bottom',     0.35),
            ('save_dir',       ''),
        ]:
            self.declare_parameter(p[0], p[1])

    def _read_params(self):
        g = self.get_parameter
        self.camera_topic   = g('camera_topic').value
        self.model_path     = g('model_path').value
        self.mask_thr       = g('mask_threshold').value
        self.roi_bottom     = g('roi_bottom').value
        self.save_dir       = g('save_dir').value

    def _resolve_model_path(self):
        if not self.model_path:
            real = os.path.realpath(__file__)
            self.model_path = os.path.normpath(os.path.join(
                os.path.dirname(os.path.dirname(real)),
                'models', 'saidao_seg_model_quant.bin'
            ))

    def _resolve_save_dir(self):
        if not self.save_dir:
            self.save_dir = os.path.normpath(os.path.join(
                os.path.expanduser('~'),
                'dev_ws', 'log', 'debug', 'vision_preview'
            ))
        self.raw_dir = os.path.join(self.save_dir, 'raw')
        self.viz_dir = os.path.join(self.save_dir, 'viz')
        os.makedirs(self.raw_dir, exist_ok=True)
        os.makedirs(self.viz_dir, exist_ok=True)

    def _get_engine(self):
        if self._engine is None:
            self._engine = VisionLaneEngine(
                self.model_path, self.mask_thr,
                self.roi_bottom, logger=self.get_logger()
            )
        return self._engine

    def _cam_cb(self, msg):
        engine = self._get_engine()
        if not engine.ready:
            return

        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        t0 = time.perf_counter()
        result = engine.process(cv_img)
        dt_ms = (time.perf_counter() - t0) * 1000.0

        self._frame_count += 1

        binary = result['binary']
        cov = f'{(binary.sum()/binary.size*100):.0f}%' if binary is not None else 'N/A'

        if self._frame_count % 10 == 0 or self._frame_count == 1:
            det = result['has_detection']
            off = f'{result["center_offset"]:+.4f}' if result['center_offset'] is not None else 'N/A'
            self.get_logger().info(
                f'infer={dt_ms:.0f}ms det={det} offset={off} cov={cov}'
            )

        ts = msg.header.stamp
        try:
            for pub, key in [(self._viz_pub, 'viz_overlay'),
                             (self._mask_pub, 'viz_mask')]:
                img_msg = self.bridge.cv2_to_imgmsg(result[key], 'bgr8')
                img_msg.header.stamp = ts
                img_msg.header.frame_id = 'camera'
                pub.publish(img_msg)
        except Exception as e:
            self.get_logger().warn(f'publish viz: {e}')

        # 保存 raw + overlay 到不同文件夹
        fc = self._frame_count
        raw_path = os.path.join(self.raw_dir, f'{fc:06d}.jpg')
        cv2.imwrite(raw_path, cv_img)
        viz_path = os.path.join(self.viz_dir, f'{fc:06d}.jpg')
        cv2.imwrite(viz_path, result['viz_overlay'])


def main(args=None):
    rclpy.init(args=args)
    node = VisionPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
