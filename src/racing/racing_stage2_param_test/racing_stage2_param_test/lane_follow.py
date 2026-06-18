#!/usr/bin/env python3
"""
lane_follow.py — 视觉赛道居中控制

提供:
  - VisionLaneEngine   : 纯 BPU 推理引擎，可被任意节点复用
  - LaneFollowNode     : 独立 ROS 2 节点，裸视觉居中测试
"""

import os
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge


# ═══════════════════════════════════════════════════════════
# YOLOv8-Seg 后处理（纯函数，与 ROS 无关）
# ═══════════════════════════════════════════════════════════

def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))


def _nms_single(boxes, scores, iou_thr=0.45):
    if len(scores) == 0:
        return -1
    order = scores.argsort()[::-1]
    keep = []
    while len(order) > 0:
        i = order[0]
        keep.append(i)
        if len(order) == 1:
            break
        x1 = np.maximum(boxes[i, 0], boxes[order[1:], 0])
        y1 = np.maximum(boxes[i, 1], boxes[order[1:], 1])
        x2 = np.minimum(boxes[i, 2], boxes[order[1:], 2])
        y2 = np.minimum(boxes[i, 3], boxes[order[1:], 3])
        inter = np.maximum(0, x2 - x1) * np.maximum(0, y2 - y1)
        ai = (boxes[i, 2] - boxes[i, 0]) * (boxes[i, 3] - boxes[i, 1])
        ao = (boxes[order[1:], 2] - boxes[order[1:], 0]) * (boxes[order[1:], 3] - boxes[order[1:], 1])
        iou = inter / (ai + ao - inter + 1e-6)
        order = order[1:][iou < iou_thr]
    return keep[0] if keep else -1


def _make_grids(strides=(8, 16, 32)):
    grids = []
    arrs = []
    for s in strides:
        n = 640 // s
        xv, yv = np.meshgrid(np.arange(n, dtype=np.float32), np.arange(n, dtype=np.float32))
        g = np.stack((xv, yv), axis=-1).reshape(-1, 2)
        grids.append(g)
        arrs.append(np.full(n * n, s, dtype=np.float32))
    return np.concatenate(grids, axis=0), np.concatenate(arrs, axis=0)


def _decode_bboxes(raw):
    """raw: (37, 8400, 1) → bboxes (N,4) xyxy, scores (N,), coeffs (N,32)"""
    raw = raw.reshape(-1, 8400).T  # (8400, 37)
    grid_xy, strides = _make_grids()

    cx = (_sigmoid(raw[:, 0]) * 2 - 0.5 + grid_xy[:, 0]) * strides
    cy = (_sigmoid(raw[:, 1]) * 2 - 0.5 + grid_xy[:, 1]) * strides
    w = ((_sigmoid(raw[:, 2]) * 2) ** 2) * strides
    h = ((_sigmoid(raw[:, 3]) * 2) ** 2) * strides

    bb = np.zeros((raw.shape[0], 4), dtype=np.float32)
    bb[:, 0] = cx - w / 2
    bb[:, 1] = cy - h / 2
    bb[:, 2] = cx + w / 2
    bb[:, 3] = cy + h / 2
    return bb, _sigmoid(raw[:, 4]), raw[:, 5:37]


def _compute_seg_mask(output0, output1, conf_thr=0.25, mask_thr=0.5):
    """→ (160,160) bool 或 None"""
    bb, scores, coeffs = _decode_bboxes(output0)
    proto = output1.reshape(32, 160, 160)

    valid = scores > conf_thr
    if not valid.any():
        return None
    best = _nms_single(bb[valid], scores[valid])
    if best < 0:
        return None

    mc = coeffs[valid][best]
    flat = _sigmoid(mc @ proto.reshape(32, -1)).reshape(160, 160)
    return flat > mask_thr


def _center_offset(binary, roi_bottom=0.35):
    """→ float ∈ [-1, 1] 或 None。+1=右转, -1=左转, 0=居中"""
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
# ═══════════════════════════════════════════════════════════

class VisionLaneEngine:
    """
    纯 BPU 视觉推理引擎，无 ROS 依赖。

    engine = VisionLaneEngine('/path/to/model.bin')
    result = engine.process(bgr_image)
    # result: {'binary', 'center_offset', 'has_detection',
    #          'viz_overlay', 'viz_mask'}
    """

    def __init__(self, model_path, conf_thr=0.3, mask_thr=0.5,
                 roi_bottom=0.35, logger=None):
        self.conf_thr = conf_thr
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
        """
        处理一帧 BGR 图像 → dict。

        center_offset: -1..1, 0=居中, +1=需右转, -1=需左转
        """
        h, w = bgr_image.shape[:2]
        if not self.ready:
            return _no_det(bgr_image)

        # 预处理
        rgb = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        inp = np.expand_dims(
            cv2.resize(rgb, (640, 640)).transpose(2, 0, 1).astype(np.uint8), axis=0
        )

        # BPU 推理（直接传 numpy 数组，无需 pyDNNTensor 包装）
        try:
            from hobot_dnn import pyeasy_dnn as dnn
            outs = self.model.forward([inp])
        except Exception as e:
            self._err(f'[VisionEngine] infer: {e}')
            return _no_det(bgr_image)

        if len(outs) < 2:
            return _no_det(bgr_image)

        binary = _compute_seg_mask(outs[0].buffer, outs[1].buffer,
                                   self.conf_thr, self.mask_thr)
        offset = _center_offset(binary, self.roi_bottom)

        return {
            'binary': binary,
            'center_offset': offset,
            'has_detection': binary is not None and offset is not None,
            'viz_overlay': _viz_overlay(bgr_image, binary, offset, self.roi_bottom),
            'viz_mask': _viz_mask(binary),
        }


# ═══════════════════════════════════════════════════════════
# 可视化辅助
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

    mr = cv2.resize(binary.astype(np.uint8) * 255, (w, h))
    mc = np.zeros_like(bgr)
    mc[:, :, 1] = mr
    ov = cv2.addWeighted(bgr, 0.6, mc, 0.4, 0)

    rh = max(5, int(h * roi_bottom))
    rb = cv2.resize(binary[-rh:, :].astype(np.uint8) * 255, (w, rh))
    ls, rs = [], []
    for row in range(rh - 1, max(0, rh - 30), -3):
        nz = np.where(rb[row, :] > 0)[0]
        if len(nz) > 10:
            ls.append(nz[0])
            rs.append(nz[-1])
    if ls and rs:
        al, ar = int(np.median(ls)), int(np.median(rs))
        dy = h - rh // 2
        cv2.line(ov, (al, dy - 20), (al, dy + 20), (255, 0, 0), 2)
        cv2.line(ov, (ar, dy - 20), (ar, dy + 20), (255, 0, 0), 2)
        cv2.line(ov, (int((al + ar) / 2), dy), (w // 2, dy), (0, 0, 255), 3)
        cv2.putText(ov, f'offset: {offset:+.3f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    return ov


# ═══════════════════════════════════════════════════════════
# 独立 ROS 节点 — LaneFollowNode
# ═══════════════════════════════════════════════════════════

class LaneFollowNode(Node):
    """独立视觉居中控制节点，用于无段序列的裸直道测试。"""

    def __init__(self):
        super().__init__('lane_follow')
        self._declare_params()
        self._read_params()
        self._resolve_model_path()
        self.bridge = CvBridge()
        self.engine = VisionLaneEngine(
            self.model_path, self.conf_thr, self.mask_thr,
            self.roi_bottom, logger=self.get_logger()
        )

        # 缓存最近一次推理结果
        self._last_result = None   # dict
        self._last_offset = 0.0
        self._offset_integral = 0.0
        self._lost_since = time.time()

        # 发布 / 订阅
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.viz_pub = self.create_publisher(Image, self.viz_topic, 10)
        self.mask_pub = self.create_publisher(Image, self.mask_topic, 10)
        self.create_subscription(Image, self.camera_topic, self._image_cb, 10)
        self.create_timer(0.05, self._control_timer)

        self.get_logger().info(f'[LaneFollow] cam={self.camera_topic} cmd={self.cmd_vel_topic}')

    def _declare_params(self):
        for p in [
            ('camera_topic',      '/aurora/rgb/image_raw'),
            ('model_path',        ''),
            ('cmd_vel_topic',     '/lane_cmd_vel'),
            ('viz_topic',         '/lane_seg_viz'),
            ('mask_topic',        '/lane_seg_mask'),
            ('linear_speed',       0.20),
            ('max_angular',        0.6),
            ('kp_center',          0.8),
            ('kd_center',          0.3),
            ('ki_center',          0.01),
            ('conf_threshold',     0.3),
            ('mask_threshold',     0.5),
            ('roi_bottom_ratio',   0.35),
            ('lost_timeout_sec',   0.5),
            ('enable_lane_follow', True),
        ]:
            self.declare_parameter(p[0], p[1])

    def _read_params(self):
        g = self.get_parameter
        self.camera_topic     = g('camera_topic').value
        self.model_path       = g('model_path').value
        self.cmd_vel_topic    = g('cmd_vel_topic').value
        self.viz_topic        = g('viz_topic').value
        self.mask_topic       = g('mask_topic').value
        self.linear_speed     = g('linear_speed').value
        self.max_angular      = g('max_angular').value
        self.kp               = g('kp_center').value
        self.kd               = g('kd_center').value
        self.ki               = g('ki_center').value
        self.conf_thr         = g('conf_threshold').value
        self.mask_thr         = g('mask_threshold').value
        self.roi_bottom       = g('roi_bottom_ratio').value
        self.lost_timeout     = g('lost_timeout_sec').value
        self.enabled          = g('enable_lane_follow').value

    def _resolve_model_path(self):
        if not self.model_path:
            real = os.path.realpath(__file__)
            self.model_path = os.path.normpath(os.path.join(
                os.path.dirname(os.path.dirname(real)),
                'models', 'saidao_seg_model_quant.bin'
            ))

    def _image_cb(self, msg):
        if not self.engine.ready:
            return
        try:
            cv_img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'cv_bridge: {e}')
            return

        result = self.engine.process(cv_img)
        self._last_result = result

        now = time.time()
        if result['has_detection']:
            self._lost_since = now

        # 发布可视化
        try:
            ts = self.get_clock().now().to_msg()
            for topic_name, pub, key in [
                (self.viz_topic,  self.viz_pub,  'viz_overlay'),
                (self.mask_topic, self.mask_pub, 'viz_mask'),
            ]:
                msg_out = self.bridge.cv2_to_imgmsg(result[key], 'bgr8')
                msg_out.header.stamp = ts
                msg_out.header.frame_id = 'camera'
                pub.publish(msg_out)
        except Exception:
            pass

    def _control_timer(self):
        if not self.enabled:
            return

        twist = Twist()
        now = time.time()
        lost = (self._last_result is None or
                not self._last_result.get('has_detection'))

        if lost and (now - self._lost_since > self.lost_timeout):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self._offset_integral = 0.0
            self._last_offset = 0.0
            self.cmd_pub.publish(twist)
            return

        offset = self._last_result['center_offset'] if self._last_result else 0.0
        if offset is None:
            offset = self._last_offset

        dt = 0.05
        self._offset_integral += offset * dt
        self._offset_integral = float(np.clip(self._offset_integral, -10, 10))
        deriv = (offset - self._last_offset) / dt

        # offset > 0 = 赛道在图像右侧 → 需右转 (负 angular.z)
        angular = -(self.kp * offset + self.kd * deriv + self.ki * self._offset_integral)
        angular = float(np.clip(angular, -self.max_angular, self.max_angular))

        linear = self.linear_speed * (1.0 - abs(angular) / self.max_angular * 0.5)
        linear = max(0.05, linear)

        twist.linear.x = linear
        twist.angular.z = angular
        self.cmd_pub.publish(twist)
        self._last_offset = offset


def main(args=None):
    rclpy.init(args=args)
    node = LaneFollowNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
