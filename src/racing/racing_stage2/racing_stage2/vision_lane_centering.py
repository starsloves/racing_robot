#!/usr/bin/env python3
"""
vision_lane_centering.py — 视觉车道中线跟随模块

提供：
1. 订阅相机 topic，裁剪下方+中间 ROI 后 BPU YOLOv8-Seg 推理
2. 多行采样赛道 mask 中心线 + 前瞻点误差 + 曲率估计
3. 实时保存可视化到 /tmp/stage2_vision.jpg，并提供 HTTP 预览

共享接口：
    get_latest_offset() -> (offset, timestamp, valid)  # 兼容旧接口，offset≈中线误差
    get_latest_line_status() -> dict(error, curve, valid, confidence, remaining_m, centered, timestamp)
    get_latest_remaining() -> (remaining_m, free_ratio, timestamp, valid)
"""

import threading
import time
import os
from collections import deque
from http.server import SimpleHTTPRequestHandler
from socketserver import ThreadingTCPServer

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hobot_dnn import pyeasy_dnn as dnn


class VisionLaneCentering:
    """
    视觉车道居中推理节点。
    
    外部接口：
        get_latest_offset() -> (offset, timestamp, valid)
            - offset: [-1.0, +1.0]，负=偏左，正=偏右
            - timestamp: 检测时刻（秒）
            - valid: 是否有效（超时或无检测→False）
    """
    
    def __init__(self, parent_node, model_path, conf_thres=0.25, iou_thres=0.45,
                 crop_ratio=0.4, http_port=8080, crop_side_ratio=0.20):
        self._node = parent_node
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.crop_ratio = crop_ratio
        self.crop_side_ratio = float(np.clip(crop_side_ratio, 0.0, 0.45))
        self.http_port = http_port
        
        # 共享变量（线程安全）
        self._lock = threading.Lock()
        self._inference_active = False  # 仅 phase=2/任务运行时推理
        self._latest_offset = 0.0
        self._latest_timestamp = 0.0
        self._valid = False
        self._detection_timeout_sec = 0.5  # 超过 0.5s 无检测 → invalid
        self._last_valid_state = None  # 记录上次有效状态（用于检测状态变化）
        # 纵向剩余距离（由赛道 mask 顶部位置粗估）
        self._latest_remaining_m = None
        self._latest_free_ratio = 0.0
        self._range_valid = False
        self._range_near_m = 0.15
        self._range_far_m = 2.50
        self._range_center_band = 0.30
        self._range_occ_thresh = 0.12
        # 横向居中：mask 质心 + 中心竖带到位
        self._offset_center_band = 0.12
        self._offset_center_occ_thresh = 0.40
        self._offset_centroid_bottom_ratio = 0.60
        self._latest_center_ratio = 0.0
        self._latest_centered = False
        # 多行中线跟随
        self._sample_rows = 9
        self._lookahead_ratio = 0.62
        self._min_mask_pixels_per_row = 12
        self._min_valid_rows = 4
        self._mask_threshold = 0.50
        self._offset_filter_alpha = 0.35
        self._latest_curve = 0.0
        self._latest_confidence = 0.0
        self._latest_valid_rows = 0
        self._filtered_error = 0.0
        self._has_filtered_error = False
        self._centerline_mode = True  # True=多行中线；False=旧质心兼容

        # 边界安全检测
        self._latest_boundary_safe = True
        self._latest_safety_weight = 1.0
        self._boundary_safety_margin = 0.15  # 边界安全裕度（归一化宽度比例）
        self._boundary_coverage_thresh = 0.20  # 侧边最小mask覆盖率

        # 转弯边界线检测
        self._boundary_ahead = False  # 前方是否检测到转弯边界线
        self._boundary_angle_deg = 0.0  # 边界线角度（相对于前进方向）
        self._boundary_distance_ratio = 0.0  # 边界线距离比例（0=远，1=近）
        self._boundary_far_ratio = 0.0
        self._boundary_mid_ratio = 0.0
        self._boundary_near_ratio = 0.0
        self._boundary_coverage_std = 0.0
        self._boundary_top_y_ratio = 0.0
        self._front_score = 0.0
        self._straight_score = 0.0
        self._left_ratio = 0.0
        self._right_ratio = 0.0
        self._near_error = 0.0
        self._far_error = 0.0
        self._path_bend = 0.0  # far_cx - near_cx 归一化，航向趋势
        # 边界贴边：0=贴着图像边缘，越大越离开边缘（越居中）
        self._left_margin = 0.0   # 左边界距画面左缘 / (w/2) 诊断
        self._right_margin = 0.0
        self._lane_clear = False  # 赛道中心≈画面中心 → 道中央
        self._lane_center_off = 0.0  # (lane_cx-mid)/half，+右
        self._rel_left = 0.0
        self._rel_right = 0.0
        self._center_fill = 0.0  # 中心±10%竖带 mask 占比
        self._center_fill_5 = 0.0  # 中心±5%竖带（纠偏回正）
        self._apex_has_mask = True  # 最高点（最远端中线）是否有黄/SEG
        self._apex_error = 0.0  # 最高点横向：+右 -左
        self._apex_fill = 0.0  # 最高点附近小窗 mask 占比
        # 用户核心判据：顶点处中心线左右占比
        self._apex_left5_fill = 0.0  # 顶点中心线左5%占比
        self._apex_right5_fill = 0.0  # 顶点中心线右5%占比
        self._apex_left10_fill = 0.0  # 顶点中心线左10%占比
        self._apex_right10_fill = 0.0  # 顶点中心线右10%占比
        self._apex_center10_fill = 0.0  # 顶点中心线±10%占比
        self._apex_left30_fill = 0.0  # 顶点中心线左30%占比
        self._apex_right30_fill = 0.0  # 顶点中心线右30%占比
        # 顶点窗口诊断：最高SEG往下10%区域 + 几何中心
        self._apex_y0 = 0.0
        self._apex_y1 = 0.0
        self._apex_top_y = 0.0
        self._apex_cx = 0.0
        self._apex_left_x = 0.0
        self._apex_right_x = 0.0
        self._apex_sample_x = 0.0
        self._apex_sample_y = 0.0
        self._apex_center_src = 'none'  # boundary|sample|image
        self._apex_left5_x0 = 0.0
        self._apex_left5_x1 = 0.0
        self._apex_right5_x0 = 0.0
        self._apex_right5_x1 = 0.0
        self._apex_left10_x0 = 0.0
        self._apex_left10_x1 = 0.0
        self._apex_right10_x0 = 0.0
        self._apex_right10_x1 = 0.0
        self._apex_c10_x0 = 0.0
        self._apex_c10_x1 = 0.0
        self._apex_far_row_fill = 0.0
        self._top20_seg_fill = 0.0  # ROI最高20%原始SEG覆盖率，入弯触发
        self._edge_angle_deg = 90.0
        self._perp_score = 0.0
        
        # 可视化缓存（供 HTTP 服务）
        self._combined_frame = None
        self._jpeg_output_path = '/tmp/stage2_vision.jpg'
        
        # HTTP 健康检查共享状态
        self._http_server_start_time = time.time()
        self._last_frame_save_time = 0.0  # 最后一次保存图像的时间戳
        
        # FPS 控制（30 FPS 高刷新率）
        self._target_fps = 30
        self._min_frame_interval = 1.0 / self._target_fps
        self._last_save_time = 0.0
        
        # 加载模型
        self._node.get_logger().info(f'[视觉] 加载模型: {model_path}')
        models = dnn.load(model_path)
        self.model = models[0]
        self.input_size = 640
        self.REG_MAX = 16
        self.strides = [8, 16, 32]
        
        # ROS 订阅
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # 订阅原始相机图像
        self._node.create_subscription(
            Image, '/aurora/rgb/image_raw', self._image_callback, qos
        )
        
        # 统计
        self._frame_count = 0
        self._det_count = 0
        self._first_frame_ready = False
        self._fps_queue = deque(maxlen=30)
        self._last_time = time.perf_counter()
        self._infer_time_ms = 0.0
        
# 创建占位图像（避免首次连接 404）
        self._write_placeholder_image()

        # 启动 HTTP 静态服务器（后台线程）
        self._start_http_server()

        self._node.get_logger().info('[视觉] 模块初始化完成，等待相机数据...')
        self._node.get_logger().info(
            f'[视觉] HTTP 服务已启动: http://0.0.0.0:{self.http_port}/vision_latest.jpg'
        )
    
    # ═══════════════════════════════════════════════════════
    # 外部接口（供导航节点调用）
    # ═══════════════════════════════════════════════════════
    
    def set_inference_active(self, active: bool):
        """启用/停用视觉推理。Stage1 期间应关闭，避免无意义刷屏和算力占用。"""
        active = bool(active)
        with self._lock:
            prev = getattr(self, '_inference_active', False)
            self._inference_active = active
            if not active:
                self._valid = False
                self._range_valid = False
        if prev != active:
            state = '启用' if active else '停用'
            self._node.get_logger().info(f'[视觉] 推理已{state}')

    def is_inference_active(self) -> bool:
        with self._lock:
            return bool(getattr(self, '_inference_active', False))

    def get_latest_offset(self):
        """
        获取最新的横向偏移量。
        
        返回：
            (offset, timestamp, valid)
            - offset: float, [-1.0, +1.0]，负=偏左，正=偏右
            - timestamp: float, 检测时刻（秒）
            - valid: bool, 是否有效（超时或无检测→False）
        """
        with self._lock:
            now = time.time()
            age = now - self._latest_timestamp
            valid = self._valid and (age < self._detection_timeout_sec)
            return (self._latest_offset, self._latest_timestamp, valid)

    def get_latest_center_status(self):
        """
        返回中心竖带到位状态。

        返回：
            (centered, center_ratio, timestamp, valid)
        """
        with self._lock:
            now = time.time()
            age = now - self._latest_timestamp
            valid = self._valid and (age < self._detection_timeout_sec)
            return (
                bool(self._latest_centered) if valid else False,
                float(self._latest_center_ratio),
                self._latest_timestamp,
                valid,
            )

    def configure_range_estimate(self, near_m=0.15, far_m=2.50,
                                 center_band=0.30, occ_thresh=0.12,
                                 timeout_sec=None):
        """配置 mask→剩余距离 的粗标定参数。"""
        with self._lock:
            self._range_near_m = max(0.0, float(near_m))
            self._range_far_m = max(self._range_near_m + 1e-3, float(far_m))
            self._range_center_band = min(0.9, max(0.05, float(center_band)))
            self._range_occ_thresh = min(0.9, max(0.01, float(occ_thresh)))
            if timeout_sec is not None:
                self._detection_timeout_sec = max(0.05, float(timeout_sec))

    def configure_offset_estimate(self, center_band=0.12, occ_thresh=0.40,
                                  centroid_bottom_ratio=0.60):
        """配置 mask 质心纠偏与中心竖带到位判据（兼容旧模式）。"""
        with self._lock:
            self._offset_center_band = min(0.8, max(0.02, float(center_band)))
            self._offset_center_occ_thresh = min(0.95, max(0.05, float(occ_thresh)))
            self._offset_centroid_bottom_ratio = min(1.0, max(0.1, float(centroid_bottom_ratio)))

    def configure_centerline_follow(
        self,
        sample_rows=9,
        lookahead_ratio=0.62,
        min_mask_pixels_per_row=12,
        min_valid_rows=4,
        mask_threshold=0.50,
        offset_filter_alpha=0.35,
        enabled=True,
    ):
        """配置多行中线跟随参数。"""
        with self._lock:
            self._sample_rows = max(3, int(sample_rows))
            self._lookahead_ratio = float(min(0.95, max(0.05, lookahead_ratio)))
            self._min_mask_pixels_per_row = max(1, int(min_mask_pixels_per_row))
            self._min_valid_rows = max(2, int(min_valid_rows))
            self._mask_threshold = float(min(0.95, max(0.05, mask_threshold)))
            self._offset_filter_alpha = float(min(1.0, max(0.05, offset_filter_alpha)))
            self._centerline_mode = bool(enabled)

    def get_latest_line_status(self):
        """返回中线跟随结构化状态。"""
        with self._lock:
            age = time.time() - self._latest_timestamp if self._latest_timestamp > 0 else 999.0
            valid = bool(self._valid and age < self._detection_timeout_sec)
            return {
                'error': float(self._latest_offset) if valid else 0.0,
                'curve': float(self._latest_curve) if valid else 0.0,
                'valid': valid,
                'confidence': float(self._latest_confidence) if valid else 0.0,
                'remaining_m': self._latest_remaining_m if (valid and self._range_valid) else None,
                'centered': bool(self._latest_centered) if valid else False,
                'center_ratio': float(self._latest_center_ratio) if valid else 0.0,
                'valid_rows': int(self._latest_valid_rows) if valid else 0,
                'timestamp': float(self._latest_timestamp),
                'age': float(age),
                'boundary_safe': bool(getattr(self, '_latest_boundary_safe', True)),
                'safety_weight': float(getattr(self, '_latest_safety_weight', 1.0)),
                # 转弯边界线检测
                'boundary_ahead': bool(getattr(self, '_boundary_ahead', False)),
                'boundary_angle_deg': float(getattr(self, '_boundary_angle_deg', 0.0)),
                'boundary_distance_ratio': float(getattr(self, '_boundary_distance_ratio', 0.0)),
                'boundary_far_ratio': float(getattr(self, '_boundary_far_ratio', 0.0)),
                'boundary_mid_ratio': float(getattr(self, '_boundary_mid_ratio', 0.0)),
                'boundary_near_ratio': float(getattr(self, '_boundary_near_ratio', 0.0)),
                'boundary_coverage_std': float(getattr(self, '_boundary_coverage_std', 0.0)),
                'boundary_top_y_ratio': float(getattr(self, '_boundary_top_y_ratio', 0.0)),
                'front_score': float(getattr(self, '_front_score', 0.0)),
                'straight_score': float(getattr(self, '_straight_score', 0.0)),
                'left_ratio': float(getattr(self, '_left_ratio', 0.0)),
                'right_ratio': float(getattr(self, '_right_ratio', 0.0)),
                'near_error': float(getattr(self, '_near_error', 0.0)),
                'far_error': float(getattr(self, '_far_error', 0.0)),
                'path_bend': float(getattr(self, '_path_bend', 0.0)),
                'left_margin': float(getattr(self, '_left_margin', 0.0)),
                'right_margin': float(getattr(self, '_right_margin', 0.0)),
                'lane_clear': bool(getattr(self, '_lane_clear', False)),
                'lane_center_off': float(getattr(self, '_lane_center_off', 0.0)),
                'rel_left': float(getattr(self, '_rel_left', 0.0)),
                'rel_right': float(getattr(self, '_rel_right', 0.0)),
                'center_fill': float(getattr(self, '_center_fill', 0.0)),
                'center_fill_5': float(getattr(self, '_center_fill_5', 0.0)),
                'apex_has_mask': bool(getattr(self, '_apex_has_mask', True)),
                'apex_error': float(getattr(self, '_apex_error', 0.0)),
                'apex_fill': float(getattr(self, '_apex_fill', 0.0)),
                'edge_angle_deg': float(getattr(self, '_edge_angle_deg', 90.0)),
                'perp_score': float(getattr(self, '_perp_score', 0.0)),
                # 用户核心判据：顶点处中心线左右占比
                'apex_left5_fill': float(getattr(self, '_apex_left5_fill', 0.0)),
                'apex_right5_fill': float(getattr(self, '_apex_right5_fill', 0.0)),
                'apex_left10_fill': float(getattr(self, '_apex_left10_fill', 0.0)),
                'apex_right10_fill': float(getattr(self, '_apex_right10_fill', 0.0)),
                'apex_center10_fill': float(getattr(self, '_apex_center10_fill', 0.0)),
                'apex_left30_fill': float(getattr(self, '_apex_left30_fill', 0.0)),
                'apex_right30_fill': float(getattr(self, '_apex_right30_fill', 0.0)),
                'top20_seg_fill': float(getattr(self, '_top20_seg_fill', 0.0)),
                # 顶点窗口诊断字段
                'apex_y0': float(getattr(self, '_apex_y0', 0.0)),
                'apex_y1': float(getattr(self, '_apex_y1', 0.0)),
                'apex_top_y': float(getattr(self, '_apex_top_y', 0.0)),
                'apex_cx': float(getattr(self, '_apex_cx', 0.0)),
                'apex_left_x': float(getattr(self, '_apex_left_x', 0.0)),
                'apex_right_x': float(getattr(self, '_apex_right_x', 0.0)),
                'apex_sample_x': float(getattr(self, '_apex_sample_x', 0.0)),
                'apex_sample_y': float(getattr(self, '_apex_sample_y', 0.0)),
                'apex_center_src': str(getattr(self, '_apex_center_src', 'none')),
                'apex_left5_x0': float(getattr(self, '_apex_left5_x0', 0.0)),
                'apex_left5_x1': float(getattr(self, '_apex_left5_x1', 0.0)),
                'apex_right5_x0': float(getattr(self, '_apex_right5_x0', 0.0)),
                'apex_right5_x1': float(getattr(self, '_apex_right5_x1', 0.0)),
                'apex_left10_x0': float(getattr(self, '_apex_left10_x0', 0.0)),
                'apex_left10_x1': float(getattr(self, '_apex_left10_x1', 0.0)),
                'apex_right10_x0': float(getattr(self, '_apex_right10_x0', 0.0)),
                'apex_right10_x1': float(getattr(self, '_apex_right10_x1', 0.0)),
                'apex_c10_x0': float(getattr(self, '_apex_c10_x0', 0.0)),
                'apex_c10_x1': float(getattr(self, '_apex_c10_x1', 0.0)),
                'apex_far_row_fill': float(getattr(self, '_apex_far_row_fill', 0.0)),
                # Pure Pursuit 路径信息
                'path_samples': getattr(self, '_latest_path_samples', []),  # [(x_m, y_m), ...]
                'lookahead_point': getattr(self, '_latest_lookahead_point', None),  # (x_m, y_m)
                'lateral_error_m': getattr(self, '_latest_lateral_error_m', 0.0),
            }

    def _extract_centerline_from_mask(self, mask):
        """多行采样 mask 中心线（优化版：偏向远处，同时提取边界）。

        返回:
            samples: list[(x_px, y_px)] 从近到远的中心点（像素坐标）
            error: 归一化横向误差 [-1,1]，负=偏左，正=偏右
            curve: 近远场中点差估计曲率
            confidence: 0~1
            target_xy: 前瞻点（像素坐标）
            path_samples_m: list[(x_m, y_m)] 世界坐标系下的路径点（相对车体）
            lookahead_point_m: (x_m, y_m) 前瞻点世界坐标
            lateral_error_m: float 实际横向误差（米）
        """
        if mask is None or mask.size == 0:
            return [], None, 0.0, 0.0, None, [], None, 0.0
        h, w = mask.shape[:2]
        if h < 8 or w < 8:
            return [], None, 0.0, 0.0, None, [], None, 0.0

        # 采样范围优化：更偏向远处（减少近处干扰）
        rows = np.linspace(int(h * 0.70), int(h * 0.15), self._sample_rows).astype(int)

        samples = []  # 中心线（像素坐标）
        left_edges = []  # 左边界
        right_edges = []  # 右边界

        for y in rows:
            y0 = max(0, y - 1)
            y1 = min(h, y + 2)
            xs = np.where(mask[y0:y1, :] > 0)[1]
            if xs.size < self._min_mask_pixels_per_row:
                continue

            # 中心点
            cx = float(xs.mean())
            samples.append((cx, float(y)))

            # 左右边界
            left_edge = float(xs.min())
            right_edge = float(xs.max())
            left_edges.append((left_edge, float(y)))
            right_edges.append((right_edge, float(y)))

        # 至少 3 行即可给出弱中线
        min_rows = max(3, min(self._min_valid_rows, self._sample_rows))
        if len(samples) < min_rows and len(samples) < 3:
            return [], None, 0.0, 0.0, None, [], None, 0.0
        if len(samples) < self._min_valid_rows:
            pass

        # samples 已近→远；前瞻点取偏远一点
        target_idx = int(round((len(samples) - 1) * max(0.0, min(1.0, 1.0 - self._lookahead_ratio))))
        target_idx = max(0, min(len(samples) - 1, target_idx))
        target = samples[target_idx]
        mid_x = 0.5 * float(w)
        raw_error = (target[0] - mid_x) / max(mid_x, 1.0)
        raw_error = float(np.clip(raw_error, -1.0, 1.0))

        near = samples[0][0]
        far = samples[-1][0]
        curve = float(np.clip((far - near) / max(mid_x, 1.0), -1.0, 1.0))
        # 近场/远场横向误差 + 路径弯曲（航向主量）
        # 负=中线在左（车偏右），正=中线在右（车偏左）——与 raw_error 同号约定
        near_error = float(np.clip((near - mid_x) / max(mid_x, 1.0), -1.0, 1.0))
        far_error = float(np.clip((far - mid_x) / max(mid_x, 1.0), -1.0, 1.0))
        # 取中段一点增强稳定
        mid_s = samples[len(samples) // 2][0]
        mid_error = float(np.clip((mid_s - mid_x) / max(mid_x, 1.0), -1.0, 1.0))
        path_bend = float(np.clip((far - near) / max(mid_x, 1.0), -1.0, 1.0))  # 同 curve
        self._near_error = near_error
        self._far_error = far_error
        self._path_bend = path_bend
        # 航向误差：远场权重大 → 弯前更早打方向
        raw_error = float(np.clip(0.35 * near_error + 0.25 * mid_error + 0.40 * far_error, -1.0, 1.0))
        # 用户规则：画面中心左右各 10% 竖带占满 mask → 可判居中可直行
        band = max(1, int(round(w * 0.10)))  # 半宽 10%
        band5 = max(1, int(round(w * 0.05)))  # 半宽 5%（纠偏回正）
        x0 = max(0, int(mid_x) - band)
        x1 = min(w, int(mid_x) + band)
        x0_5 = max(0, int(mid_x) - band5)
        x1_5 = min(w, int(mid_x) + band5)
        # 用近场 40% 高度，避免远处对面赛道
        y0 = int(h * 0.45)
        center_band = mask[y0:, x0:x1] if mask.ndim >= 2 else None
        center_band5 = mask[y0:, x0_5:x1_5] if mask.ndim >= 2 else None
        if center_band is not None and center_band.size > 0:
            if center_band.dtype == np.uint8 and center_band.max() > 1:
                fill = float((center_band > 127).mean())
            else:
                fill = float((center_band > 0.5).mean())
        else:
            fill = 0.0
        if center_band5 is not None and center_band5.size > 0:
            if center_band5.dtype == np.uint8 and center_band5.max() > 1:
                fill5 = float((center_band5 > 127).mean())
            else:
                fill5 = float((center_band5 > 0.5).mean())
        else:
            fill5 = 0.0
        self._center_fill = float(np.clip(fill, 0.0, 1.0))
        self._center_fill_5 = float(np.clip(fill5, 0.0, 1.0))
        # 最高点（最远端中线）：有没有黄/SEG，偏哪边
        # samples 近→远，最远点 = samples[-1]
        # 关键：顶点必须锚定“本车道中线远端”，绝不能取画面最上任意SEG
        # （环形时对面墙/对面道会在更上方出现，会把 apex 拉飞）
        far_x, far_y = samples[-1]
        self._apex_error = float(np.clip((far_x - mid_x) / max(mid_x, 1.0), -1.0, 1.0))
        self._apex_sample_x = float(far_x)
        self._apex_sample_y = float(far_y)

        if mask.dtype == np.uint8 and mask.max() > 1:
            thresh = 127
        else:
            thresh = 0.5
        binary = mask > thresh
        top20_h = max(1, int(round(h * 0.20)))
        self._top20_seg_fill = float(binary[:top20_h, :].mean())

        # 中线近→远的横向锚点：用远端 2~3 个采样平均，防单点跳边
        n_anchor = max(1, min(3, len(samples)))
        anchor_xs = [float(p[0]) for p in samples[-n_anchor:]]
        anchor_x = float(np.median(np.asarray(anchor_xs, dtype=np.float64)))
        # 近场中心也参与约束：弯中 far 可能贴边，但近场仍在本道
        near_anchor = float(samples[0][0]) if samples else float(w) * 0.5
        # 混合：远端为主，近场拖一点，避免瞬间飞到对面
        anchor_x = float(0.75 * anchor_x + 0.25 * near_anchor)
        anchor_x = float(np.clip(anchor_x, 0.0, float(w - 1)))

        # 顶点 y：优先中线最远采样行；仅当该行附近几乎无SEG时，再在“锚点附近”
        # 向上最多 18% 高度内找本车道顶端（不是全图最上）
        apex_top_y = int(max(0, min(h - 1, far_y)))
        min_row_px = max(3, int(round(w * 0.01)))
        search_y0 = max(0, int(far_y) - max(2, int(round(h * 0.18))))
        search_y1 = min(h, int(far_y) + 2)
        local_top = None
        x_gate = max(8, int(round(w * 0.28)))  # 只认锚点左右 28% 宽内的 SEG
        for yy in range(search_y0, search_y1):
            row = binary[yy, :]
            xs = np.where(row)[0]
            if xs.size < min_row_px:
                continue
            # 取最靠近锚点的连通段
            best_l = best_r = None
            best_dist = 1e18
            start = int(xs[0])
            prev = int(xs[0])
            for xv in xs[1:]:
                xv = int(xv)
                if xv > prev + 1:
                    seg_l, seg_r = start, prev
                    seg_c = 0.5 * (seg_l + seg_r)
                    dist = abs(seg_c - anchor_x)
                    # 丢弃几乎铺满整行的墙面残渣
                    if (seg_r - seg_l) < 0.72 * float(w) and dist < best_dist:
                        best_dist = dist
                        best_l, best_r = seg_l, seg_r
                    start = xv
                prev = xv
            seg_l, seg_r = start, prev
            seg_c = 0.5 * (seg_l + seg_r)
            dist = abs(seg_c - anchor_x)
            if (seg_r - seg_l) < 0.72 * float(w) and dist < best_dist:
                best_dist = dist
                best_l, best_r = seg_l, seg_r
            if best_l is None:
                continue
            # 段中心必须在锚点门内，否则是对面道
            if abs(0.5 * (best_l + best_r) - anchor_x) > x_gate:
                continue
            local_top = yy
            break  # 从远侧向上扫到的第一条本道SEG
        if local_top is not None:
            apex_top_y = int(local_top)

        apex_band_h = max(2, int(round(h * 0.10)))
        apex_y0 = max(0, apex_top_y)
        apex_y1 = min(h, apex_y0 + apex_band_h)
        if apex_y1 <= apex_y0:
            apex_y1 = min(h, apex_y0 + 1)

        # 顶点左右边界：逐行取“靠近锚点”的本道连通段，再中位数
        # 禁止对整带 min/max —— 那会把对面墙/满宽残渣并进来（日志 L=0 R=573）
        row_lefts = []
        row_rights = []
        row_cxs = []
        for yy in range(apex_y0, apex_y1):
            xs = np.where(binary[yy, :])[0]
            if xs.size < min_row_px:
                continue
            best_l = best_r = None
            best_dist = 1e18
            start = int(xs[0])
            prev = int(xs[0])
            for xv in xs[1:]:
                xv = int(xv)
                if xv > prev + 1:
                    seg_l, seg_r = start, prev
                    width = seg_r - seg_l
                    if width >= 0.72 * float(w):
                        start = xv
                        prev = xv
                        continue
                    seg_c = 0.5 * (seg_l + seg_r)
                    dist = abs(seg_c - anchor_x)
                    if dist < best_dist:
                        best_dist = dist
                        best_l, best_r = seg_l, seg_r
                    start = xv
                prev = xv
            seg_l, seg_r = start, prev
            width = seg_r - seg_l
            if width < 0.72 * float(w):
                seg_c = 0.5 * (seg_l + seg_r)
                dist = abs(seg_c - anchor_x)
                if dist < best_dist:
                    best_dist = dist
                    best_l, best_r = seg_l, seg_r
            if best_l is None:
                continue
            if abs(0.5 * (best_l + best_r) - anchor_x) > x_gate:
                continue
            row_lefts.append(float(best_l))
            row_rights.append(float(best_r))
            row_cxs.append(0.5 * (float(best_l) + float(best_r)))

        apex_left_x = None
        apex_right_x = None
        apex_cx = None
        apex_center_src = 'none'
        img_cx = float(w) * 0.5
        if len(row_cxs) >= 1:
            apex_left_x = float(np.median(np.asarray(row_lefts, dtype=np.float64)))
            apex_right_x = float(np.median(np.asarray(row_rights, dtype=np.float64)))
            # 边界中点与锚点再融合，防止段选偏
            bound_cx = 0.5 * (apex_left_x + apex_right_x)
            apex_cx = float(0.65 * bound_cx + 0.35 * anchor_x)
            apex_center_src = 'boundary'
        elif samples:
            apex_cx = float(far_x)
            apex_center_src = 'sample'
            # 用采样行的左右边作弱边界
            if left_edges and right_edges:
                # left_edges/right_edges 与 samples 同步近→远
                apex_left_x = float(left_edges[-1][0])
                apex_right_x = float(right_edges[-1][0])
            else:
                apex_left_x = float(apex_cx)
                apex_right_x = float(apex_cx)
        else:
            apex_cx = img_cx
            apex_center_src = 'image'
            apex_left_x = float(apex_cx)
            apex_right_x = float(apex_cx)

        # 边界宽度异常（几乎整幅）→ 退回采样中线
        if (
            apex_left_x is not None
            and apex_right_x is not None
            and (apex_right_x - apex_left_x) >= 0.72 * float(w)
        ):
            apex_cx = float(far_x)
            apex_left_x = float(far_x)
            apex_right_x = float(far_x)
            apex_center_src = 'sample'

        apex_cx = float(np.clip(apex_cx, 0.0, float(w - 1)))
        ax = int(round(apex_cx))
        ay = int(max(0, min(h - 1, apex_top_y)))

        # 远端小窗：仍用于 apex_fill / apex_has_mask 兼容
        hy = max(2, int(round(h * 0.04)))
        hx = max(2, int(round(w * 0.08)))
        y_a0 = max(0, ay - hy)
        y_a1 = min(h, ay + hy + 1)
        x_a0 = max(0, ax - hx)
        x_a1 = min(w, ax + hx + 1)
        apex_win = mask[y_a0:y_a1, x_a0:x_a1] if mask.ndim >= 2 else None
        if apex_win is not None and apex_win.size > 0:
            if apex_win.dtype == np.uint8 and apex_win.max() > 1:
                apex_fill = float((apex_win > 127).mean())
            else:
                apex_fill = float((apex_win > 0.5).mean())
        else:
            apex_fill = 0.0
        # 远端行：只统计锚点附近，避免对面墙把 far_row 灌满
        far_x0 = max(0, ax - max(8, int(round(w * 0.22))))
        far_x1 = min(w, ax + max(8, int(round(w * 0.22))))
        far_row = mask[max(0, ay - 1):min(h, ay + 2), far_x0:far_x1] if mask.ndim >= 2 else None
        if far_row is not None and far_row.size > 0:
            if far_row.dtype == np.uint8 and far_row.max() > 1:
                far_row_fill = float((far_row > 127).mean())
            else:
                far_row_fill = float((far_row > 0.5).mean())
        else:
            far_row_fill = 0.0
        self._apex_fill = float(np.clip(max(apex_fill, far_row_fill * 0.6), 0.0, 1.0))
        # 无黄：小窗几乎空 且 远端行覆盖很低
        self._apex_has_mask = bool(self._apex_fill >= 0.12 or far_row_fill >= 0.08)
        self._apex_far_row_fill = float(np.clip(far_row_fill, 0.0, 1.0))

        # 用户核心判据：顶点区域中线左右各5%/10%/30%的mask占比
        # 纠偏完成：顶点中心线左右各5%有SEG
        # 转弯完成：顶点中心线±10%有SEG
        band5_apex = max(1, int(round(w * 0.05)))
        band10_apex = max(1, int(round(w * 0.10)))
        band30_apex = max(1, int(round(w * 0.30)))

        apex_left5_x0 = max(0, ax - band5_apex)
        apex_left5_x1 = ax
        apex_right5_x0 = ax
        apex_right5_x1 = min(w, ax + band5_apex)
        apex_left10_x0 = max(0, ax - band10_apex)
        apex_left10_x1 = ax
        apex_right10_x0 = ax
        apex_right10_x1 = min(w, ax + band10_apex)
        apex_left30_x0 = max(0, ax - band30_apex)
        apex_left30_x1 = ax
        apex_right30_x0 = ax
        apex_right30_x1 = min(w, ax + band30_apex)

        # 顶点区域窗口 = 本道远端顶点往下10%高度
        apex_row_win = mask[apex_y0:apex_y1, :]

        apex_left5_fill = 0.0
        apex_right5_fill = 0.0
        apex_left10_fill = 0.0
        apex_right10_fill = 0.0
        apex_center10_fill = 0.0
        apex_left30_fill = 0.0
        apex_right30_fill = 0.0

        if apex_row_win is not None and apex_row_win.size > 0:
            if apex_row_win.dtype == np.uint8 and apex_row_win.max() > 1:
                win_thresh = 127
            else:
                win_thresh = 0.5

            # 左5%
            left5_win = apex_row_win[:, apex_left5_x0:apex_left5_x1]
            if left5_win.size > 0:
                apex_left5_fill = float((left5_win > win_thresh).mean())

            # 右5%
            right5_win = apex_row_win[:, apex_right5_x0:apex_right5_x1]
            if right5_win.size > 0:
                apex_right5_fill = float((right5_win > win_thresh).mean())

            # 左10%
            left10_win = apex_row_win[:, apex_left10_x0:apex_left10_x1]
            if left10_win.size > 0:
                apex_left10_fill = float((left10_win > win_thresh).mean())

            # 右10%
            right10_win = apex_row_win[:, apex_right10_x0:apex_right10_x1]
            if right10_win.size > 0:
                apex_right10_fill = float((right10_win > win_thresh).mean())

            left30_win = apex_row_win[:, apex_left30_x0:apex_left30_x1]
            if left30_win.size > 0:
                apex_left30_fill = float((left30_win > win_thresh).mean())
            right30_win = apex_row_win[:, apex_right30_x0:apex_right30_x1]
            if right30_win.size > 0:
                apex_right30_fill = float((right30_win > win_thresh).mean())

            # 中心±10%：要求左右都有一点才算“中线居中”
            # 单侧墙面把整带灌满时，C10 会被抬高 → 再乘左右平衡系数
            center10_win = apex_row_win[:, apex_left10_x0:apex_right10_x1]
            if center10_win.size > 0:
                raw_c10 = float((center10_win > win_thresh).mean())
            else:
                raw_c10 = 0.0
            # 左右都要有：min(L10,R10) 太低则压低 C10，避免墙面假回正
            side_bal = float(min(apex_left10_fill, apex_right10_fill))
            if side_bal < 0.12:
                apex_center10_fill = raw_c10 * (side_bal / 0.12)
            else:
                apex_center10_fill = raw_c10

        self._apex_left5_fill = float(np.clip(apex_left5_fill, 0.0, 1.0))
        self._apex_right5_fill = float(np.clip(apex_right5_fill, 0.0, 1.0))
        self._apex_left10_fill = float(np.clip(apex_left10_fill, 0.0, 1.0))
        self._apex_right10_fill = float(np.clip(apex_right10_fill, 0.0, 1.0))
        self._apex_center10_fill = float(np.clip(apex_center10_fill, 0.0, 1.0))
        self._apex_left30_fill = float(np.clip(apex_left30_fill, 0.0, 1.0))
        self._apex_right30_fill = float(np.clip(apex_right30_fill, 0.0, 1.0))
        self._apex_y0 = float(apex_y0)
        self._apex_y1 = float(apex_y1)
        self._apex_top_y = float(apex_top_y)
        self._apex_cx = float(apex_cx)
        self._apex_left_x = float(apex_left_x if apex_left_x is not None else apex_cx)
        self._apex_right_x = float(apex_right_x if apex_right_x is not None else apex_cx)
        self._apex_center_src = str(apex_center_src)
        self._apex_left5_x0 = float(apex_left5_x0)
        self._apex_left5_x1 = float(apex_left5_x1)
        self._apex_right5_x0 = float(apex_right5_x0)
        self._apex_right5_x1 = float(apex_right5_x1)
        self._apex_left10_x0 = float(apex_left10_x0)
        self._apex_left10_x1 = float(apex_left10_x1)
        self._apex_right10_x0 = float(apex_right10_x0)
        self._apex_right10_x1 = float(apex_right10_x1)
        self._apex_c10_x0 = float(apex_left10_x0)
        self._apex_c10_x1 = float(apex_right10_x1)
        # 中心带足够满 → 强制 lane_clear（用户指定）
        if self._center_fill >= 0.55:
            self._lane_clear = True

        # 置信度：综合考虑行数覆盖率 + 边界一致性
        coverage = float(len(samples)) / float(max(self._sample_rows, 1))

        boundary_consistency = 1.0
        if len(left_edges) >= 3:
            left_xs = [p[0] for p in left_edges]
            right_xs = [p[0] for p in right_edges]
            left_std = float(np.std(left_xs))
            right_std = float(np.std(right_xs))
            boundary_consistency = max(0.0, 1.0 - (left_std + right_std) / (w * 0.05))

        confidence = min(1.0, coverage * 0.7 + boundary_consistency * 0.3)
        if len(samples) < self._min_valid_rows:
            confidence *= 0.55

        # 存储边界信息供外部使用
        if not hasattr(self, '_latest_left_boundary'):
            self._latest_left_boundary = []
            self._latest_right_boundary = []
        self._latest_left_boundary = left_edges
        self._latest_right_boundary = right_edges
        # 近场左右边界：用最近若干行
        # 关键改用「赛道中心相对画面中心」判居中，而不是「是否离开画面左右缘」。
        # 裁切 FOV 下，居中时 mask 也可能贴画面边，旧 margin 会永远 clr=0。
        n_edge = max(1, min(3, len(left_edges)))
        if left_edges and right_edges:
            l_near = float(sum(p[0] for p in left_edges[:n_edge]) / n_edge)
            r_near = float(sum(p[0] for p in right_edges[:n_edge]) / n_edge)
            half = max(mid_x, 1.0)
            lane_cx = 0.5 * (l_near + r_near)
            lane_half_w = 0.5 * max(1.0, r_near - l_near)
            # 归一化：0=画面中心，+1=右半幅
            center_off = float(np.clip((lane_cx - mid_x) / half, -1.5, 1.5))
            # margin：边界到画面边的归一化距离（仅诊断）
            self._left_margin = float(np.clip(l_near / half, 0.0, 2.0))
            self._right_margin = float(np.clip((float(w) - 1.0 - r_near) / half, 0.0, 2.0))
            # 相对裕度：边界到画面中心的内侧空间 / 半宽（更稳）
            # 左边界在 mid 左侧越远，左相对裕度越大
            rel_l = float(np.clip((mid_x - l_near) / half, 0.0, 2.0))
            rel_r = float(np.clip((r_near - mid_x) / half, 0.0, 2.0))
            # 居中：赛道中心靠近画面中心 + 左右都有一定宽度
            width_ok = (r_near - l_near) > 0.18 * float(w)
            self._lane_clear = bool(
                width_ok
                and abs(center_off) < 0.18
                and rel_l > 0.08
                and rel_r > 0.08
            )
            # 覆盖 near_error 用赛道中心（比 mask 质心更稳）
            self._lane_center_off = center_off
            self._rel_left = rel_l
            self._rel_right = rel_r
        else:
            self._left_margin = 0.0
            self._right_margin = 0.0
            self._lane_clear = False
            self._lane_center_off = 0.0
            self._rel_left = 0.0
            self._rel_right = 0.0

        # ═══ 像素坐标 → 世界坐标转换 ═══
        # 相机模型简化假设：
        # - 图像底部 = 车前 0.1m，图像顶部 = 车前 2.5m
        # - 图像宽度在前瞻距离处约为 1.2m
        # - 坐标系：车体为原点，X=横向（右为正），Y=纵向（前为正）

        near_distance = 0.1  # 图像底部对应的实际距离（米）
        far_distance = 2.5   # 图像顶部对应的实际距离（米）
        fov_width_at_lookahead = 1.2  # 在前瞻距离处的视野宽度（米）

        path_samples_m = []
        for px, py in samples:
            # Y坐标：线性映射 y_px → distance
            ratio_y = 1.0 - (py / float(h))  # 从下往上，0→1
            y_m = near_distance + ratio_y * (far_distance - near_distance)

            # X坐标：归一化横向偏移 * 视野宽度
            ratio_x = (px - mid_x) / mid_x  # [-1, 1]
            # 视野宽度随距离线性变化（近处窄，远处宽）
            fov_width = fov_width_at_lookahead * (y_m / far_distance)
            x_m = ratio_x * (fov_width / 2.0)

            path_samples_m.append((x_m, y_m))

        # 前瞻点世界坐标
        lookahead_point_m = path_samples_m[target_idx] if target_idx < len(path_samples_m) else None

        # 横向误差（米）
        lateral_error_m = lookahead_point_m[0] if lookahead_point_m else 0.0

        return samples, raw_error, curve, confidence, target, path_samples_m, lookahead_point_m, lateral_error_m

    def _estimate_offset_from_mask(self, mask, fallback_cx=None, fallback_cy=None):
        """
        由 SEG mask 估计横向 offset。

        规则：
        1) 中心竖带 mask 占比足够高 → 已居中，offset=0
        2) 否则用近场（底部）mask 质心相对画面中心
        3) mask 为空时回退 bbox 中心（由调用方传入 fallback_cx）

        返回：(offset, center_ratio, centered, cx, cy)
            offset 为 None 表示需调用方用 bbox 回退
        """
        if mask is None or mask.size == 0:
            return None, 0.0, False, fallback_cx, fallback_cy

        h, w = mask.shape[:2]
        if h < 4 or w < 4:
            return None, 0.0, False, fallback_cx, fallback_cy

        binary = (mask > 0).astype(np.uint8)
        half_band = max(1, int(round(w * float(self._offset_center_band) * 0.5)))
        mid = w // 2
        x0 = max(0, mid - half_band)
        x1 = min(w, mid + half_band)
        band = binary[:, x0:x1]
        center_ratio = float(band.mean()) if band.size else 0.0

        # 中心竖带已被赛道占满 → 到位停纠
        if center_ratio >= float(self._offset_center_occ_thresh):
            return 0.0, center_ratio, True, float(mid), float(h * 0.75)

        # 近场质心：只用底部若干行，减少远景干扰
        bottom_ratio = float(self._offset_centroid_bottom_ratio)
        y_start = int(h * (1.0 - bottom_ratio))
        y_start = max(0, min(h - 1, y_start))
        near = binary[y_start:, :]
        if np.any(near > 0):
            ys, xs = np.where(near > 0)
            cx = float(xs.mean())
            cy = float(ys.mean() + y_start)
        else:
            ys, xs = np.where(binary > 0)
            if xs.size == 0:
                return None, center_ratio, False, fallback_cx, fallback_cy
            cx = float(xs.mean())
            cy = float(ys.mean())

        offset = cx / (w / 2.0) - 1.0
        offset = float(np.clip(offset, -1.0, 1.0))
        return offset, center_ratio, False, cx, cy

    def get_latest_remaining(self):

        """
        获取最新的视觉剩余距离估计。

        返回：
            (remaining_m, free_ratio, timestamp, valid)
            - remaining_m: float|None，前方赛道剩余距离粗估值（m）
            - free_ratio: float，中心带赛道向上延伸比例 [0,1]
            - timestamp: float，检测时刻
            - valid: bool，是否有效
        """
        with self._lock:
            now = time.time()
            age = now - self._latest_timestamp
            valid = (
                self._valid
                and self._range_valid
                and self._latest_remaining_m is not None
                and (age < self._detection_timeout_sec)
            )
            return (
                self._latest_remaining_m,
                float(self._latest_free_ratio),
                self._latest_timestamp,
                valid,
            )

    def _estimate_remaining_from_mask(self, mask):
        """由赛道 mask 中心带顶部位置粗估前方剩余距离。"""
        if mask is None or mask.size == 0:
            return None, 0.0
        h, w = mask.shape[:2]
        if h < 8 or w < 8:
            return None, 0.0

        band = max(1, int(round(w * self._range_center_band * 0.5)))
        cx = w // 2
        x0 = max(0, cx - band)
        x1 = min(w, cx + band)
        center = mask[:, x0:x1]
        if center.size == 0:
            return None, 0.0

        row_occ = center.mean(axis=1)
        occupied = np.where(row_occ > self._range_occ_thresh)[0]
        if occupied.size == 0:
            return float(self._range_near_m), 0.0

        top_y = int(occupied.min())
        free_ratio = 1.0 - (top_y / float(max(1, h - 1)))
        free_ratio = float(np.clip(free_ratio, 0.0, 1.0))
        remaining = self._range_near_m + free_ratio * (self._range_far_m - self._range_near_m)
        return float(remaining), free_ratio

    def _check_lane_boundary_safety(self, mask, error):
        """
        检测车辆是否接近mask边界，触发保护性控制。

        Args:
            mask: 赛道分割mask (H, W)
            error: 横向误差 [-1, 1]，负=偏左，正=偏右

        Returns:
            (boundary_safe: bool, safety_weight: float)
            - boundary_safe: True=安全, False=接近边界
            - safety_weight: 视觉权重系数 [0.3, 1.0]，越危险越小
        """
        if mask is None or mask.size == 0:
            return True, 1.0  # 无mask，假设安全（由上层处理失效）

        h, w = mask.shape[:2]
        if h < 8 or w < 8:
            return True, 1.0

        # 检查左右两侧mask覆盖情况（侧边安全裕度区域）
        margin_px = max(1, int(w * self._boundary_safety_margin))

        left_region = mask[:, :margin_px]
        right_region = mask[:, -margin_px:]

        left_coverage = float(left_region.mean()) if left_region.size > 0 else 0.0
        right_coverage = float(right_region.mean()) if right_region.size > 0 else 0.0

        thresh = self._boundary_coverage_thresh

        # 如果车辆往某侧偏移，但该侧mask稀疏→危险
        # 误差阈值：0.25（中等偏离），0.40（严重偏离）
        if error < -0.25 and left_coverage < thresh:  # 左偏且左侧无路
            if error < -0.40:  # 严重左偏
                return False, 0.30  # 视觉权重降至30%，IMU主导
            else:
                return False, 0.55  # 中等左偏，混合控制

        if error > 0.25 and right_coverage < thresh:  # 右偏且右侧无路
            if error > 0.40:  # 严重右偏
                return False, 0.30
            else:
                return False, 0.55

        # 双侧都无路（窄通道末端）→极度危险
        if left_coverage < thresh and right_coverage < thresh:
            return False, 0.20  # 几乎完全IMU接管

        return True, 1.0  # 安全

    def _detect_boundary_ahead(self, mask):
        """
        前边界 / 直道几何（可通行 mask 的远端截断）。

        返回字段:
          boundary_ahead: 前方有横截边界（准备转弯）
          front_score / straight_score: 0~1 连续分数
          far/mid/near_ratio, left/right_ratio, coverage_std, top_y_ratio
        注意: 左右侧空不作转向方向，方向由扫码环向决定。
        """
        empty = {
            'boundary_ahead': False,
            'distance_ratio': 0.0,
            'far_ratio': 0.0,
            'mid_ratio': 0.0,
            'near_ratio': 0.0,
            'coverage_std': 0.0,
            'top_y_ratio': 0.0,
            'front_score': 0.0,
            'straight_score': 0.0,
            'left_ratio': 0.0,
            'right_ratio': 0.0,
            'edge_angle_deg': 90.0,
            'perp_score': 0.0,
        }
        if mask is None or mask.size == 0:
            return empty

        h, w = mask.shape[:2]
        if h < 20 or w < 20:
            return empty

        if mask.dtype == np.uint8 and mask.max() > 1:
            mask_norm = (mask > 127).astype(np.float32)
        else:
            mask_norm = (mask > 0.5).astype(np.float32)

        third = h // 3
        far_region = mask_norm[0:third, :]
        mid_region = mask_norm[third:2 * third, :]
        near_region = mask_norm[2 * third:, :]

        far_ratio = float(far_region.mean()) if far_region.size > 0 else 0.0
        mid_ratio = float(mid_region.mean()) if mid_region.size > 0 else 0.0
        near_ratio = float(near_region.mean()) if near_region.size > 0 else 0.0
        left_ratio = float(mask_norm[:, : max(1, w // 3)].mean())
        right_ratio = float(mask_norm[:, (2 * w) // 3 :].mean())

        row_has_mask = (mask_norm.sum(axis=1) > w * 0.08)
        top_rows = np.where(row_has_mask)[0]
        if len(top_rows) == 0:
            out = dict(empty)
            out.update({
                'far_ratio': far_ratio,
                'mid_ratio': mid_ratio,
                'near_ratio': near_ratio,
                'left_ratio': left_ratio,
                'right_ratio': right_ratio,
                'front_score': 1.0 if near_ratio < 0.15 else 0.65,
                'straight_score': 0.0,
            })
            return out

        top_y = int(top_rows[0])
        top_y_ratio = top_y / float(max(1, h - 1))
        distance_ratio = top_y_ratio

        top_region = mask_norm[max(0, top_y - 5):min(h, top_y + 12), :]
        if top_region.size == 0:
            coverage_std = 1.0
        else:
            col_coverage = top_region.mean(axis=0)
            coverage_std = float(np.std(col_coverage))

        gap = max(0.0, near_ratio - far_ratio)
        front_score = 0.0
        if near_ratio > 0.28:
            front_score += max(0.0, 0.48 - far_ratio) / 0.48 * 0.40
            front_score += min(1.0, gap / 0.45) * 0.35
            front_score += min(1.0, max(0.0, top_y_ratio - 0.06) / 0.35) * 0.15
            front_score += max(0.0, 0.38 - coverage_std) / 0.38 * 0.10
        front_score = float(np.clip(front_score, 0.0, 1.0))

        boundary_ahead = (
            front_score >= 0.52
            and far_ratio < 0.40
            and near_ratio > 0.38
            and gap >= 0.18
        )
        if (not boundary_ahead) and far_ratio < 0.22 and near_ratio > 0.48:
            boundary_ahead = True
            front_score = max(front_score, 0.70)

        straight_score = 0.0
        straight_score += min(1.0, near_ratio / 0.70) * 0.30
        straight_score += min(1.0, mid_ratio / 0.65) * 0.30
        straight_score += min(1.0, far_ratio / 0.55) * 0.30
        if gap < 0.20 and far_ratio > 0.40:
            straight_score += 0.10
        if coverage_std < 0.25:
            straight_score += 0.05
        straight_score = float(np.clip(straight_score, 0.0, 1.0))
        if boundary_ahead:
            straight_score *= 0.45

        # 前边界倾角：对截断带做列方向梯度，估计边相对水平的角度
        # 0°=水平横边（正对车头），90°=竖边
        edge_angle_deg = 90.0  # 默认当竖/未知
        try:
            band0 = max(0, top_y - 2)
            band1 = min(h, top_y + 15)
            band = mask_norm[band0:band1, :]
            if band.size > 0 and band1 > band0 + 2:
                # 每列最上有 mask 的 y
                col_top = []
                for c in range(0, w, max(1, w // 40)):
                    col = mask_norm[:, c]
                    ys = np.where(col > 0.5)[0]
                    if len(ys):
                        col_top.append((float(c), float(ys[0])))
                if len(col_top) >= 4:
                    xs = np.array([p[0] for p in col_top], dtype=np.float64)
                    ys = np.array([p[1] for p in col_top], dtype=np.float64)
                    # 拟合 y = a*x + b，斜率 a；水平边 a≈0 → angle_from_horizontal≈0
                    a = float(np.polyfit(xs, ys, 1)[0])
                    # 边相对水平的角度（度），0=横边，90=竖边
                    edge_angle_deg = float(abs(np.degrees(np.arctan(a))))
                    edge_angle_deg = min(90.0, edge_angle_deg)
        except Exception:
            edge_angle_deg = 90.0

        # 相对车头前进方向：横边≈垂直于前进方向
        # free-space 顶部常拟合出“假水平边”，角度只能在远端已空时加分
        perp_score = float(np.clip((28.0 - edge_angle_deg) / 28.0, 0.0, 1.0))
        # 只有 far 真截断时，才允许角度抬 front / ba
        if edge_angle_deg <= 28.0 and near_ratio > 0.35 and far_ratio < 0.34:
            front_score = max(front_score, 0.40 + 0.35 * perp_score)
            if edge_angle_deg <= 20.0 and far_ratio < 0.28 and near_ratio > 0.42:
                boundary_ahead = True
                front_score = max(front_score, 0.68)
        # far 还开着：禁止因角度误触发
        if far_ratio >= 0.40:
            boundary_ahead = False if not (near_ratio > 0.55 and far_ratio < 0.22) else boundary_ahead
            # 不强行抬 front

        return {
            'boundary_ahead': bool(boundary_ahead),
            'distance_ratio': float(distance_ratio),
            'far_ratio': float(far_ratio),
            'mid_ratio': float(mid_ratio),
            'near_ratio': float(near_ratio),
            'coverage_std': float(coverage_std),
            'top_y_ratio': float(top_y_ratio),
            'front_score': float(front_score),
            'straight_score': float(straight_score),
            'left_ratio': float(left_ratio),
            'right_ratio': float(right_ratio),
            'edge_angle_deg': float(edge_angle_deg),
            'perp_score': float(perp_score),
        }

    def _start_http_server(self):
        """启动 HTTP 静态文件服务器（后台线程，多线程处理请求）"""
        parent_self = self  # 闭包引用
        
        def serve():
            try:
                parent_self._node.get_logger().info('[视觉] HTTP 服务线程启动中...')
                
                # 自定义 Handler，禁用缓存 + /health 端点 + CORS 支持 + 静默 BrokenPipeError
                class NoCacheHandler(SimpleHTTPRequestHandler):
                    def do_OPTIONS(self):
                        """处理 CORS preflight 请求"""
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
                        self.send_header('Access-Control-Allow-Headers', 'Content-Type, Pragma, Cache-Control')
                        self.send_header('Access-Control-Max-Age', '86400')
                        self.end_headers()
                    
                    def do_GET(self):
                        if self.path in ('/', '/index.html', '/seg.html'):
                            body = f'''<!doctype html>
<html><head><meta charset="utf-8"><title>Stage2 SEG</title>
<style>body{{background:#202124;color:#eee;font-family:sans-serif;margin:20px}}
img{{max-width:100%;border:1px solid #555}}#status{{margin:10px 0;color:#8f8}}</style></head>
<body><h2>Stage2 SEG 实时推理画面</h2>
<p>ROI=下方40%+中间60%宽；mask质心纠偏，中心竖带到位则停</p>
<div id="status">连接中...</div>
<img id="frame" src="/stream.mjpg">
<script>
const status=document.getElementById('status');
async function health(){{ try{{ const r=await fetch('/health?t='+Date.now(),{{cache:'no-store'}}); const d=await r.json();
status.textContent='status='+d.status+' | frames='+d.frame_count+' | age='+d.frame_age_sec+'s';
}}catch(e){{status.textContent='HTTP 等待中: '+e;}} }}
setInterval(health, 500); health();
</script></body></html>'''.encode('utf-8')
                            self.send_response(200)
                            self.send_header('Content-Type', 'text/html; charset=utf-8')
                            self.send_header('Content-Length', len(body))
                            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                            self.end_headers()
                            self.wfile.write(body)
                            return

                        if self.path == '/health' or self.path.startswith('/health?'):
                            # 健康检查：返回服务状态 + 最后一帧时间
                            try:
                                with parent_self._lock:
                                    last_frame_time = parent_self._last_frame_save_time
                                    frame_count = parent_self._frame_count
                                
                                uptime = time.time() - parent_self._http_server_start_time
                                age = time.time() - last_frame_time if last_frame_time > 0 else -1
                                
                                import json
                                response = {
                                    "status": "ok",
                                    "uptime_sec": round(uptime, 2),
                                    "last_frame_time": last_frame_time,
                                    "frame_age_sec": round(age, 2) if age >= 0 else None,
                                    "frame_count": frame_count
                                }
                                
                                self.send_response(200)
                                self.send_header('Content-Type', 'application/json')
                                self.send_header('Access-Control-Allow-Origin', '*')
                                self.send_header('Cache-Control', 'no-cache, no-store')
                                self.end_headers()
                                self.wfile.write(json.dumps(response).encode('utf-8'))
                            except Exception as e:
                                parent_self._node.get_logger().error(f'[视觉] /health 处理失败: {e}')
                                self.send_error(500, str(e))
                            return

                        if self.path.startswith('/stream.mjpg'):
                            self.send_response(200)
                            self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                            self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                            self.send_header('Pragma', 'no-cache')
                            self.send_header('Expires', '0')
                            self.send_header('Access-Control-Allow-Origin', '*')
                            self.end_headers()
                            last_sent = 0.0
                            while True:
                                try:
                                    frame = None
                                    with parent_self._lock:
                                        if parent_self._combined_frame is not None:
                                            frame = parent_self._combined_frame.copy()
                                    if frame is not None:
                                        ok, encoded = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                                        if ok:
                                            content = encoded.tobytes()
                                            self.wfile.write(b'--frame\r\n')
                                            self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                            self.wfile.write(f'Content-Length: {len(content)}\r\n\r\n'.encode('ascii'))
                                            self.wfile.write(content)
                                            self.wfile.write(b'\r\n')
                                            last_sent = time.time()
                                    elif time.time() - last_sent > 0.5:
                                        with open(parent_self._jpeg_output_path, 'rb') as f:
                                            content = f.read()
                                        self.wfile.write(b'--frame\r\n')
                                        self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                        self.wfile.write(f'Content-Length: {len(content)}\r\n\r\n'.encode('ascii'))
                                        self.wfile.write(content)
                                        self.wfile.write(b'\r\n')
                                        last_sent = time.time()
                                    time.sleep(parent_self._min_frame_interval)
                                except (BrokenPipeError, ConnectionResetError):
                                    break
                                except FileNotFoundError:
                                    time.sleep(0.2)
                                except Exception as e:
                                    parent_self._node.get_logger().error(f'[视觉] MJPEG 流失败: {e}')
                                    break
                            return
                        
                        # 图像请求：固定返回 /tmp/vision_latest.jpg
                        if self.path.startswith('/image') or self.path.startswith('/vision_latest.jpg'):
                            try:
                                with open(parent_self._jpeg_output_path, 'rb') as f:
                                    content = f.read()
                                self.send_response(200)
                                self.send_header('Content-Type', 'image/jpeg')
                                self.send_header('Content-Length', len(content))
                                self.send_header('Access-Control-Allow-Origin', '*')
                                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                                self.send_header('Pragma', 'no-cache')
                                self.send_header('Expires', '0')
                                self.end_headers()
                                self.wfile.write(content)
                            except FileNotFoundError:
                                self.send_error(404, 'Image not found')
                            except Exception as e:
                                parent_self._node.get_logger().error(f'[视觉] 图像服务失败: {e}')
                                self.send_error(500, str(e))
                            return
                        
                        # 其他请求 404
                        self.send_error(404)

                    def log_message(self, format, *args):
                        pass  # 禁止打印访问日志（避免刷屏）

                    def handle(self):
                        """重写 handle，捕获 BrokenPipeError"""
                        try:
                            super().handle()
                        except (BrokenPipeError, ConnectionResetError):
                            pass  # 浏览器取消请求，正常现象

                parent_self._node.get_logger().info(f'[视觉] 尝试绑定 0.0.0.0:{parent_self.http_port}')
                
                # 设置 SO_REUSEADDR（必须在创建前设置）
                ThreadingTCPServer.allow_reuse_address = True
                
                httpd = ThreadingTCPServer(("0.0.0.0", parent_self.http_port), NoCacheHandler)
                httpd.daemon_threads = True
                
                parent_self._node.get_logger().info(f'[视觉] ✓ HTTP 服务已创建，准备进入 serve_forever()')
                httpd.serve_forever()
                parent_self._node.get_logger().warn('[视觉] serve_forever() 退出（不应该发生）')
            except OSError as e:
                parent_self._node.get_logger().error(f'[视觉] ✗ HTTP 端口绑定失败（端口可能被占用）: {e}')
            except Exception as e:
                parent_self._node.get_logger().error(f'[视觉] ✗ HTTP 服务失败: {e}')
                import traceback
                parent_self._node.get_logger().error(traceback.format_exc())

        http_thread = threading.Thread(target=serve, daemon=True, name='VisionHTTPServer')
        http_thread.start()
        parent_self._node.get_logger().info(f'[视觉] HTTP 线程已启动（thread={http_thread.name}）')

    def _write_placeholder_image(self):
        """预生成占位图像，避免浏览器首次连接时 404"""
        try:
            placeholder = np.full((360, 640, 3), 30, dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for camera data...', (50, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(placeholder, f'http://0.0.0.0:{self.http_port}', (50, 220),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            cv2.imwrite(self._jpeg_output_path, placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self._node.get_logger().info('[视觉] 占位图像已写入')
        except Exception as e:
            self._node.get_logger().warn(f'[视觉] 写入占位图像失败: {e}')

    # ═══════════════════════════════════════════════════════
    # 内部推理逻辑
    # ═══════════════════════════════════════════════════════

    def _image_callback(self, msg):
        """ROS 图像回调 → 推理 → 更新 offset"""
        try:
            if not self.is_inference_active():
                return

            if self._frame_count == 0:
                self._node.get_logger().info('[视觉] 收到第一帧相机数据（phase2 推理已启用）')
            
            # 1. ROI：下方 crop_ratio + 左右各裁 crop_side_ratio（默认底40% + 中60%）
            img_full = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h_full, w_full = img_full.shape[:2]
            crop_ratio = float(np.clip(self.crop_ratio, 0.1, 1.0))
            side_ratio = float(np.clip(self.crop_side_ratio, 0.0, 0.45))
            crop_start_row = int(h_full * (1.0 - crop_ratio))
            crop_start_row = max(0, min(h_full - 1, crop_start_row))
            side_px = int(w_full * side_ratio)
            x0 = max(0, side_px)
            x1 = min(w_full, w_full - side_px)
            if x1 <= x0 + 8:
                x0, x1 = 0, w_full
            img_cropped = img_full[crop_start_row:, x0:x1].copy()
            h_crop, w_crop = img_cropped.shape[:2]

            frame_before = img_cropped.copy()
            keep_w = max(1, 100 - int(round(side_ratio * 200)))
            cv2.putText(
                frame_before,
                f'INPUT (Bottom {int(round(crop_ratio * 100))}% | Mid {keep_w}%W)',
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
            )
            
            # 2. 预处理
            canvas, scale = self._letterbox(img_cropped, self.input_size)
            nv12 = self._bgr2nv12(canvas)
            
            # 3. 推理
            t0 = time.perf_counter()
            outs = self.model.forward([nv12])
            self._infer_time_ms = (time.perf_counter() - t0) * 1000
            
            # 4. 后处理
            all_bboxes, all_scores, all_masks_coeff = [], [], []
            for si, s in enumerate(self.strides):
                reg = outs[si*3].buffer.reshape(-1, 64)
                cls = self._sigmoid(outs[si*3+1].buffer.reshape(-1, 1))
                coeff = outs[si*3+2].buffer.reshape(-1, 32)
                hg, wg = outs[si*3].buffer.shape[1:3]
                sx, sy = np.meshgrid(np.arange(wg)+0.5, np.arange(hg)+0.5, indexing='xy')
                ap = np.stack((sx.ravel(), sy.ravel()), axis=1).astype(np.float32)
                box = self._dfl_decode(reg) * s
                x1 = ap[:, 0] * s - box[:, 0]
                y1 = ap[:, 1] * s - box[:, 1]
                x2 = ap[:, 0] * s + box[:, 2]
                y2 = ap[:, 1] * s + box[:, 3]
                bboxes = np.column_stack([x1, y1, x2, y2])
                all_bboxes.append(bboxes)
                all_scores.append(cls[:, 0])
                all_masks_coeff.append(coeff)
            
            bboxes = np.concatenate(all_bboxes)
            scores = np.concatenate(all_scores)
            masks_coeff = np.concatenate(all_masks_coeff)
            
            keep = scores > self.conf_thres
            bboxes = bboxes[keep]
            scores = scores[keep]
            masks_coeff = masks_coeff[keep]
            
            # 5. 计算 offset / 剩余距离（取第一个检测）
            frame_after = img_cropped.copy()
            best_offset = 0.0
            best_box = None
            best_score = 0.0
            best_mask = None
            best_remaining_m = None
            best_free_ratio = 0.0
            best_center_ratio = 0.0
            best_centered = False
            best_curve = 0.0
            best_confidence = 0.0
            best_valid_rows = 0
            best_boundary_safe = True
            best_safety_weight = 1.0
            best_path_samples_m = []
            best_lookahead_point_m = None
            best_lateral_error_m = 0.0
            boundary_ahead = False
            boundary_dist = 0.0
            boundary_diag = {
                'boundary_ahead': False,
                'distance_ratio': 0.0,
                'far_ratio': 0.0,
                'mid_ratio': 0.0,
                'near_ratio': 0.0,
                'coverage_std': 0.0,
                'top_y_ratio': 0.0,
            }
            detection_valid = False
            
            if len(bboxes) > 0:
                xywh = np.zeros_like(bboxes)
                xywh[:, 0] = bboxes[:, 0]
                xywh[:, 1] = bboxes[:, 1]
                xywh[:, 2] = bboxes[:, 2] - bboxes[:, 0]
                xywh[:, 3] = bboxes[:, 3] - bboxes[:, 1]
                idxs = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), 
                                       self.conf_thres, self.iou_thres)
                if isinstance(idxs, np.ndarray):
                    idxs = idxs.flatten()
                elif isinstance(idxs, (list, tuple)):
                    idxs = np.array(idxs).flatten()
                else:
                    idxs = np.array([], dtype=int)
                
                proto = outs[9].buffer.squeeze()
                proto_h, proto_w = proto.shape[0], proto.shape[1]
                protos_2d = proto.reshape(proto_h*proto_w, -1).T
                mask_scale = proto_h / self.input_size
                
                self._det_count = len(idxs)
                
                for i in idxs:
                    box = bboxes[i]
                    x1 = max(0, min(w_crop-1, box[0]/scale))
                    y1 = max(0, min(h_crop-1, box[1]/scale))
                    x2 = max(0, min(w_crop-1, box[2]/scale))
                    y2 = max(0, min(h_crop-1, box[3]/scale))
                    
                    mc = masks_coeff[i]
                    mask_raw = mc @ protos_2d
                    mask_sig = self._sigmoid(mask_raw.reshape(proto_h, proto_w))
                    bx1 = int(max(0, box[0]*mask_scale))
                    by1 = int(max(0, box[1]*mask_scale))
                    bx2 = int(min(proto_w, box[2]*mask_scale))
                    by2 = int(min(proto_h, box[3]*mask_scale))
                    mcrop = mask_sig[by1:by2, bx1:bx2]
                    
                    if mcrop.size > 0:
                        bw = int(x2 - x1)
                        bh = int(y2 - y1)
                        if bw > 0 and bh > 0:
                            mr = cv2.resize(mcrop, (bw, bh))
                            mb = (mr > 0.5).astype(np.uint8)
                            mf = np.zeros((h_crop, w_crop), dtype=np.uint8)
                            y0i = int(y1)
                            x0i = int(x1)
                            y1i = min(h_crop, y0i + bh)
                            x1i = min(w_crop, x0i + bw)
                            mf[y0i:y1i, x0i:x1i] = mb[:y1i - y0i, :x1i - x0i]
                            cm = np.zeros_like(frame_after)
                            cm[mf > 0] = (0, 255, 0)
                            frame_after = cv2.addWeighted(frame_after, 1.0, cm, 0.4, 0)
                            ct, _ = cv2.findContours(mf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            cv2.drawContours(frame_after, ct, -1, (0, 255, 0), 2)
                        else:
                            mf = None
                    else:
                        mf = None
                    
                    cv2.rectangle(frame_after, (int(x1), int(y1)), (int(x2), int(y2)),
                                 (0, 255, 0), 2)

                    # 中线跟随主路径；质心仅作 fallback
                    bbox_cx = (x1 + x2) * 0.5
                    bbox_cy = (y1 + y2) * 0.5
                    curve = 0.0
                    confidence = 0.0
                    valid_rows = 0
                    samples = []
                    target_xy = None
                    path_samples_m = []
                    lookahead_point_m = None
                    lateral_error_m = 0.0
                    mid = w_crop // 2

                    if self._centerline_mode and mf is not None:
                        samples, line_err, curve, confidence, target_xy, path_samples_m, lookahead_point_m, lateral_error_m = self._extract_centerline_from_mask(mf)
                        valid_rows = len(samples)
                        if line_err is not None:
                            if self._has_filtered_error:
                                alpha = float(self._offset_filter_alpha)
                                self._filtered_error = (
                                    (1.0 - alpha) * self._filtered_error + alpha * float(line_err)
                                )
                            else:
                                self._filtered_error = float(line_err)
                                self._has_filtered_error = True
                            offset = float(np.clip(self._filtered_error, -1.0, 1.0))
                            center_ratio = max(0.0, 1.0 - abs(offset))
                            _lc = bool(getattr(self, '_lane_clear', False))
                            _coff = float(getattr(self, '_lane_center_off', 0.0) or 0.0)
                            centered = (
                                (abs(offset) < 0.10 and abs(curve) < 0.25)
                                or (_lc and abs(_coff) < 0.18)
                                or (abs(_coff) < 0.12 and abs(curve) < 0.30)
                            )
                            if target_xy is not None:
                                cx_f, cy_f = float(target_xy[0]), float(target_xy[1])
                            else:
                                cx_f, cy_f = float(bbox_cx), float(bbox_cy)
                        else:
                            # fallback centroid still needs non-zero confidence or fusion rejects it
                            offset, center_ratio, centered, cx_f, cy_f = self._estimate_offset_from_mask(
                                mf, fallback_cx=bbox_cx, fallback_cy=bbox_cy
                            )
                            if offset is None:
                                cx_f = float(bbox_cx)
                                cy_f = float(bbox_cy)
                                offset = cx_f / (w_crop / 2.0) - 1.0
                                offset = float(np.clip(offset, -1.0, 1.0))
                                center_ratio = 0.0
                                centered = False
                            # weak confidence: det score * row coverage
                            confidence = float(np.clip(0.35 * float(scores[i]) + 0.15 * min(1.0, valid_rows / max(self._sample_rows, 1)), 0.0, 0.75))
                            curve = 0.0
                    else:
                        offset, center_ratio, centered, cx_f, cy_f = self._estimate_offset_from_mask(
                            mf, fallback_cx=bbox_cx, fallback_cy=bbox_cy
                        )
                        if offset is None:
                            cx_f = float(bbox_cx)
                            cy_f = float(bbox_cy)
                            offset = cx_f / (w_crop / 2.0) - 1.0
                            offset = float(np.clip(offset, -1.0, 1.0))
                            center_ratio = 0.0
                            centered = False
                        confidence = float(np.clip(0.45 * float(scores[i]), 0.0, 0.80))
                        curve = 0.0
                        valid_rows = max(valid_rows, 1 if offset is not None else 0)

                    cx = int(round(cx_f if cx_f is not None else bbox_cx))
                    cy = int(round(cy_f if cy_f is not None else bbox_cy))

                    # ═══ 边界 + 引导线可视化增强 ═══
                    half_band = max(1, int(round(w_crop * float(self._offset_center_band) * 0.5)))
                    band_color = (0, 255, 0) if centered else (0, 165, 255)
                    cv2.rectangle(
                        frame_after,
                        (max(0, mid - half_band), 0),
                        (min(w_crop - 1, mid + half_band), h_crop - 1),
                        band_color, 1,
                    )
                    cv2.line(frame_after, (mid, 0), (mid, h_crop - 1), (255, 255, 255), 1)

                    # 绘制左右边界线（红色=左边界，蓝色=右边界）
                    if hasattr(self, '_latest_left_boundary') and hasattr(self, '_latest_right_boundary'):
                        left_pts = getattr(self, '_latest_left_boundary', [])
                        right_pts = getattr(self, '_latest_right_boundary', [])

                        # 左边界（红色粗线）
                        if len(left_pts) >= 2:
                            for a, b in zip(left_pts[:-1], left_pts[1:]):
                                cv2.line(frame_after,
                                        (int(a[0]), int(a[1])),
                                        (int(b[0]), int(b[1])),
                                        (0, 0, 255), 3)  # 红色
                            # 标注左边界点
                            for lx, ly in left_pts:
                                cv2.circle(frame_after, (int(lx), int(ly)), 2, (0, 0, 255), -1)

                        # 右边界（蓝色粗线）
                        if len(right_pts) >= 2:
                            for a, b in zip(right_pts[:-1], right_pts[1:]):
                                cv2.line(frame_after,
                                        (int(a[0]), int(a[1])),
                                        (int(b[0]), int(b[1])),
                                        (255, 0, 0), 3)  # 蓝色
                            # 标注右边界点
                            for rx, ry in right_pts:
                                cv2.circle(frame_after, (int(rx), int(ry)), 2, (255, 0, 0), -1)

                    # 引导中线（黄色粗线 + 采样点）
                    if samples:
                        # 绘制引导中线（黄色粗线，更醒目）
                        for a, b in zip(samples[:-1], samples[1:]):
                            cv2.line(
                                frame_after,
                                (int(a[0]), int(a[1])),
                                (int(b[0]), int(b[1])),
                                (0, 255, 255), 4,  # 加粗到4px
                            )
                        # 标注中线采样点（黄色圆点）
                        for sx, sy in samples:
                            cv2.circle(frame_after, (int(sx), int(sy)), 4, (0, 255, 255), -1)

                        # 前瞻目标点（洋红色大圆）
                        if target_xy is not None:
                            cv2.circle(frame_after, (int(target_xy[0]), int(target_xy[1])), 8, (255, 0, 255), -1)
                            cv2.circle(frame_after, (int(target_xy[0]), int(target_xy[1])), 12, (255, 0, 255), 2)

                    # 车辆位置指示（绿色圆点 + 指向线）
                    cv2.circle(frame_after, (cx, cy), 6, (0, 255, 0), -1)
                    cv2.line(frame_after, (mid, h_crop - 1), (cx, cy), (0, 255, 0), 2)

                    rem_m, free_ratio = self._estimate_remaining_from_mask(mf)

                    # 边界安全检测
                    boundary_safe, safety_weight = self._check_lane_boundary_safety(mf, offset)

                    rem_txt = f'{rem_m:.2f}m' if rem_m is not None else 'N/A'
                    state_txt = 'CENTERED' if centered else f'e={offset:+.2f} c={curve:+.2f}'
                    safe_txt = f'SAFE' if boundary_safe else f'DANGER(w={safety_weight:.1f})'
                    lateral_txt = f'lat={lateral_error_m:+.3f}m' if lateral_error_m != 0.0 else ''
                    cv2.putText(
                        frame_after,
                        f'conf={scores[i]:.2f} {state_txt} rows={valid_rows} rem={rem_txt} {safe_txt} {lateral_txt}',
                        (int(x1), max(20, int(y1) - 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2,
                    )

                    # 取第一个检测的中线误差 + 纵向剩余 + 边界安全 + 路径信息 + 边界线检测
                    if not detection_valid:
                        best_offset = float(offset)
                        best_box = box
                        best_score = scores[i]
                        best_mask = mf
                        best_remaining_m = rem_m
                        best_free_ratio = free_ratio
                        best_center_ratio = float(center_ratio)
                        best_centered = bool(centered)
                        best_curve = float(curve)
                        best_confidence = float(confidence)
                        best_valid_rows = int(valid_rows)
                        best_boundary_safe = bool(boundary_safe)
                        best_safety_weight = float(safety_weight)
                        best_path_samples_m = path_samples_m
                        best_lookahead_point_m = lookahead_point_m
                        best_lateral_error_m = float(lateral_error_m)

                        # 检测前方转弯边界（返回详细诊断）
                        if mf is not None and mf.size > 0:
                            binfo = self._detect_boundary_ahead(mf)
                        else:
                            binfo = {
                                'boundary_ahead': False,
                                'distance_ratio': 0.0,
                                'far_ratio': 0.0,
                                'mid_ratio': 0.0,
                                'near_ratio': 0.0,
                                'coverage_std': 0.0,
                                'top_y_ratio': 0.0,
                            }
                        boundary_ahead = bool(binfo.get('boundary_ahead', False))
                        boundary_dist = float(binfo.get('distance_ratio', 0.0))
                        boundary_diag = binfo

                        detection_valid = True
            
            # 6. 更新共享变量 + 状态变化日志
            with self._lock:
                if detection_valid:
                    self._latest_offset = float(best_offset)
                    self._latest_timestamp = time.time()
                    self._valid = True
                    self._latest_center_ratio = float(best_center_ratio)
                    self._latest_centered = bool(best_centered)
                    self._latest_curve = float(best_curve)
                    self._latest_confidence = float(best_confidence)
                    self._latest_valid_rows = int(best_valid_rows)
                    self._latest_boundary_safe = bool(best_boundary_safe)
                    self._latest_safety_weight = float(best_safety_weight)
                    # Pure Pursuit 路径信息
                    self._latest_path_samples = list(best_path_samples_m or [])
                    self._latest_lookahead_point = best_lookahead_point_m
                    self._latest_lateral_error_m = float(best_lateral_error_m or 0.0)
                    # 边界线检测
                    self._boundary_ahead = bool(boundary_ahead)
                    self._boundary_distance_ratio = float(boundary_dist)
                    diag = boundary_diag if isinstance(boundary_diag, dict) else {}
                    self._boundary_far_ratio = float(diag.get('far_ratio', 0.0))
                    self._boundary_mid_ratio = float(diag.get('mid_ratio', 0.0))
                    self._boundary_near_ratio = float(diag.get('near_ratio', 0.0))
                    self._boundary_coverage_std = float(diag.get('coverage_std', 0.0))
                    self._boundary_top_y_ratio = float(diag.get('top_y_ratio', 0.0))
                    self._front_score = float(diag.get('front_score', 0.0))
                    self._straight_score = float(diag.get('straight_score', 0.0))
                    self._left_ratio = float(diag.get('left_ratio', 0.0))
                    self._right_ratio = float(diag.get('right_ratio', 0.0))
                    self._edge_angle_deg = float(diag.get('edge_angle_deg', 90.0))
                    self._perp_score = float(diag.get('perp_score', 0.0))
                    if best_remaining_m is not None:
                        self._latest_remaining_m = float(best_remaining_m)
                        self._latest_free_ratio = float(best_free_ratio)
                        self._range_valid = True
                    else:
                        self._range_valid = False

                    # 状态变化日志：从失效→恢复
                    if self._last_valid_state is False:
                        rem_txt = (
                            f'{best_remaining_m:.2f}m'
                            if best_remaining_m is not None else 'N/A'
                        )
                        safe_status = 'SAFE' if best_boundary_safe else f'WARN(w={best_safety_weight:.2f})'
                        if best_box is not None:
                            self._node.get_logger().info(
                                f'[VISION] 推理成功 e={best_offset:+.3f} curve={best_curve:+.3f} rows={best_valid_rows} rem={rem_txt} {safe_status} | '
                                f'bbox=({int(best_box[0])},{int(best_box[1])})→'
                                f'({int(best_box[2])},{int(best_box[3])}) '
                                f'conf={best_score:.2f}'
                            )
                        else:
                            self._node.get_logger().info(
                                f'[VISION] 推理成功 e={best_offset:+.3f} curve={best_curve:+.3f} rows={best_valid_rows} rem={rem_txt} {safe_status}'
                            )
                    self._last_valid_state = True
                else:
                    self._valid = False  # 本帧无检测
                    self._range_valid = False
                    self._latest_center_ratio = 0.0
                    self._latest_centered = False
                    self._latest_curve = 0.0
                    self._latest_confidence = 0.0
                    self._latest_valid_rows = 0
                    self._latest_boundary_safe = True
                    self._latest_safety_weight = 1.0
                    self._latest_path_samples = []
                    self._latest_lookahead_point = None
                    self._latest_lateral_error_m = 0.0
                    self._boundary_ahead = False
                    self._boundary_distance_ratio = 0.0
                    self._boundary_far_ratio = 0.0
                    self._boundary_mid_ratio = 0.0
                    self._boundary_near_ratio = 0.0
                    self._boundary_coverage_std = 0.0
                    self._boundary_top_y_ratio = 0.0
                    self._front_score = 0.0
                    self._straight_score = 0.0
                    self._left_ratio = 0.0
                    self._right_ratio = 0.0
                    self._near_error = 0.0
                    self._far_error = 0.0
                    self._path_bend = 0.0
                    self._left_margin = 0.0
                    self._right_margin = 0.0
                    self._lane_clear = False
                    self._lane_center_off = 0.0
                    self._rel_left = 0.0
                    self._rel_right = 0.0
                    self._center_fill = 0.0
                    self._center_fill_5 = 0.0
                    self._apex_has_mask = False
                    self._apex_error = 0.0
                    self._apex_fill = 0.0
                    self._edge_angle_deg = 90.0
                    self._perp_score = 0.0
                    self._has_filtered_error = False
                    
                    # 状态变化日志：从有效→失效
                    if self._last_valid_state is True:
                        det_count = len(bboxes) if 'bboxes' in locals() else 0
                        self._node.get_logger().warn(
                            f'[VISION] 推理失败：无有效检测 | '
                            f'检测数={det_count} 过滤后=0'
                        )
                    self._last_valid_state = False
            
            # 7. FPS 统计
            now = time.perf_counter()
            self._fps_queue.append(now - self._last_time)
            self._last_time = now
            avg_fps = len(self._fps_queue) / sum(self._fps_queue) if self._fps_queue else 0
            
            cv2.putText(frame_after, f'OUTPUT (FPS: {avg_fps:.1f} | Infer: {self._infer_time_ms:.1f}ms)',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame_after, f'Detections: {self._det_count}',
                       (10, h_crop-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            # 图例（右上角）
            legend_x = w_crop - 280
            legend_y = 20
            line_h = 25
            cv2.putText(frame_after, 'Legend:', (legend_x, legend_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.line(frame_after, (legend_x, legend_y+10), (legend_x+50, legend_y+10), (0, 0, 255), 3)
            cv2.putText(frame_after, 'Left Bound', (legend_x+60, legend_y+15),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.line(frame_after, (legend_x, legend_y+10+line_h), (legend_x+50, legend_y+10+line_h), (255, 0, 0), 3)
            cv2.putText(frame_after, 'Right Bound', (legend_x+60, legend_y+15+line_h),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.line(frame_after, (legend_x, legend_y+10+line_h*2), (legend_x+50, legend_y+10+line_h*2), (0, 255, 255), 4)
            cv2.putText(frame_after, 'Guide Line', (legend_x+60, legend_y+15+line_h*2),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            cv2.circle(frame_after, (legend_x+25, legend_y+10+line_h*3), 8, (255, 0, 255), -1)
            cv2.putText(frame_after, 'Target Pt', (legend_x+60, legend_y+15+line_h*3),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
            
            # 8. 拼接可视化（降低分辨率加速传输）
            target_h = 360  # 固定高度 360p（从原始可能的 480+ 降低）
            before_resized = cv2.resize(frame_before, 
                                       (int(frame_before.shape[1] * target_h / frame_before.shape[0]), target_h))
            after_resized = cv2.resize(frame_after, 
                                      (int(frame_after.shape[1] * target_h / frame_after.shape[0]), target_h))
            combined = np.hstack([before_resized, after_resized])
            
            mid_x = before_resized.shape[1]
            cv2.line(combined, (mid_x, 0), (mid_x, target_h), (255, 255, 255), 3)
            
            # 更新缓存
            with self._lock:
                self._combined_frame = combined.copy()
                self._frame_count += 1
            
            # 9. 保存到文件（限制 10 FPS，原子写入避免跳变）
            now = time.perf_counter()
            if now - self._last_save_time >= self._min_frame_interval:
                try:
                    # 原子写入：先写临时文件（.jpg 后缀），再重命名（避免读写冲突）
                    temp_path = '/tmp/vision_latest_tmp.jpg'
                    cv2.imwrite(temp_path, combined, [cv2.IMWRITE_JPEG_QUALITY, 85])
                    os.replace(temp_path, self._jpeg_output_path)  # 原子操作
                    self._last_save_time = now
                    
                    # 记录最后一帧时间（供健康检查使用）
                    with self._lock:
                        self._last_frame_save_time = time.time()
                    
                    # 帧保存仅写文件，不再刷终端诊断日志
                except Exception as save_err:
                    self._node.get_logger().error(f'[视觉] 保存图像失败: {save_err}')
            
        except Exception as e:
            self._node.get_logger().error(f'[视觉-诊断] 推理失败: {e}')
    
    # ═══════════════════════════════════════════════════════
    # 工具函数
    # ═══════════════════════════════════════════════════════
    
    def _sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))
    
    def _dfl_decode(self, reg):
        N = reg.shape[0]
        proj = np.arange(16, dtype=np.float32)
        reg = reg.reshape(N, 4, 16)
        sm = np.exp(reg - reg.max(axis=-1, keepdims=True))
        sm /= sm.sum(axis=-1, keepdims=True)
        return (sm @ proj).reshape(N, 4)
    
    def _bgr2nv12(self, bgr, w=640, h=640):
        i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).flatten()
        hw = h * w
        hw_uv = hw // 4
        nv12 = np.empty(hw * 3 // 2, dtype=np.uint8)
        nv12[:hw] = i420[:hw]
        uv = np.zeros(hw_uv * 2, dtype=np.uint8)
        uv[0::2] = i420[hw:hw+hw_uv]
        uv[1::2] = i420[hw+hw_uv:hw+hw_uv*2]
        nv12[hw:] = uv
        return nv12
    
    def _letterbox(self, img, target_size=640):
        h0, w0 = img.shape[:2]
        scale = min(target_size / h0, target_size / w0)
        nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
        img_resized = cv2.resize(img, (nw, nh))
        canvas = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = img_resized
        return canvas, scale


# ═══════════════════════════════════════════════════════
# 工具函数
# ═══════════════════════════════════════════════════════
