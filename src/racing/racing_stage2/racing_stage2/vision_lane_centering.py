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
                            centered = abs(offset) < 0.08 and abs(curve) < 0.20
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

                    # 取第一个检测的中线误差 + 纵向剩余 + 边界安全 + 路径信息
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
