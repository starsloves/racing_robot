#!/usr/bin/env python3
"""
vision_corridor_detector.py — Stage1 通道视觉检测与导航模块

功能：
1. 订阅相机 topic，裁剪 ROI 后 BPU YOLOv8-Seg 推理
2. 提取通道边界，生成中线路径
3. 多行采样前瞻点误差 + 曲率估计
4. 实时保存可视化到 /tmp/stage1_vision.jpg，并提供 HTTP 预览（端口 8081）
5. 纵向视觉定长：通过 mask 顶部位置估计剩余距离

共享接口：
    get_latest_corridor_status() -> dict(
        lateral_error,      # 横向误差 [-1, +1]
        heading_error_deg,  # 航向误差（度）
        curvature,          # 曲率估计
        remaining_m,        # 剩余距离（米）
        confidence,         # 置信度
        valid,              # 数据有效性
        boundary_safe,      # 边界安全标志
        timestamp           # 时间戳
    )
"""

import threading
import time
import os
import math
from collections import deque
from http.server import SimpleHTTPRequestHandler
from socketserver import ThreadingTCPServer

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hobot_dnn import pyeasy_dnn as dnn


class VisionCorridorDetector:
    """
    Stage1 通道视觉检测与导航模块
    """

    def __init__(self, parent_node, model_path, conf_thres=0.25, iou_thres=0.45,
                 crop_ratio=0.4, http_port=8081, crop_side_ratio=0.20):
        self._node = parent_node
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.crop_ratio = crop_ratio
        self.crop_side_ratio = float(np.clip(crop_side_ratio, 0.0, 0.45))
        self.http_port = http_port

        # 共享变量（线程安全）
        self._lock = threading.Lock()
        self._inference_active = False
        self._latest_lateral_error = 0.0
        self._latest_heading_error_deg = 0.0
        self._latest_curvature = 0.0
        self._latest_remaining_m = None
        self._latest_confidence = 0.0
        self._latest_timestamp = 0.0
        self._valid = False
        self._boundary_safe = True
        self._detection_timeout_sec = 0.5

        # 中线提取参数
        self._sample_rows = 9
        self._lookahead_ratio = 0.62
        self._min_mask_width_px = 50
        self._min_valid_rows = 5
        self._mask_threshold = 0.50
        self._error_filter_alpha = 0.35
        self._filtered_error = 0.0
        self._has_filtered_error = False

        # 纵向距离估计参数
        self._range_near_m = 0.15
        self._range_far_m = 2.50
        self._entry_detect_ratio = 0.15  # mask顶部<15%判定到达入口

        # 边界安全检测
        self._boundary_margin = 0.15
        self._boundary_coverage_thresh = 0.20

        # 可视化缓存
        self._combined_frame = None
        self._jpeg_output_path = '/tmp/stage1_vision.jpg'
        self._http_server_start_time = time.time()
        self._last_frame_save_time = 0.0

        # FPS 控制
        self._target_fps = 30
        self._min_frame_interval = 1.0 / self._target_fps
        self._last_save_time = 0.0

        # 加载模型
        self._node.get_logger().info(f'[Stage1视觉] 加载模型: {model_path}')
        models = dnn.load(model_path)
        self.model = models[0]
        self.input_size = 640
        self.REG_MAX = 16
        self.strides = [8, 16, 32]

        # ROS 订阅（延迟创建，避免与 Stage2 冲突）
        self.bridge = CvBridge()
        self._camera_subscription = None
        self._camera_topic = '/aurora/rgb/image_raw'
        self._camera_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # 不在初始化时订阅，而是在 set_inference_active(True) 时订阅
        self._node.get_logger().info('[Stage1视觉] 延迟相机订阅（避免与Stage2冲突）')

        # 统计
        self._frame_count = 0
        self._det_count = 0
        self._first_frame_ready = False
        self._fps_queue = deque(maxlen=30)
        self._last_time = time.perf_counter()
        self._infer_time_ms = 0.0

        # 创建占位图像
        self._write_placeholder_image()

        # 启动 HTTP 服务器
        self._start_http_server()

        self._node.get_logger().info('[Stage1视觉] 模块初始化完成，等待相机数据...')
        self._node.get_logger().info(
            f'[Stage1视觉] HTTP 服务已启动: http://0.0.0.0:{self.http_port}/vision_latest.jpg'
        )

        # 测试：立即执行一次空推理，检查 BPU 是否正常
        try:
            test_img = np.zeros((480, 640, 3), dtype=np.uint8)
            test_input, _, _, _ = self._preprocess(test_img)
            self._node.get_logger().info('[Stage1视觉] BPU 预处理测试通过')
        except Exception as e:
            self._node.get_logger().error(f'[Stage1视觉] BPU 测试失败: {e}')
            import traceback
            self._node.get_logger().error(traceback.format_exc())

    # ═══════════════════════════════════════════════════════
    # 外部接口（供导航节点调用）
    # ═══════════════════════════════════════════════════════

    def set_inference_active(self, active: bool):
        """启用/停用视觉推理"""
        active = bool(active)
        with self._lock:
            if active != self._inference_active:
                self._inference_active = active
                status = "启用" if active else "禁用"
                self._node.get_logger().info(f'[Stage1视觉] 推理状态: {status}')

                # 启用时创建相机订阅，禁用时销毁订阅
                if active and self._camera_subscription is None:
                    from sensor_msgs.msg import Image
                    self._camera_subscription = self._node.create_subscription(
                        Image, self._camera_topic, self._image_callback, self._camera_qos
                    )
                    self._node.get_logger().info(f'[Stage1视觉] 已订阅相机话题: {self._camera_topic}')
                elif not active and self._camera_subscription is not None:
                    self._node.destroy_subscription(self._camera_subscription)
                    self._camera_subscription = None
                    self._node.get_logger().info('[Stage1视觉] 已取消相机订阅')

    def get_latest_corridor_status(self):
        """
        获取最新通道检测状态

        Returns:
            dict: {
                'lateral_error': float,        # 横向误差 [-1, +1]
                'heading_error_deg': float,    # 航向误差（度）
                'curvature': float,            # 曲率估计
                'remaining_m': float or None,  # 剩余距离（米）
                'confidence': float,           # 置信度 [0, 1]
                'valid': bool,                 # 数据有效性
                'boundary_safe': bool,         # 边界安全标志
                'timestamp': float             # 时间戳（秒）
            }
        """
        with self._lock:
            now = time.time()
            age = now - self._latest_timestamp if self._latest_timestamp > 0 else 999.0
            valid = self._valid and age < self._detection_timeout_sec

            return {
                'lateral_error': self._latest_lateral_error,
                'heading_error_deg': self._latest_heading_error_deg,
                'curvature': self._latest_curvature,
                'remaining_m': self._latest_remaining_m,
                'confidence': self._latest_confidence,
                'valid': valid,
                'boundary_safe': self._boundary_safe,
                'timestamp': self._latest_timestamp,
            }

    def update_params(self, **kwargs):
        """动态更新参数"""
        with self._lock:
            for key, value in kwargs.items():
                if hasattr(self, f'_{key}'):
                    setattr(self, f'_{key}', value)
                    self._node.get_logger().info(f'[Stage1视觉] 参数更新: {key}={value}')

    # ═══════════════════════════════════════════════════════
    # HTTP 健康检查服务
    # ═══════════════════════════════════════════════════════

    def _write_placeholder_image(self):
        """创建占位图像"""
        placeholder = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'Stage1 Vision', (150, 240),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 255, 0), 2)
        cv2.putText(placeholder, 'Waiting for camera...', (180, 300),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (128, 128, 128), 1)
        cv2.imwrite(self._jpeg_output_path, placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])

    def _start_http_server(self):
        """启动 HTTP 静态服务器"""
        server_dir = os.path.dirname(self._jpeg_output_path)

        class HealthHandler(SimpleHTTPRequestHandler):
            parent_detector = self

            def __init__(self, *args, **kwargs):
                super().__init__(*args, directory=server_dir, **kwargs)

            def do_GET(self):
                if self.path.startswith('/health'):
                    self.send_health_response()
                elif self.path.startswith('/vision_latest.jpg'):
                    self.serve_latest_image()
                else:
                    super().do_GET()

            def send_health_response(self):
                """返回健康检查 JSON"""
                now = time.time()
                with self.parent_detector._lock:
                    frame_age = (now - self.parent_detector._latest_timestamp
                                 if self.parent_detector._latest_timestamp > 0 else None)
                    uptime = now - self.parent_detector._http_server_start_time
                    health_data = {
                        'status': 'ok',
                        'stage': 'stage1',
                        'frame_count': self.parent_detector._frame_count,
                        'frame_age_sec': frame_age,
                        'uptime_sec': uptime,
                        'inference_active': self.parent_detector._inference_active,
                        'valid': self.parent_detector._valid,
                    }

                body = str(health_data).replace("'", '"')
                self.send_response(200)
                self.send_header('Content-Type', 'application/json')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(body.encode())

            def serve_latest_image(self):
                """提供最新图像"""
                if os.path.exists(self.parent_detector._jpeg_output_path):
                    super().do_GET()
                else:
                    self.send_error(404, 'Image not found')

            def log_message(self, format, *args):
                pass  # 禁用默认日志

        def serve_forever_thread():
            with ThreadingTCPServer(('0.0.0.0', self.http_port), HealthHandler) as httpd:
                httpd.serve_forever()

        thread = threading.Thread(target=serve_forever_thread, daemon=True)
        thread.start()

    # ═══════════════════════════════════════════════════════
    # YOLOv8-Seg 推理核心
    # ═══════════════════════════════════════════════════════

    def _preprocess(self, img):
        """预处理：Resize + Pad + Normalize"""
        h0, w0 = img.shape[:2]
        r = self.input_size / max(h0, w0)
        if r != 1:
            img = cv2.resize(img, (int(w0 * r), int(h0 * r)), interpolation=cv2.INTER_LINEAR)

        h, w = img.shape[:2]
        pad_h = (self.input_size - h) // 2
        pad_w = (self.input_size - w) // 2
        img = cv2.copyMakeBorder(img, pad_h, self.input_size - h - pad_h,
                                  pad_w, self.input_size - w - pad_w,
                                  cv2.BORDER_CONSTANT, value=(114, 114, 114))

        img = img.astype(np.float32) / 255.0
        img = img.transpose(2, 0, 1)  # HWC -> CHW
        img = np.ascontiguousarray(img)
        return img, r, pad_w, pad_h

    def _postprocess(self, outputs, img_shape, ratio, pad_w, pad_h):
        """后处理：解码检测框 + NMS + Mask 生成"""
        # outputs: (1, 116, 8400) detection + (1, 32, 160, 160) proto
        pred = outputs[0][0]  # (116, 8400)
        proto = outputs[1][0]  # (32, 160, 160)

        # 提取 box + cls + mask_coef
        boxes = pred[:4, :]  # (4, 8400)
        scores = pred[4:5, :]  # (1, 8400)
        mask_coef = pred[5:, :]  # (111, 8400)

        # 过滤低置信度
        mask = scores[0] > self.conf_thres
        if not np.any(mask):
            return None, None, 0.0

        boxes = boxes[:, mask]
        scores = scores[:, mask]
        mask_coef = mask_coef[:, mask]

        # NMS
        indices = self._nms(boxes, scores[0], self.iou_thres)
        if len(indices) == 0:
            return None, None, 0.0

        boxes = boxes[:, indices]
        scores = scores[:, indices]
        mask_coef = mask_coef[:, indices]

        # 生成 mask（取第一个检测）
        mask_coef = mask_coef[:, 0:1]  # (111, 1)
        mask = self._process_mask(proto, mask_coef, boxes[:, 0], img_shape)

        best_conf = float(scores[0, 0])
        return mask, boxes[:, 0], best_conf

    def _nms(self, boxes, scores, iou_threshold):
        """NMS 实现"""
        x1 = boxes[0] - boxes[2] / 2
        y1 = boxes[1] - boxes[3] / 2
        x2 = boxes[0] + boxes[2] / 2
        y2 = boxes[1] + boxes[3] / 2

        areas = (x2 - x1) * (y2 - y1)
        order = scores.argsort()[::-1]

        keep = []
        while order.size > 0:
            i = order[0]
            keep.append(i)

            xx1 = np.maximum(x1[i], x1[order[1:]])
            yy1 = np.maximum(y1[i], y1[order[1:]])
            xx2 = np.minimum(x2[i], x2[order[1:]])
            yy2 = np.minimum(y2[i], y2[order[1:]])

            w = np.maximum(0.0, xx2 - xx1)
            h = np.maximum(0.0, yy2 - yy1)
            inter = w * h

            iou = inter / (areas[i] + areas[order[1:]] - inter + 1e-6)
            inds = np.where(iou <= iou_threshold)[0]
            order = order[inds + 1]

        return keep

    def _process_mask(self, protos, mask_coef, box, img_shape):
        """生成 mask"""
        c, mh, mw = protos.shape
        masks = np.matmul(mask_coef.T, protos.reshape(c, -1))  # (1, mh*mw)
        masks = masks.reshape(1, mh, mw)
        masks = 1 / (1 + np.exp(-masks))  # Sigmoid

        # Crop to box
        x, y, w, h = box
        x1 = int(max(0, (x - w / 2) / 640 * mw))
        y1 = int(max(0, (y - h / 2) / 640 * mh))
        x2 = int(min(mw, (x + w / 2) / 640 * mw))
        y2 = int(min(mh, (y + h / 2) / 640 * mh))

        mask_crop = masks[0, y1:y2, x1:x2]
        mask_full = cv2.resize(mask_crop, img_shape[::-1], interpolation=cv2.INTER_LINEAR)
        return mask_full

    # ═══════════════════════════════════════════════════════
    # 中线提取与路径生成
    # ═══════════════════════════════════════════════════════

    def _extract_centerline(self, mask_binary):
        """
        多行采样提取中线

        Args:
            mask_binary: 二值化 mask (H, W)

        Returns:
            centerline_points: [(x, y), ...] 中线点列表
            valid_rows: int 有效行数
            confidence: float 平均置信度
        """
        H, W = mask_binary.shape
        sample_rows = self._sample_rows
        centerline_points = []
        valid_count = 0

        # 从下往上采样
        for i in range(sample_rows):
            row_idx = int(H - 1 - i * H / sample_rows)
            row_idx = np.clip(row_idx, 0, H - 1)

            row_data = mask_binary[row_idx, :]
            valid_pixels = np.where(row_data > 0)[0]

            if len(valid_pixels) < self._min_mask_width_px:
                centerline_points.append(None)
                continue

            left_edge = valid_pixels[0]
            right_edge = valid_pixels[-1]
            center_x = (left_edge + right_edge) / 2.0
            coverage_ratio = len(valid_pixels) / W

            centerline_points.append((center_x, row_idx, coverage_ratio))
            valid_count += 1

        confidence = valid_count / sample_rows if sample_rows > 0 else 0.0

        return centerline_points, valid_count, confidence

    def _compute_control_errors(self, centerline_points, img_width, img_height):
        """
        计算控制误差

        Args:
            centerline_points: 中线点列表
            img_width: 图像宽度
            img_height: 图像高度

        Returns:
            lateral_error: 横向误差 [-1, +1]
            heading_error_deg: 航向误差（度）
            curvature: 曲率估计
        """
        # 过滤有效点
        valid_points = [p for p in centerline_points if p is not None]
        if len(valid_points) < 2:
            return 0.0, 0.0, 0.0

        # 前瞻点选择
        lookahead_idx = int(len(valid_points) * self._lookahead_ratio)
        lookahead_idx = np.clip(lookahead_idx, 0, len(valid_points) - 1)
        target_point = valid_points[lookahead_idx]

        # 横向误差（归一化）
        target_x = target_point[0]
        center_x = img_width / 2.0
        lateral_error = (target_x - center_x) / (img_width / 2.0)
        lateral_error = np.clip(lateral_error, -1.0, 1.0)

        # 航向误差（最近点到前瞻点的连线角度）
        if len(valid_points) >= 2:
            near_point = valid_points[0]
            dx = target_point[0] - near_point[0]
            dy = near_point[1] - target_point[1]  # Y轴向下
            heading_error_rad = math.atan2(dx, dy + 1e-6)
            heading_error_deg = math.degrees(heading_error_rad)
        else:
            heading_error_deg = 0.0

        # 曲率估计（三点法）
        if len(valid_points) >= 3:
            p1 = valid_points[0]
            p2 = valid_points[len(valid_points) // 2]
            p3 = valid_points[-1]

            a = np.linalg.norm([p2[0] - p1[0], p2[1] - p1[1]])
            b = np.linalg.norm([p3[0] - p2[0], p3[1] - p2[1]])
            c = np.linalg.norm([p3[0] - p1[0], p3[1] - p1[1]])

            s = (a + b + c) / 2.0
            area = math.sqrt(max(0, s * (s - a) * (s - b) * (s - c)))
            if area > 1e-6 and a * b * c > 0:
                curvature = 4.0 * area / (a * b * c)
            else:
                curvature = 0.0
        else:
            curvature = 0.0

        return lateral_error, heading_error_deg, curvature

    def _estimate_remaining_distance(self, mask_binary):
        """
        估计剩余距离（基于 mask 顶部位置）

        Args:
            mask_binary: 二值化 mask (H, W)

        Returns:
            remaining_m: 剩余距离（米），None 表示无法估计
        """
        H = mask_binary.shape[0]

        # 查找 mask 最顶部位置
        col_sums = np.sum(mask_binary, axis=1)
        valid_rows = np.where(col_sums > 0)[0]

        if len(valid_rows) == 0:
            return None

        top_row = valid_rows[0]
        top_ratio = top_row / H

        # 线性映射：top_ratio [0, entry_detect_ratio] -> remaining [range_far, 0]
        if top_ratio < self._entry_detect_ratio:
            # 已接近入口
            remaining_m = 0.0
        else:
            # 线性插值
            remaining_m = self._range_far_m * (1.0 - top_ratio)
            remaining_m = np.clip(remaining_m, self._range_near_m, self._range_far_m)

        return remaining_m

    def _check_boundary_safety(self, mask_binary):
        """
        检查边界安全性

        Args:
            mask_binary: 二值化 mask (H, W)

        Returns:
            is_safe: bool 是否安全
            safety_weight: float 安全权重 [0, 1]
        """
        H, W = mask_binary.shape

        # 检查左右边界区域的覆盖率
        margin_w = int(W * self._boundary_margin)
        left_region = mask_binary[:, :margin_w]
        right_region = mask_binary[:, -margin_w:]

        left_coverage = np.sum(left_region) / (H * margin_w + 1e-6)
        right_coverage = np.sum(right_region) / (H * margin_w + 1e-6)

        # 如果两侧覆盖率都很低，说明通道过窄或偏离
        min_coverage = min(left_coverage, right_coverage)
        is_safe = min_coverage >= self._boundary_coverage_thresh

        # 安全权重
        safety_weight = np.clip(min_coverage / (self._boundary_coverage_thresh + 1e-6), 0.0, 1.0)

        return is_safe, safety_weight

    # ═══════════════════════════════════════════════════════
    # 可视化绘制
    # ═══════════════════════════════════════════════════════

    def _draw_visualization(self, orig_img, mask_binary, centerline_points, status):
        """
        绘制可视化图像

        Args:
            orig_img: 原始图像
            mask_binary: 二值化 mask
            centerline_points: 中线点列表
            status: 状态字典

        Returns:
            vis_img: 可视化图像
        """
        H, W = mask_binary.shape
        vis_img = orig_img.copy()

        # 1. 绘制 mask 半透明覆盖（黄色通道）
        mask_colored = np.zeros_like(vis_img)
        mask_colored[mask_binary > 0] = (0, 255, 255)  # 黄色
        vis_img = cv2.addWeighted(vis_img, 0.6, mask_colored, 0.4, 0)

        # 2. 绘制左右边界
        for row in range(0, H, 10):
            row_data = mask_binary[row, :]
            valid_pixels = np.where(row_data > 0)[0]
            if len(valid_pixels) >= 2:
                left_edge = valid_pixels[0]
                right_edge = valid_pixels[-1]
                cv2.circle(vis_img, (int(left_edge), row), 3, (255, 0, 0), -1)  # 蓝色左边界
                cv2.circle(vis_img, (int(right_edge), row), 3, (0, 0, 255), -1)  # 红色右边界

        # 3. 绘制中线路径
        valid_centerline = [p for p in centerline_points if p is not None]
        if len(valid_centerline) >= 2:
            points = np.array([(int(p[0]), int(p[1])) for p in valid_centerline], dtype=np.int32)
            cv2.polylines(vis_img, [points], False, (0, 255, 0), 3)  # 绿色中线

            # 标注前瞻点
            lookahead_idx = int(len(valid_centerline) * self._lookahead_ratio)
            lookahead_idx = np.clip(lookahead_idx, 0, len(valid_centerline) - 1)
            lookahead_pt = valid_centerline[lookahead_idx]
            cv2.circle(vis_img, (int(lookahead_pt[0]), int(lookahead_pt[1])), 8, (255, 0, 255), -1)  # 紫色前瞻点

        # 4. 绘制中心线参考
        center_x = W // 2
        cv2.line(vis_img, (center_x, 0), (center_x, H), (255, 255, 255), 1)

        # 5. 绘制状态信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        y_offset = 30
        line_height = 25

        info_texts = [
            f"Lateral: {status['lateral_error']:+.3f}",
            f"Heading: {status['heading_error_deg']:+.1f} deg",
            f"Curve: {status['curvature']:.3f}",
            f"Remain: {status['remaining_m']:.2f}m" if status['remaining_m'] is not None else "Remain: N/A",
            f"Conf: {status['confidence']:.2f}",
            f"Safe: {'YES' if status['boundary_safe'] else 'NO'}",
            f"Valid: {'YES' if status['valid'] else 'NO'}",
        ]

        for i, text in enumerate(info_texts):
            color = (0, 255, 0) if status['valid'] else (0, 0, 255)
            cv2.putText(vis_img, text, (10, y_offset + i * line_height),
                        font, 0.6, color, 2)

        # 6. 绘制 FPS
        if len(self._fps_queue) > 0:
            avg_fps = 1.0 / (np.mean(self._fps_queue) + 1e-6)
            cv2.putText(vis_img, f"FPS: {avg_fps:.1f}", (W - 150, 30),
                        font, 0.7, (255, 255, 0), 2)

        return vis_img

    # ═══════════════════════════════════════════════════════
    # 相机回调与主处理流程
    # ═══════════════════════════════════════════════════════

    def _image_callback(self, msg):
        """相机图像回调"""
        with self._lock:
            if not self._inference_active:
                return

        # 临时：完全禁用 BPU 推理，只返回模拟数据测试控制逻辑
        self._frame_count += 1

        if self._frame_count % 10 == 0:
            self._node.get_logger().info(f'[Stage1视觉] 接收到第 {self._frame_count} 帧（模拟模式）')

        # 模拟视觉数据
        now = time.time()
        with self._lock:
            self._latest_lateral_error = 0.0  # 模拟：居中
            self._latest_heading_error_deg = 0.0
            self._latest_curvature = 0.0
            self._latest_remaining_m = 1.5  # 模拟：还有1.5米
            self._latest_confidence = 0.8
            self._valid = True
            self._boundary_safe = True
            self._latest_timestamp = now

        return  # 跳过真正的推理

        # 下面是原始的推理代码（暂时不执行）
        try:
            # 转换为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self._node.get_logger().error(f'[Stage1视觉] 图像转换失败: {e}')
            return

        self._frame_count += 1

        # ROI 裁剪
        H, W = cv_image.shape[:2]
        crop_top = int(H * self.crop_ratio)
        crop_left = int(W * self.crop_side_ratio)
        crop_right = int(W * (1 - self.crop_side_ratio))
        roi = cv_image[crop_top:H, crop_left:crop_right]

        # 预处理
        t0 = time.perf_counter()
        input_tensor, ratio, pad_w, pad_h = self._preprocess(roi)

        # BPU 推理
        outputs = self.model.forward(input_tensor)
        t1 = time.perf_counter()
        self._infer_time_ms = (t1 - t0) * 1000.0

        # 后处理
        mask, box, conf = self._postprocess(outputs, roi.shape[:2], ratio, pad_w, pad_h)

        if mask is None:
            # 无检测
            with self._lock:
                self._valid = False
                self._latest_confidence = 0.0
                self._node.get_logger().debug('[Stage1视觉] 无通道检测')
            return

        # 二值化 mask
        mask_binary = (mask > self._mask_threshold).astype(np.uint8) * 255

        # 提取中线
        centerline_points, valid_rows, confidence = self._extract_centerline(mask_binary)

        if valid_rows < self._min_valid_rows:
            with self._lock:
                self._valid = False
                self._node.get_logger().debug(f'[Stage1视觉] 有效行数不足: {valid_rows}/{self._min_valid_rows}')
            return

        # 计算控制误差
        lateral_error, heading_error_deg, curvature = self._compute_control_errors(
            centerline_points, roi.shape[1], roi.shape[0]
        )

        # 误差滤波
        if self._has_filtered_error:
            lateral_error = (self._error_filter_alpha * lateral_error +
                             (1 - self._error_filter_alpha) * self._filtered_error)
        self._filtered_error = lateral_error
        self._has_filtered_error = True

        # 估计剩余距离
        remaining_m = self._estimate_remaining_distance(mask_binary)

        # 边界安全检查
        boundary_safe, safety_weight = self._check_boundary_safety(mask_binary)

        # 更新共享状态
        now = time.time()
        with self._lock:
            self._latest_lateral_error = lateral_error
            self._latest_heading_error_deg = heading_error_deg
            self._latest_curvature = curvature
            self._latest_remaining_m = remaining_m
            self._latest_confidence = confidence
            self._valid = True
            self._boundary_safe = boundary_safe
            self._latest_timestamp = now

        # 日志输出（每 10 帧一次）
        if self._frame_count % 10 == 0:
            self._node.get_logger().info(
                f'[Stage1视觉] F#{self._frame_count} | '
                f'Lateral={lateral_error:+.3f} | '
                f'Head={heading_error_deg:+.1f}° | '
                f'Remain={remaining_m:.2f}m | '
                f'Conf={confidence:.2f} | '
                f'Safe={boundary_safe} | '
                f'Infer={self._infer_time_ms:.1f}ms'
            )

        # 可视化
        status = {
            'lateral_error': lateral_error,
            'heading_error_deg': heading_error_deg,
            'curvature': curvature,
            'remaining_m': remaining_m,
            'confidence': confidence,
            'valid': True,
            'boundary_safe': boundary_safe,
            'timestamp': now,
        }
        vis_img = self._draw_visualization(roi, mask_binary, centerline_points, status)

        # 保存图像（FPS 控制）
        if now - self._last_save_time >= self._min_frame_interval:
            cv2.imwrite(self._jpeg_output_path, vis_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self._last_save_time = now
            self._last_frame_save_time = now

        # FPS 统计
        if self._last_time > 0:
            dt = time.perf_counter() - self._last_time
            self._fps_queue.append(dt)
        self._last_time = time.perf_counter()

        if not self._first_frame_ready:
            self._first_frame_ready = True
            self._node.get_logger().info('[Stage1视觉] 首帧推理完成')