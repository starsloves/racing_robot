#!/usr/bin/env python3
"""
vision_lane_centering.py — 视觉车道居中模块

从 camera_all_in_one.py 抽离的推理节点，提供：
1. 订阅相机 topic，BPU 推理，缓存最新 offset
2. 实时保存处理后的可视化图像到 /tmp/vision_latest.jpg
3. 提供 HTTP 静态服务（端口 8080）供浏览器查看

共享接口：
    get_latest_offset() -> (offset: float, timestamp: float, valid: bool)
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
                 crop_ratio=0.4, http_port=8080):
        self._node = parent_node
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.crop_ratio = crop_ratio
        self.http_port = http_port
        
        # 共享变量（线程安全）
        self._lock = threading.Lock()
        self._latest_offset = 0.0
        self._latest_timestamp = 0.0
        self._valid = False
        self._detection_timeout_sec = 0.5  # 超过 0.5s 无检测 → invalid
        self._last_valid_state = None  # 记录上次有效状态（用于检测状态变化）
        
        # 可视化缓存（供 HTTP 服务）
        self._combined_frame = None
        self._jpeg_output_path = '/tmp/vision_latest.jpg'
        
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
                        
                        # 图像请求：固定返回 /tmp/vision_latest.jpg
                        if self.path.startswith('/vision_latest.jpg'):
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
            if self._frame_count == 0:
                self._node.get_logger().info('[视觉-诊断] 收到第一帧相机数据')
            
            # 1. 裁剪下方
            img_full = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h_full, w_full = img_full.shape[:2]
            crop_start_row = int(h_full * (1 - self.crop_ratio))
            img_cropped = img_full[crop_start_row:, :].copy()
            h_crop, w_crop = img_cropped.shape[:2]
            
            frame_before = img_cropped.copy()
            cv2.putText(frame_before, f'INPUT (Bottom {int(self.crop_ratio*100)}%)', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
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
            
            # 5. 计算 offset（取第一个检测）
            frame_after = img_cropped.copy()
            best_offset = 0.0
            best_box = None
            best_score = 0.0
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
                            mf[int(y1):int(y1)+bh, int(x1):int(x1)+bw] = mb
                            cm = np.zeros_like(frame_after)
                            cm[mf > 0] = (0, 255, 0)
                            frame_after = cv2.addWeighted(frame_after, 1.0, cm, 0.4, 0)
                            ct, _ = cv2.findContours(mf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                            cv2.drawContours(frame_after, ct, -1, (0, 255, 0), 2)
                    
                    cv2.rectangle(frame_after, (int(x1), int(y1)), (int(x2), int(y2)), 
                                 (0, 255, 0), 2)
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    cv2.circle(frame_after, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.line(frame_after, (cx, cy), (w_crop//2, h_crop//2), (255, 0, 0), 2)
                    
                    offset = cx / (w_crop / 2) - 1.0
                    cv2.putText(frame_after, f'conf={scores[i]:.2f} off={offset:+.2f}', 
                               (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 
                               0.5, (0, 255, 255), 2)
                    
                    # 取第一个检测的 offset
                    if not detection_valid:
                        best_offset = offset
                        best_box = box  # 保存用于日志
                        best_score = scores[i]  # 保存用于日志
                        detection_valid = True
            
            # 6. 更新共享变量 + 状态变化日志
            with self._lock:
                if detection_valid:
                    self._latest_offset = float(best_offset)
                    self._latest_timestamp = time.time()
                    self._valid = True
                    
                    # 状态变化日志：从失效→恢复
                    if self._last_valid_state is False:
                        if best_box is not None:
                            self._node.get_logger().info(
                                f'[VISION] 推理成功 offset={best_offset:+.3f} | '
                                f'bbox=({int(best_box[0])},{int(best_box[1])})→'
                                f'({int(best_box[2])},{int(best_box[3])}) '
                                f'conf={best_score:.2f}'
                            )
                        else:
                            self._node.get_logger().info(
                                f'[VISION] 推理成功 offset={best_offset:+.3f}'
                            )
                    self._last_valid_state = True
                else:
                    self._valid = False  # 本帧无检测
                    
                    # 状态变化日志：从有效→失效
                    if self._last_valid_state is True:
                        detections = None  # 占位，实际从 bboxes 长度获取
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
                    
                    # 每 30 帧打印一次确认
                    if self._frame_count % 30 == 0:
                        file_size = os.path.getsize(self._jpeg_output_path) / 1024
                        self._node.get_logger().info(
                            f'[视觉-诊断] 已保存第 {self._frame_count} 帧到 {self._jpeg_output_path} '
                            f'({combined.shape[1]}x{combined.shape[0]}, {file_size:.1f} KB, '
                            f'{avg_fps:.1f} FPS 推理, {self._target_fps} FPS 保存)'
                        )
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
