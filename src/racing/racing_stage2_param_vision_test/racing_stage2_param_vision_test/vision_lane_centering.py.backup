#!/usr/bin/env python3
"""
vision_lane_centering.py — 视觉车道居中模块

从 camera_all_in_one.py 抽离的推理节点，提供：
1. 订阅相机 topic，BPU 推理，缓存最新 offset
2. Flask Web 服务（实时推理画面流）

共享接口：
    get_latest_offset() -> (offset: float, timestamp: float, valid: bool)
"""

import threading
import time
from collections import deque
from io import BytesIO

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, ReliabilityPolicy

from hobot_dnn import pyeasy_dnn as dnn
from flask import Flask, Response, render_template_string


class VisionLaneCentering:
    """
    视觉车道居中推理节点。
    
    外部接口：
        get_latest_offset() -> (offset, timestamp, valid)
            - offset: [-1.0, +1.0]，负=偏左，正=偏右
            - timestamp: 检测时刻（秒）
            - valid: 是否有效（超时或无检测→False）
        
        start_web_server(port=8080) -> Flask app（可选，用于实时查看）
    """
    
    def __init__(self, parent_node, model_path, conf_thres=0.25, iou_thres=0.45, crop_ratio=0.4):
        self._node = parent_node
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.crop_ratio = crop_ratio
        
        # 共享变量（线程安全）
        self._lock = threading.Lock()
        self._latest_offset = 0.0
        self._latest_timestamp = 0.0
        self._valid = False
        self._detection_timeout_sec = 0.5  # 超过 0.5s 无检测 → invalid
        
        # 可视化缓存（供 Web 服务）
        self._combined_frame = None
        self._jpeg_buffer = None  # 预编码 JPEG 缓存（避免 Web 线程重复编码）
        
        # 占位图缓存（只生成一次，避免重复编码）
        self._placeholder_jpeg = self._generate_placeholder_jpeg()
        
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
        
        self._node.get_logger().info('[视觉] 模块初始化完成，等待相机数据...')
    
    # ═══════════════════════════════════════════════════════
    # 外部接口（供导航节点调用）
    # ═══════════════════════════════════════════════════════
    
    def _generate_placeholder_jpeg(self):
        """生成占位图 JPEG（只调用一次，缓存结果）"""
        # 小尺寸占位图（320x180），方便 JavaScript 识别（真实画面是 2880x360）
        placeholder = np.zeros((180, 320, 3), dtype=np.uint8)
        cv2.putText(placeholder, 'Waiting...', (80, 90),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (100, 100, 100), 2)
        ret, jpeg = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 50])
        return jpeg.tobytes() if ret else b''
    
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
    
    def get_frame(self):
        """获取最新可视化帧（供 Web 服务）"""
        with self._lock:
            if self._combined_frame is not None:
                return self._combined_frame.copy()
            else:
                placeholder = np.zeros((480, 960, 3), dtype=np.uint8)
                cv2.putText(placeholder, 'Waiting for camera...', (300, 220),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
                return placeholder
    
    def get_jpeg_buffer(self):
        """获取最新预编码 JPEG 字节（Web 直接发送，避免重复编码）"""
        with self._lock:
            if self._jpeg_buffer is not None:
                return self._jpeg_buffer
            # 无预编码帧时，返回缓存的占位图（小尺寸，方便 JS 识别）
            return self._placeholder_jpeg
    
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
                        detection_valid = True
            
            # 6. 更新共享变量
            with self._lock:
                if detection_valid:
                    self._latest_offset = float(best_offset)
                    self._latest_timestamp = time.time()
                    self._valid = True
                else:
                    self._valid = False  # 本帧无检测
            
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
            
            with self._lock:
                self._combined_frame = combined.copy()
                # 预编码 JPEG（供 Web 直接使用，避免重复编码）
                ret, jpeg = cv2.imencode('.jpg', combined, [cv2.IMWRITE_JPEG_QUALITY, 60])
                if ret:
                    self._jpeg_buffer = jpeg.tobytes()
                self._frame_count += 1
            
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
# Flask Web 服务（可选）
# ═══════════════════════════════════════════════════════

def create_web_app(vision_node):
    """创建 Flask Web 服务，供实时查看推理画面（优化版：gevent + 零缓存）"""
    import logging
    log = logging.getLogger('werkzeug')
    log.setLevel(logging.ERROR)  # 减少 Flask 日志噪音
    
    app = Flask(__name__)
    app.config['SECRET_KEY'] = 'racing-vision-2026'
    app.config['SEND_FILE_MAX_AGE_DEFAULT'] = 0  # 禁用静态文件缓存
    vision_node._node.get_logger().info('[视觉] Flask app 创建完成（配置：零缓存）')
    
    # HTML 模板（预编译为常量，避免每次请求都用 render_template_string 解析）
    HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>视觉车道居中 - 实时推理</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
    <meta http-equiv="Cache-Control" content="no-cache, no-store, must-revalidate">
    <meta http-equiv="Pragma" content="no-cache">
    <meta http-equiv="Expires" content="0">
    <style>
        body {
            margin: 0;
            padding: 20px;
            background: #1a1a1a;
            font-family: Arial, sans-serif;
            color: white;
        }
        h1 {
            text-align: center;
            color: #00ff00;
            margin-bottom: 10px;
        }
        .subtitle {
            text-align: center;
            color: #888;
            margin-bottom: 20px;
        }
        .container {
            max-width: 1600px;
            margin: 0 auto;
        }
        img {
            width: 100%;
            height: auto;
            border: 3px solid #00ff00;
            border-radius: 8px;
        }
        .info {
            margin-top: 20px;
            padding: 15px;
            background: #2a2a2a;
            border-radius: 8px;
        }
        .info p {
            margin: 5px 0;
        }
        .status {
            display: inline-block;
            padding: 5px 10px;
            background: #00ff00;
            color: #000;
            border-radius: 4px;
            font-weight: bold;
        }
    </style>
</head>
<body>
    <div class="container">
        <h1>🚗 视觉车道居中 - 实时推理</h1>
        <div class="subtitle"><span class="status">LIVE</span> 视觉修正 + 导航融合</div>
        <img src="/video_feed" alt="实时推理画面">
        <div class="info">
            <p><strong>功能说明：</strong></p>
            <p>• 左侧：裁剪后输入（下方 40%）</p>
            <p>• 右侧：检测结果（绿色 mask + bbox + 中心点 + 偏移）</p>
            <p>• offset < 0（负）→ 目标偏左 → 需要左转</p>
            <p>• offset > 0（正）→ 目标偏右 → 需要右转</p>
            <p><strong>集成状态：</strong> 已融合到导航控制器（move 段 + leg2 段）</p>
            <p><strong>提示：</strong>如果看不到画面，请等待 2-3 秒后手动刷新页面（F5）</p>
        </div>
    </div>
</body>
</html>
"""
    
    @app.route('/')
    def index():
        """返回 HTML 首页（静态字符串，无模板解析开销）"""
        return HTML_TEMPLATE, 200, {'Content-Type': 'text/html; charset=utf-8'}
    
    @app.route('/video_feed')
    def video_feed():
        """MJPEG 视频流（预编码 JPEG + 零缓存头）"""
        def generate():
            """极简：直接发送预编码 JPEG，零编码延迟"""
            while True:
                jpeg = vision_node.get_jpeg_buffer()
                if jpeg is None:
                    time.sleep(0.05)
                    continue
                # 添加 Cache-Control 头到每一帧（避免浏览器缓存导致卡顿）
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n'
                       b'Cache-Control: no-cache, no-store, must-revalidate\r\n'
                       b'Pragma: no-cache\r\n'
                       b'Expires: 0\r\n\r\n' + jpeg + b'\r\n')
                time.sleep(0.03)  # 约 30fps，但实际取决于推理帧率
        
        response = Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
        # 响应头级别的缓存控制
        response.headers['Cache-Control'] = 'no-cache, no-store, must-revalidate'
        response.headers['Pragma'] = 'no-cache'
        response.headers['Expires'] = '0'
        response.headers['X-Accel-Buffering'] = 'no'  # 禁用 nginx 等反向代理缓冲
        return response
    
    @app.route('/health')
    def health():
        """健康检查端点（快速响应，无视频流，纯 JSON）"""
        import time
        with vision_node._lock:
            status = {
                'status': 'ok',
                'timestamp': time.time(),
                'frame_count': vision_node._frame_count,
                'has_frame': vision_node._jpeg_buffer is not None,
                'last_offset': vision_node._last_offset if hasattr(vision_node, '_last_offset') else None,
                'frame_ready': vision_node._first_frame_ready
            }
        return status, 200, {'Content-Type': 'application/json'}
    
    return app
