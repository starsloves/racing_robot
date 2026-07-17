#!/usr/bin/env python3
"""Stage2 SEG line follower.

独立实验节点：
- 裁掉相机画面左右边缘，只保留中间 ROI；
- 使用 YOLOv8-Seg BPU 模型分割黄色赛道；
- 从 mask 多行采样赛道中心线，发布跟线控制速度；
- 提供 MJPEG 实时画面，便于现场看中线/目标点。
"""

import json
import math
import os
import threading
import time
from collections import deque
from http.server import SimpleHTTPRequestHandler
from socketserver import ThreadingTCPServer

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String

try:
    from hobot_dnn import pyeasy_dnn as dnn
except Exception as exc:  # pragma: no cover - 板端依赖
    dnn = None
    _DNN_IMPORT_ERROR = exc
else:
    _DNN_IMPORT_ERROR = None


class SegLineFollower(Node):
    """基于 SEG mask 中线的阶段二跟线实验节点。"""

    def __init__(self):
        super().__init__('seg_line_follower')
        self._declare_params()
        self._read_params()

        self.bridge = CvBridge()
        self.model = self._load_model()
        self.input_size = 640
        self.strides = [8, 16, 32]

        self.phase = 1
        self.direction = None
        self.enabled_by_state = not self.phase_gate_enabled
        self.latest_center_error = 0.0
        self.latest_line_valid = False
        self.last_detection_time = 0.0
        self.latest_image_size = None
        self.latest_roi_size = None
        self.frame_count = 0
        self.infer_time_ms = 0.0
        self.fps_queue = deque(maxlen=30)
        self.last_frame_time = time.perf_counter()

        self.lock = threading.Lock()
        self.latest_frame = self._placeholder_frame()
        self.latest_jpeg_path = '/tmp/stage2_seg_follow.jpg'
        cv2.imwrite(self.latest_jpeg_path, self.latest_frame)

        qos_image = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Image, self.image_topic, self._image_cb, qos_image)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.debug_pub = self.create_publisher(Image, self.debug_image_topic, 2)

        qos_latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Int32, self.phase_topic, self._phase_cb, qos_latched)
        self.create_subscription(String, self.qr_task_topic, self._qr_task_cb, qos_latched)

        self.create_timer(0.1, self._watchdog_cb)
        self._start_http_server()
        self.get_logger().info(
            f'SEG跟线节点就绪: topic={self.image_topic}, cmd={self.cmd_vel_topic}, '
            f'crop_left/right={self.crop_left_px}/{self.crop_right_px}, phase_gate={self.phase_gate_enabled}, '
            f'HTTP=http://0.0.0.0:{self.http_port}/'
        )

    def _declare_params(self):
        self.declare_parameter('image_topic', '/aurora/rgb/image_raw')
        self.declare_parameter('cmd_vel_topic', '/stage2_cmd_vel')
        self.declare_parameter('debug_image_topic', '/stage2_seg_follow/debug_image')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('qr_task_topic', 'competition_qr_task')
        self.declare_parameter('model_path', '/home/sunrise/dev_ws/src/racing/racing_stage2/models/bset.bin')
        self.declare_parameter('conf_thres', 0.25)
        self.declare_parameter('iou_thres', 0.45)
        self.declare_parameter('crop_left_px', 160)
        self.declare_parameter('crop_right_px', 160)
        self.declare_parameter('crop_reference_width', 640)
        self.declare_parameter('scale_crop_to_image_width', True)
        self.declare_parameter('phase_gate_enabled', True)
        self.declare_parameter('target_phase', 2)
        self.declare_parameter('linear_speed', 0.08)
        self.declare_parameter('corner_linear_speed', 0.05)
        self.declare_parameter('min_linear_speed', 0.03)
        self.declare_parameter('angular_kp', 1.25)
        self.declare_parameter('angular_kd', 0.20)
        self.declare_parameter('curvature_kp', 0.45)
        self.declare_parameter('max_angular_speed', 0.75)
        self.declare_parameter('deadband', 0.035)
        self.declare_parameter('offset_filter_alpha', 0.35)
        self.declare_parameter('lookahead_ratio', 0.62)
        self.declare_parameter('sample_rows', 9)
        self.declare_parameter('mask_threshold', 0.50)
        self.declare_parameter('min_mask_pixels_per_row', 12)
        self.declare_parameter('min_valid_rows', 4)
        self.declare_parameter('lost_timeout_sec', 0.35)
        self.declare_parameter('search_angular_speed', 0.25)
        self.declare_parameter('http_port', 8092)
        self.declare_parameter('publish_debug_image', True)

    def _read_params(self):
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.cmd_vel_topic = str(self.get_parameter('cmd_vel_topic').value)
        self.debug_image_topic = str(self.get_parameter('debug_image_topic').value)
        self.phase_topic = str(self.get_parameter('phase_topic').value)
        self.qr_task_topic = str(self.get_parameter('qr_task_topic').value)
        self.model_path = str(self.get_parameter('model_path').value)
        self.conf_thres = float(self.get_parameter('conf_thres').value)
        self.iou_thres = float(self.get_parameter('iou_thres').value)
        self.crop_left_px = int(self.get_parameter('crop_left_px').value)
        self.crop_right_px = int(self.get_parameter('crop_right_px').value)
        self.crop_reference_width = max(1, int(self.get_parameter('crop_reference_width').value))
        self.scale_crop_to_image_width = bool(self.get_parameter('scale_crop_to_image_width').value)
        self.phase_gate_enabled = bool(self.get_parameter('phase_gate_enabled').value)
        self.target_phase = int(self.get_parameter('target_phase').value)
        self.linear_speed = float(self.get_parameter('linear_speed').value)
        self.corner_linear_speed = float(self.get_parameter('corner_linear_speed').value)
        self.min_linear_speed = float(self.get_parameter('min_linear_speed').value)
        self.angular_kp = float(self.get_parameter('angular_kp').value)
        self.angular_kd = float(self.get_parameter('angular_kd').value)
        self.curvature_kp = float(self.get_parameter('curvature_kp').value)
        self.max_angular_speed = float(self.get_parameter('max_angular_speed').value)
        self.deadband = float(self.get_parameter('deadband').value)
        self.offset_filter_alpha = float(self.get_parameter('offset_filter_alpha').value)
        self.lookahead_ratio = float(self.get_parameter('lookahead_ratio').value)
        self.sample_rows = max(3, int(self.get_parameter('sample_rows').value))
        self.mask_threshold = float(self.get_parameter('mask_threshold').value)
        self.min_mask_pixels_per_row = int(self.get_parameter('min_mask_pixels_per_row').value)
        self.min_valid_rows = int(self.get_parameter('min_valid_rows').value)
        self.lost_timeout_sec = float(self.get_parameter('lost_timeout_sec').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.http_port = int(self.get_parameter('http_port').value)
        self.publish_debug_image = bool(self.get_parameter('publish_debug_image').value)
        self.filtered_error = 0.0
        self.prev_error = 0.0
        self.prev_error_time = time.time()

    def _load_model(self):
        if dnn is None:
            raise RuntimeError(f'hobot_dnn import failed: {_DNN_IMPORT_ERROR}')
        self.get_logger().info(f'加载 SEG 模型: {self.model_path}')
        models = dnn.load(self.model_path)
        return models[0]

    def _phase_cb(self, msg):
        self.phase = int(msg.data)
        self.enabled_by_state = (not self.phase_gate_enabled) or self.phase == self.target_phase
        if not self.enabled_by_state:
            self.latest_line_valid = False
            self.cmd_pub.publish(Twist())

    def _qr_task_cb(self, msg):
        text = msg.data.strip().lower()
        if 'counter' in text or 'ccw' in text or '逆' in text:
            self.direction = 'counterclockwise'
        elif 'clockwise' in text or 'cw' in text or '顺' in text:
            self.direction = 'clockwise'

    def _watchdog_cb(self):
        if not self.enabled_by_state:
            return
        if self.latest_line_valid and time.time() - self.last_detection_time > self.lost_timeout_sec:
            self.latest_line_valid = False
            self.cmd_pub.publish(Twist())
            self.get_logger().warn('SEG中线超时，停车等待重新检测')

    def _image_cb(self, msg):
        if not self.enabled_by_state:
            return
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = img.shape[:2]
            left, right = self._compute_crop_bounds(w)
            roi = img[:, left:right].copy()

            t0 = time.perf_counter()
            mask, detections = self._infer_seg_mask(roi)
            self.infer_time_ms = (time.perf_counter() - t0) * 1000.0

            line = self._extract_centerline(mask)
            vis, command = self._visualize_and_control(roi, mask, line, detections)

            self.cmd_pub.publish(command)
            self.frame_count += 1
            with self.lock:
                self.latest_image_size = (w, h)
                self.latest_roi_size = (right - left, h)
                self.latest_frame = vis.copy()
            if self.publish_debug_image:
                self.debug_pub.publish(self.bridge.cv2_to_imgmsg(vis, encoding='bgr8'))
            cv2.imwrite(self.latest_jpeg_path, vis, [cv2.IMWRITE_JPEG_QUALITY, 85])
        except Exception as exc:
            self.get_logger().error(f'SEG跟线处理失败: {exc}')

    def _compute_crop_bounds(self, image_width):
        scale = 1.0
        if self.scale_crop_to_image_width:
            scale = float(image_width) / float(self.crop_reference_width)
        left_crop = int(round(max(0, self.crop_left_px) * scale))
        right_crop = int(round(max(0, self.crop_right_px) * scale))
        min_roi_width = max(80, int(image_width * 0.25))
        if image_width - left_crop - right_crop < min_roi_width:
            max_total_crop = max(0, image_width - min_roi_width)
            total_requested = max(1, left_crop + right_crop)
            left_crop = int(round(max_total_crop * left_crop / total_requested))
            right_crop = max_total_crop - left_crop
        left = min(max(0, left_crop), max(0, image_width - 2))
        right = max(left + 1, image_width - max(0, right_crop))
        return left, right

    def _infer_seg_mask(self, image):
        canvas, scale = self._letterbox(image, self.input_size)
        nv12 = self._bgr2nv12(canvas)
        outs = self.model.forward([nv12])

        all_bboxes, all_scores, all_coeff = [], [], []
        for si, stride in enumerate(self.strides):
            reg = outs[si * 3].buffer.reshape(-1, 64)
            cls = self._sigmoid(outs[si * 3 + 1].buffer.reshape(-1, 1))
            coeff = outs[si * 3 + 2].buffer.reshape(-1, 32)
            grid_h, grid_w = outs[si * 3].buffer.shape[1:3]
            sx, sy = np.meshgrid(np.arange(grid_w) + 0.5, np.arange(grid_h) + 0.5, indexing='xy')
            anchors = np.stack((sx.ravel(), sy.ravel()), axis=1).astype(np.float32)
            box = self._dfl_decode(reg) * stride
            bboxes = np.column_stack([
                anchors[:, 0] * stride - box[:, 0],
                anchors[:, 1] * stride - box[:, 1],
                anchors[:, 0] * stride + box[:, 2],
                anchors[:, 1] * stride + box[:, 3],
            ])
            all_bboxes.append(bboxes)
            all_scores.append(cls[:, 0])
            all_coeff.append(coeff)

        bboxes = np.concatenate(all_bboxes)
        scores = np.concatenate(all_scores)
        coeffs = np.concatenate(all_coeff)
        keep = scores > self.conf_thres
        bboxes, scores, coeffs = bboxes[keep], scores[keep], coeffs[keep]

        h, w = image.shape[:2]
        final_mask = np.zeros((h, w), dtype=np.uint8)
        detections = []
        if len(bboxes) == 0:
            return final_mask, detections

        xywh = np.zeros_like(bboxes)
        xywh[:, 0] = bboxes[:, 0]
        xywh[:, 1] = bboxes[:, 1]
        xywh[:, 2] = bboxes[:, 2] - bboxes[:, 0]
        xywh[:, 3] = bboxes[:, 3] - bboxes[:, 1]
        idxs = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), self.conf_thres, self.iou_thres)
        if isinstance(idxs, np.ndarray):
            idxs = idxs.flatten()
        elif isinstance(idxs, (list, tuple)):
            idxs = np.array(idxs).flatten()
        else:
            idxs = np.array([], dtype=int)

        proto = outs[9].buffer.squeeze()
        proto_h, proto_w = proto.shape[:2]
        protos_2d = proto.reshape(proto_h * proto_w, -1).T
        mask_scale = proto_h / self.input_size

        for i in idxs:
            box = bboxes[i]
            x1 = max(0, min(w - 1, box[0] / scale))
            y1 = max(0, min(h - 1, box[1] / scale))
            x2 = max(0, min(w - 1, box[2] / scale))
            y2 = max(0, min(h - 1, box[3] / scale))
            bw, bh = int(x2 - x1), int(y2 - y1)
            if bw <= 0 or bh <= 0:
                continue
            raw = coeffs[i] @ protos_2d
            mask_sig = self._sigmoid(raw.reshape(proto_h, proto_w))
            bx1 = int(max(0, box[0] * mask_scale))
            by1 = int(max(0, box[1] * mask_scale))
            bx2 = int(min(proto_w, box[2] * mask_scale))
            by2 = int(min(proto_h, box[3] * mask_scale))
            crop = mask_sig[by1:by2, bx1:bx2]
            if crop.size == 0:
                continue
            resized = cv2.resize(crop, (bw, bh))
            mask_bin = (resized > self.mask_threshold).astype(np.uint8)
            det_mask = np.zeros((h, w), dtype=np.uint8)
            det_mask[int(y1):int(y1) + bh, int(x1):int(x1) + bw] = mask_bin
            final_mask = np.maximum(final_mask, det_mask)
            detections.append((int(x1), int(y1), int(x2), int(y2), float(scores[i])))
        return final_mask, detections

    def _extract_centerline(self, mask):
        h, w = mask.shape[:2]
        rows = np.linspace(int(h * 0.90), int(h * 0.25), self.sample_rows).astype(int)
        samples = []
        for row in rows:
            y0 = max(0, row - 2)
            y1 = min(h, row + 3)
            xs = np.where(mask[y0:y1, :] > 0)[1]
            if xs.size < self.min_mask_pixels_per_row:
                continue
            left_x = float(xs.min())
            right_x = float(xs.max())
            center_x = (left_x + right_x) * 0.5
            samples.append({
                'left': (left_x, float(row)),
                'right': (right_x, float(row)),
                'center': (center_x, float(row)),
                'width': right_x - left_x,
            })
        if len(samples) < self.min_valid_rows:
            return None
        return samples

    def _visualize_and_control(self, roi, mask, line, detections):
        h, w = roi.shape[:2]
        overlay = roi.copy()
        green = np.zeros_like(overlay)
        green[mask > 0] = (0, 255, 0)
        overlay = cv2.addWeighted(overlay, 1.0, green, 0.45, 0.0)
        for x1, y1, x2, y2, score in detections:
            cv2.rectangle(overlay, (x1, y1), (x2, y2), (0, 220, 255), 2)
            cv2.putText(overlay, f'{score:.2f}', (x1, max(20, y1 - 6)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 220, 255), 2)

        command = Twist()
        mode = 'lost'
        target = None
        curve_error = 0.0
        if line is not None:
            center_points = [(int(s['center'][0]), int(s['center'][1])) for s in line]
            left_points = [(int(s['left'][0]), int(s['left'][1])) for s in line]
            right_points = [(int(s['right'][0]), int(s['right'][1])) for s in line]
            cv2.polylines(overlay, [np.array(left_points, dtype=np.int32)], False, (0, 165, 255), 2)
            cv2.polylines(overlay, [np.array(right_points, dtype=np.int32)], False, (0, 165, 255), 2)
            cv2.polylines(overlay, [np.array(center_points, dtype=np.int32)], False, (255, 0, 255), 3)
            for sample in line:
                left_pt = (int(sample['left'][0]), int(sample['left'][1]))
                right_pt = (int(sample['right'][0]), int(sample['right'][1]))
                center_pt = (int(sample['center'][0]), int(sample['center'][1]))
                cv2.line(overlay, left_pt, right_pt, (255, 255, 0), 1)
                cv2.circle(overlay, center_pt, 4, (255, 0, 255), -1)

            target_idx = int(round((len(line) - 1) * max(0.0, min(1.0, 1.0 - self.lookahead_ratio))))
            target_idx = max(0, min(len(line) - 1, target_idx))
            target = line[target_idx]['center']
            bottom_x = line[0]['center'][0]
            top_x = line[-1]['center'][0]
            curve_error = (bottom_x - top_x) / max(1.0, w * 0.5)
            raw_error = target[0] / (w * 0.5) - 1.0
            alpha = max(0.0, min(1.0, self.offset_filter_alpha))
            self.filtered_error = alpha * raw_error + (1.0 - alpha) * self.filtered_error
            error = 0.0 if abs(self.filtered_error) < self.deadband else self.filtered_error
            now = time.time()
            dt = max(1e-3, now - self.prev_error_time)
            deriv = (error - self.prev_error) / dt
            angular = -(self.angular_kp * error + self.angular_kd * deriv + self.curvature_kp * curve_error)
            angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
            speed = self.linear_speed
            if abs(curve_error) > 0.25 or abs(error) > 0.35:
                speed = self.corner_linear_speed
            speed = max(self.min_linear_speed, speed)
            command.linear.x = speed
            command.angular.z = angular
            self.prev_error = error
            self.prev_error_time = now
            self.latest_center_error = error
            self.latest_line_valid = True
            self.last_detection_time = time.time()
            mode = 'follow'
            cv2.circle(overlay, (int(target[0]), int(target[1])), 8, (0, 0, 255), -1)
            cv2.line(overlay, (w // 2, h), (int(target[0]), int(target[1])), (255, 0, 0), 2)
        else:
            command.linear.x = 0.0
            command.angular.z = self._search_direction() * self.search_angular_speed
            self.latest_line_valid = False
            self.filtered_error = 0.0

        cv2.line(overlay, (w // 2, 0), (w // 2, h), (255, 255, 255), 2)
        cv2.putText(overlay, f'mode={mode} v={command.linear.x:.2f} w={command.angular.z:.2f}', (12, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(overlay, f'err={self.latest_center_error:+.3f} curve={curve_error:+.3f} infer={self.infer_time_ms:.1f}ms', (12, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 255), 2)
        cv2.putText(overlay, f'CROPPED ROI: left {self.crop_left_px}px | right {self.crop_right_px}px', (12, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        target_h = 480
        if overlay.shape[0] != target_h:
            overlay = cv2.resize(overlay, (int(overlay.shape[1] * target_h / overlay.shape[0]), target_h))
        return overlay, command

    def _search_direction(self):
        if self.direction == 'counterclockwise':
            return -1.0
        return 1.0

    def _start_http_server(self):
        parent = self

        class Handler(SimpleHTTPRequestHandler):
            def do_GET(self):
                if self.path in ('/', '/index.html'):
                    body = f'''<!doctype html><html><head><meta charset="utf-8"><title>Stage2 SEG Follow</title>
<style>body{{background:#202124;color:#eee;font-family:sans-serif;margin:18px}}img{{max-width:100%;border:1px solid #555}}</style></head>
<body><h2>Stage2 SEG Follow 实时画面</h2><p id="s">connecting...</p><img src="/stream.mjpg">
<script>async function h(){{try{{let r=await fetch('/health?t='+Date.now(),{{cache:'no-store'}});let d=await r.json();s.textContent=JSON.stringify(d)}}catch(e){{s.textContent=e}}}}setInterval(h,500);h();</script></body></html>'''.encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'text/html; charset=utf-8')
                    self.send_header('Content-Length', len(body))
                    self.send_header('Cache-Control', 'no-cache, no-store')
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith('/health'):
                    payload = {
                        'status': 'ok',
                        'phase': parent.phase,
                        'enabled': parent.enabled_by_state,
                        'valid': parent.latest_line_valid,
                        'frame_count': parent.frame_count,
                        'image_size': parent.latest_image_size,
                        'roi_size': parent.latest_roi_size,
                        'error': round(parent.latest_center_error, 3),
                    }
                    body = json.dumps(payload).encode('utf-8')
                    self.send_response(200)
                    self.send_header('Content-Type', 'application/json')
                    self.send_header('Content-Length', len(body))
                    self.send_header('Cache-Control', 'no-cache, no-store')
                    self.end_headers()
                    self.wfile.write(body)
                    return
                if self.path.startswith('/stream.mjpg'):
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache, no-store')
                    self.end_headers()
                    while True:
                        try:
                            with parent.lock:
                                frame = parent.latest_frame.copy()
                            ok, enc = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 82])
                            if ok:
                                data = enc.tobytes()
                                self.wfile.write(b'--frame\r\n')
                                self.wfile.write(b'Content-Type: image/jpeg\r\n')
                                self.wfile.write(f'Content-Length: {len(data)}\r\n\r\n'.encode('ascii'))
                                self.wfile.write(data)
                                self.wfile.write(b'\r\n')
                            time.sleep(0.05)
                        except (BrokenPipeError, ConnectionResetError):
                            break
                    return
                self.send_error(404)

            def log_message(self, *_args):
                pass

        def serve():
            try:
                ThreadingTCPServer.allow_reuse_address = True
                httpd = ThreadingTCPServer(('0.0.0.0', self.http_port), Handler)
                httpd.daemon_threads = True
                httpd.serve_forever()
            except Exception as exc:
                parent.get_logger().error(f'SEG跟线HTTP服务失败: {exc}')

        threading.Thread(target=serve, daemon=True, name='SegFollowHTTP').start()

    def _placeholder_frame(self):
        frame = np.full((480, 640, 3), 28, dtype=np.uint8)
        cv2.putText(frame, 'Waiting for SEG follow frames...', (60, 230), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        return frame

    def _sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))

    def _dfl_decode(self, reg):
        count = reg.shape[0]
        proj = np.arange(16, dtype=np.float32)
        reg = reg.reshape(count, 4, 16)
        softmax = np.exp(reg - reg.max(axis=-1, keepdims=True))
        softmax /= softmax.sum(axis=-1, keepdims=True)
        return (softmax @ proj).reshape(count, 4)

    def _letterbox(self, img, target_size=640):
        h0, w0 = img.shape[:2]
        scale = min(target_size / h0, target_size / w0)
        nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
        resized = cv2.resize(img, (nw, nh))
        canvas = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = resized
        return canvas, scale

    def _bgr2nv12(self, bgr):
        h, w = bgr.shape[:2]
        i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).flatten()
        hw = h * w
        hw_uv = hw // 4
        nv12 = np.empty(hw * 3 // 2, dtype=np.uint8)
        nv12[:hw] = i420[:hw]
        uv = np.zeros(hw_uv * 2, dtype=np.uint8)
        uv[0::2] = i420[hw:hw + hw_uv]
        uv[1::2] = i420[hw + hw_uv:hw + hw_uv * 2]
        nv12[hw:] = uv
        return nv12


def main(args=None):
    rclpy.init(args=args)
    node = SegLineFollower()
    try:
        rclpy.spin(node)
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
