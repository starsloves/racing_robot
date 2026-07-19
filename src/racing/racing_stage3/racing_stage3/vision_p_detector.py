"""
vision_p_detector.py — P 标牌视觉检测模块

专用于 best_p.bin YOLO 检测模型推理：
1. 订阅相机 topic，BPU 推理，缓存最新检测结果
2. 解码 YOLO 检测头的 bbox 和单类别置信度，不计算 mask
3. 提供 get_p_detection() 接口供导航节点调用
4. HTTP 图片、健康检查和自动刷新的 Web 页面（兼容 vision_viewer.html 的 8080 接口）
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


class VisionPDetector:
    """
    P 标牌视觉检测模块。

    外部接口：
        get_p_detection() -> (detected, confidence, bbox, timestamp)
            - detected: bool, 是否检测到 P
            - confidence: float, 最高检测置信度
            - bbox: (x1,y1,x2,y2) 原始图像坐标或 None
            - timestamp: float, 检测时刻
    """

    def __init__(self, parent_node, model_path, conf_thres=0.25, iou_thres=0.45,
                 crop_ratio=0.4, input_size=640, http_port=8083):
        self._node = parent_node
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.crop_ratio = crop_ratio
        self.input_size = input_size
        self.http_port = http_port

        self._lock = threading.Lock()
        self._detected = False
        self._confidence = 0.0
        self._bbox = None
        self._timestamp = 0.0
        self._offset = 0.0
        self._fill_ratio = 0.0
        self._inference_active = False

        # HTTP 可视化
        self._combined_frame = None
        # 与现有 vision_viewer.html 共用的图像路径。
        self._jpeg_output_path = '/tmp/stage3_vision.jpg'
        self._http_server_start_time = time.time()
        self._last_frame_save_time = 0.0
        self._http_lock = threading.Lock()
        self._http_server = None
        self._http_enabled = False
        self._target_fps = 30
        self._min_frame_interval = 1.0 / self._target_fps
        self._last_save_time = 0.0
        self._layout_logged = False

        self.REG_MAX = 16
        self.strides = [8, 16, 32]

        self._node.get_logger().info(f'[P-DET] 加载模型: {model_path}')
        models = dnn.load(model_path)
        self.model = models[0]

        self.bridge = CvBridge()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._node.create_subscription(
            Image, '/aurora/rgb/image_raw', self._image_callback, qos
        )

        self._frame_count = 0
        self._fps_queue = deque(maxlen=30)
        self._last_time = time.perf_counter()
        self._infer_time_ms = 0.0

        # 创建占位图像
        self._write_placeholder_image()
        self._node.get_logger().info('[P-DET] 模块初始化完成，等待相机数据...')
        self._node.get_logger().info(f'[P-DET] HTTP 服务将在 phase=3 时绑定端口 {self.http_port}')

    def get_p_detection(self):
        with self._lock:
            return (self._detected, self._confidence, self._bbox, self._timestamp)

    def get_p_detection_geometry(self):
        """返回 P 检测结果及归一化水平偏差、画面填充比例。"""
        with self._lock:
            return (
                self._detected, self._confidence, self._bbox, self._timestamp,
                self._offset, self._fill_ratio,
            )

    def set_inference_active(self, active: bool):
        """启用/停用 P YOLO 推理。仅 phase=3 期间应启用。"""
        active = bool(active)
        with self._lock:
            previous = self._inference_active
            self._inference_active = active
            if not active:
                self._detected = False
                self._confidence = 0.0
                self._bbox = None
                self._timestamp = time.time()
                self._offset = 0.0
                self._fill_ratio = 0.0
        if previous != active:
            state = '启用' if active else '停用'
            self._node.get_logger().info(f'[P-DET] YOLO 推理已{state}')

    def is_inference_active(self) -> bool:
        with self._lock:
            return bool(self._inference_active)

    def start_http_server(self):
        """仅在 Stage3 活跃时绑定 8083。"""
        if self.http_port <= 0:
            return
        with self._http_lock:
            if self._http_enabled:
                return
            self._http_enabled = True
            self._http_server_start_time = time.time()
        self._start_http_server()

    def stop_http_server(self):
        """Stage3 结束时立即释放 8083。"""
        with self._http_lock:
            self._http_enabled = False
            server = self._http_server
        if server is not None:
            self._node.get_logger().info(f'[P-DET] HTTP_STOP port={self.http_port}')
            server.shutdown()

    def _write_placeholder_image(self):
        """预生成占位图像，避免浏览器首次连接时 404"""
        try:
            placeholder = np.full((360, 640, 3), 30, dtype=np.uint8)
            cv2.putText(placeholder, 'Waiting for P detection...', (50, 180),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            cv2.putText(placeholder, f'http://0.0.0.0:{self.http_port}', (50, 220),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
            cv2.imwrite(self._jpeg_output_path, placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
            self._node.get_logger().info('[P-DET] 占位图像已写入')
        except Exception as e:
            self._node.get_logger().warn(f'[P-DET] 写入占位图像失败: {e}')

    def _start_http_server(self):
        """启动 HTTP 静态文件服务器（后台线程）"""
        parent_self = self

        def serve():
            try:
                parent_self._node.get_logger().info('[P-DET] HTTP 服务线程启动...')

                class NoCacheHandler(SimpleHTTPRequestHandler):
                    def do_OPTIONS(self):
                        self.send_response(200)
                        self.send_header('Access-Control-Allow-Origin', '*')
                        self.send_header('Access-Control-Allow-Methods', 'GET, OPTIONS')
                        self.send_header('Access-Control-Allow-Headers', 'Content-Type, Pragma, Cache-Control')
                        self.send_header('Access-Control-Max-Age', '86400')
                        self.end_headers()

                    def do_GET(self):
                        if self.path in ('/', '/index.html', '/vision_p.html'):
                            viewer_path = os.path.abspath(
                                os.path.join(os.path.dirname(__file__), '../../../../vision_viewer.html')
                            )
                            try:
                                with open(viewer_path, 'rb') as viewer_file:
                                    body = viewer_file.read()
                            except OSError:
                                body = f'''<!doctype html>
<html><head><meta charset="utf-8"><title>Stage3 P YOLO</title>
<style>body{{background:#202124;color:#eee;font-family:sans-serif;margin:20px}}
img{{max-width:100%;border:1px solid #555}}#status{{margin:10px 0;color:#8f8}}</style></head>
<body><h2>Stage3 P YOLO 实时推理画面</h2><div id="status">连接中...</div>
<img id="frame" src="/stream.mjpg">
<script>
const status=document.getElementById('status');
async function health(){{ try{{ const r=await fetch('/health?t='+Date.now(),{{cache:'no-store'}}); const d=await r.json();
status.textContent='status='+d.status+' | phase='+d.phase+' | yolo='+(d.inference_active?'on':'off')+' | frames='+d.frame_count+' | age='+d.frame_age_sec+'s';
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
                            try:
                                with parent_self._lock:
                                    last_frame_time = parent_self._last_frame_save_time
                                    frame_count = parent_self._frame_count
                                    phase = getattr(parent_self._node, 'phase', 3)
                                    inference_active = parent_self._inference_active

                                uptime = time.time() - parent_self._http_server_start_time
                                age = time.time() - last_frame_time if last_frame_time > 0 else -1

                                import json
                                response = {
                                    "status": "ok",
                                    "uptime_sec": round(uptime, 2),
                                    "last_frame_time": last_frame_time,
                                    "frame_age_sec": round(age, 2) if age >= 0 else None,
                                    "frame_count": frame_count,
                                    "phase": phase,
                                    "inference_active": inference_active,
                                }

                                self.send_response(200)
                                self.send_header('Content-Type', 'application/json')
                                self.send_header('Access-Control-Allow-Origin', '*')
                                self.send_header('Cache-Control', 'no-cache, no-store')
                                self.end_headers()
                                self.wfile.write(json.dumps(response).encode('utf-8'))
                            except Exception as e:
                                parent_self._node.get_logger().error(f'[P-DET] /health 处理失败: {e}')
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
                                    parent_self._node.get_logger().error(f'[P-DET] MJPEG 流失败: {e}')
                                    break
                            return

                        if (
                            self.path.startswith('/image') or self.path.startswith('/vision_latest.jpg')
                            or self.path.startswith('/vision_p_latest.jpg')
                        ):
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
                            except BrokenPipeError:
                                # 客户端提前断开连接（浏览器刷新/超时），忽略
                                pass
                            except Exception as e:
                                parent_self._node.get_logger().error(f'[P-DET] 图像服务失败: {e}')
                                self.send_error(500, str(e))
                            return

                        self.send_error(404)

                    def log_message(self, format, *args):
                        pass  # 禁止打印访问日志

                    def handle(self):
                        try:
                            super().handle()
                        except (BrokenPipeError, ConnectionResetError):
                            pass

                ThreadingTCPServer.allow_reuse_address = True
                httpd = ThreadingTCPServer(("0.0.0.0", parent_self.http_port), NoCacheHandler)
                httpd.daemon_threads = True
                with parent_self._http_lock:
                    if not parent_self._http_enabled:
                        httpd.server_close()
                        return
                    parent_self._http_server = httpd

                parent_self._node.get_logger().info(f'[P-DET] HTTP 服务已创建，准备进入 serve_forever()')
                try:
                    httpd.serve_forever()
                finally:
                    httpd.server_close()
                    with parent_self._http_lock:
                        if parent_self._http_server is httpd:
                            parent_self._http_server = None
                    parent_self._node.get_logger().info('[P-DET] HTTP 服务已停止')
            except OSError as e:
                parent_self._node.get_logger().error(f'[P-DET] HTTP 端口绑定失败: {e}')
            except Exception as e:
                parent_self._node.get_logger().error(f'[P-DET] HTTP 服务失败: {e}')

        http_thread = threading.Thread(target=serve, daemon=True, name='VisionPHTTPServer')
        http_thread.start()
        parent_self._node.get_logger().info(f'[P-DET] HTTP 线程已启动（thread={http_thread.name}）')

    def _image_callback(self, msg):
        try:
            # 1. 使用完整相机画面
            img_full = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h_full, w_full = img_full.shape[:2]
            crop_start_row = 0
            img_cropped = img_full.copy()
            h_crop, w_crop = img_cropped.shape[:2]

            frame_before = img_cropped.copy()
            cv2.putText(frame_before, 'INPUT (Full frame)',
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

            # 总启动时检测器会早于 Stage3 创建。未启用时不做 BPU 推理，
            # 但仍持续更新 Web 画面，避免网页停在占位图或上一帧。
            if not self.is_inference_active():
                frame_after = img_cropped.copy()
                cv2.putText(
                    frame_after,
                    f'P YOLO DISABLED (phase={getattr(self._node, "phase", 0)})',
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 200, 255), 2,
                )
                with self._lock:
                    self._detected = False
                    self._confidence = 0.0
                    self._bbox = None
                    self._timestamp = time.time()
                    self._offset = 0.0
                    self._fill_ratio = 0.0
                self._publish_visualization(frame_before, frame_after, 'P inference disabled')
                return

            # 2. 预处理
            canvas, scale = self._letterbox(img_cropped, self.input_size)
            nv12 = self._bgr2nv12(canvas)

            # 3. 推理
            t0 = time.perf_counter()
            outs = self.model.forward([nv12])
            self._infer_time_ms = (time.perf_counter() - t0) * 1000

            # 4. 后处理
            # best_p.bin 是纯 YOLO 检测模型，不含 prototype/mask 输出。
            # 6 个输出按 [cls80, reg80, cls40, reg40, cls20, reg20] 排列；
            # 分类和 DFL 回归必须使用同一尺度的网格。
            if len(outs) != 6:
                raise ValueError(f'best_p.bin 输出数量异常: {len(outs)}，期望 6')

            if not self._layout_logged:
                self._node.get_logger().info(
                    '[P-DET] 输出布局: '
                    + ', '.join(f'{i}:{out.buffer.shape}' for i, out in enumerate(outs))
                )
                self._layout_logged = True

            all_bboxes, all_scores = [], []
            for si, s in enumerate(self.strides):
                cls = self._sigmoid(outs[si * 2].buffer.reshape(-1))
                reg_output = outs[si * 2 + 1].buffer
                reg = reg_output.reshape(-1, 64)
                hg, wg = reg_output.shape[1:3]
                sx, sy = np.meshgrid(np.arange(wg)+0.5, np.arange(hg)+0.5, indexing='xy')
                ap = np.stack((sx.ravel(), sy.ravel()), axis=1).astype(np.float32)
                box = self._dfl_decode(reg) * s
                x1 = ap[:, 0] * s - box[:, 0]
                y1 = ap[:, 1] * s - box[:, 1]
                x2 = ap[:, 0] * s + box[:, 2]
                y2 = ap[:, 1] * s + box[:, 3]
                all_bboxes.append(np.column_stack([x1, y1, x2, y2]))
                all_scores.append(cls)

            bboxes = np.concatenate(all_bboxes)
            scores = np.concatenate(all_scores)

            keep = scores > self.conf_thres
            bboxes = bboxes[keep]
            scores = scores[keep]

            # 5. NMS + 可视化
            detected = False
            confidence = 0.0
            best_bbox = None
            offset = 0.0
            fill_ratio = 0.0
            frame_after = img_cropped.copy()

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

                if len(idxs) > 0:
                    best_i = idxs[0]
                    box = bboxes[best_i]
                    confidence = float(scores[best_i])
                    x1 = max(0, min(w_crop-1, box[0]/scale))
                    y1 = max(0, min(h_crop-1, box[1]/scale))
                    x2 = max(0, min(w_crop-1, box[2]/scale))
                    y2 = max(0, min(h_crop-1, box[3]/scale))
                    
                    # 全图坐标（供外部调用）
                    best_bbox = (
                        int(x1), 
                        int(y1), 
                        int(x2), 
                        int(y2)
                    )
                    detected = True

                    # 绘制可视化（裁剪后坐标）
                    cv2.rectangle(frame_after, (int(x1), int(y1)), (int(x2), int(y2)),
                                 (0, 255, 0), 2)
                    cx = int((x1 + x2) / 2)
                    cy = int((y1 + y2) / 2)
                    cv2.circle(frame_after, (cx, cy), 5, (0, 0, 255), -1)
                    cv2.line(frame_after, (cx, cy), (w_crop//2, h_crop//2), (255, 0, 0), 2)

                    offset = cx / (w_crop / 2) - 1.0
                    bbox_area = (x2 - x1) * (y2 - y1)
                    image_area = w_crop * h_crop
                    fill_ratio = bbox_area / image_area if image_area > 0 else 0.0

                    cv2.putText(frame_after, f'P conf={confidence:.2f} off={offset:+.2f} fill={fill_ratio:.2%}',
                               (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX,
                               0.5, (0, 255, 255), 2)

            # 6. 更新共享变量
            with self._lock:
                self._detected = detected
                self._confidence = confidence
                self._bbox = best_bbox
                self._timestamp = time.time()
                self._offset = float(offset)
                self._fill_ratio = float(fill_ratio)

            self._publish_visualization(
                frame_before,
                frame_after,
                f'P Detected: {"YES" if detected else "NO"}',
            )

        except Exception as e:
            if self._frame_count % 60 == 0:
                self._node.get_logger().error(f'[P-DET] 推理失败: {e}')

    def _publish_visualization(self, frame_before, frame_after, status_text):
        now = time.perf_counter()
        self._fps_queue.append(now - self._last_time)
        self._last_time = now
        avg_fps = len(self._fps_queue) / sum(self._fps_queue) if self._fps_queue else 0.0
        cv2.putText(
            frame_after,
            f'OUTPUT (FPS: {avg_fps:.1f} | Infer: {self._infer_time_ms:.1f}ms)',
            (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2,
        )
        cv2.putText(
            frame_after,
            status_text,
            (10, max(55, frame_after.shape[0] - 10)),
            cv2.FONT_HERSHEY_SIMPLEX, 0.7,
            (0, 255, 0) if 'YES' in status_text else (0, 200, 255), 2,
        )

        target_h = 360
        before_resized = cv2.resize(
            frame_before,
            (int(frame_before.shape[1] * target_h / frame_before.shape[0]), target_h),
        )
        after_resized = cv2.resize(
            frame_after,
            (int(frame_after.shape[1] * target_h / frame_after.shape[0]), target_h),
        )
        combined = np.hstack([before_resized, after_resized])
        mid_x = before_resized.shape[1]
        cv2.line(combined, (mid_x, 0), (mid_x, target_h), (255, 255, 255), 3)

        with self._lock:
            self._combined_frame = combined.copy()
            self._frame_count += 1

        if now - self._last_save_time < self._min_frame_interval:
            return
        try:
            # OpenCV 根据扩展名选择编码器；临时文件也必须保留 .jpg 后缀。
            temp_path = f'{self._jpeg_output_path}.tmp.jpg'
            if not cv2.imwrite(temp_path, combined, [cv2.IMWRITE_JPEG_QUALITY, 85]):
                raise RuntimeError('cv2.imwrite returned false')
            os.replace(temp_path, self._jpeg_output_path)
            self._last_save_time = now
            with self._lock:
                self._last_frame_save_time = time.time()
        except Exception as save_err:
            self._node.get_logger().error(f'[P-DET] 保存图像失败: {save_err}')

    def _sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))

    def _dfl_decode(self, reg):
        N = reg.shape[0]
        proj = np.arange(16, dtype=np.float32)
        reg = reg.reshape(N, 4, 16)
        sm = np.exp(reg - reg.max(axis=-1, keepdims=True))
        sm /= sm.sum(axis=-1, keepdims=True)
        return (sm @ proj).reshape(N, 4)

    def _bgr2nv12(self, bgr, w=None, h=None):
        if w is None: w = self.input_size
        if h is None: h = self.input_size
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

    def _letterbox(self, img, target_size=None):
        if target_size is None:
            target_size = self.input_size
        h0, w0 = img.shape[:2]
        scale = min(target_size / h0, target_size / w0)
        nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
        img_resized = cv2.resize(img, (nw, nh))
        canvas = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = img_resized
        return canvas, scale
