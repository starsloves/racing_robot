"""Generic single-class YOLO bbox detector for RDK/BPU models.

This is a lightweight extraction of the Stage3 P detector's bbox decoding
path.  It intentionally does not own navigation logic; callers only enable or
disable inference and read the latest bbox center/fill status.
"""

from __future__ import annotations

import os
import threading
import time
from typing import Optional, Tuple
from http.server import BaseHTTPRequestHandler
from socketserver import ThreadingTCPServer
import json

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from hobot_dnn import pyeasy_dnn as dnn


class YoloBBoxDetector:
    """Single-class YOLO bbox detector.

    Public result tuple:
        detected, confidence, bbox, timestamp, offset, fill_ratio

    bbox is (x1, y1, x2, y2) in source image pixels.
    offset is bbox center relative to image center in [-1, 1], positive right.
    fill_ratio is bbox width / image width, matching the current Stage3 P
    completion convention.
    """

    def __init__(
        self,
        parent_node,
        *,
        model_path: str,
        camera_topic: str = '/aurora/rgb/image_raw',
        camera_info_topic: str = '/aurora/rgb/camera_info',
        target_name: str = 'target',
        conf_thres: float = 0.25,
        iou_thres: float = 0.45,
        input_size: int = 640,
        jpeg_output_path: str = '/tmp/yolo_bbox_latest.jpg',
        raw_output_path: str = '/tmp/yolo_bbox_raw.jpg',
        http_port: int = 8081,
    ) -> None:
        self._node = parent_node
        self.model_path = str(model_path)
        self.camera_topic = str(camera_topic)
        self.camera_info_topic = str(camera_info_topic)
        self.target_name = str(target_name)
        self.conf_thres = float(conf_thres)
        self.iou_thres = float(iou_thres)
        self.input_size = int(input_size)
        self.jpeg_output_path = str(jpeg_output_path)
        self.raw_output_path = str(raw_output_path)
        self.http_port = int(http_port)
        self._http_start_time = time.time()

        self._lock = threading.Lock()
        self._active = False
        self._detected = False
        self._confidence = 0.0
        self._bbox: Optional[Tuple[int, int, int, int]] = None
        self._timestamp = 0.0
        self._offset = 0.0
        self._fill_ratio = 0.0
        self._image_width = 640
        self._image_height = 480
        self._frame_count = 0
        self._last_frame_time = 0.0
        self._layout_logged = False
        self._camera_info = None
        self._latest_frame_id = ''
        self._latest_raw_frame = None
        self._latest_preview_frame = None

        if not os.path.exists(self.model_path):
            raise FileNotFoundError(self.model_path)
        self._node.get_logger().info(f'[YOLO-{self.target_name}] loading model: {self.model_path}')
        self.model = dnn.load(self.model_path)[0]
        self.bridge = CvBridge()
        self.strides = [8, 16, 32]
        self._output_pairs = None

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self._node.create_subscription(Image, self.camera_topic, self._image_cb, qos)
        self._node.create_subscription(CameraInfo, self.camera_info_topic, self._camera_info_cb, qos)
        self._write_placeholder_images()
        self._start_http_server()
        self._node.get_logger().info(
            f'[YOLO-{self.target_name}] ready topic={self.camera_topic} active=false'
        )

    def _write_placeholder_images(self) -> None:
        """Keep every HTTP endpoint usable before the first camera callback."""
        placeholder = np.full((360, 640, 3), 30, dtype=np.uint8)
        cv2.putText(placeholder, f'{self.target_name}: waiting for camera', (40, 165),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 220, 255), 2)
        cv2.putText(placeholder, f'http://0.0.0.0:{self.http_port}', (40, 210),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (180, 180, 180), 1)
        self._save_jpeg(placeholder, self.raw_output_path)
        self._save_jpeg(placeholder, self.jpeg_output_path)
        with self._lock:
            self._latest_raw_frame = placeholder.copy()
            self._latest_preview_frame = placeholder.copy()

    def _stream_frame(self, preview: bool):
        with self._lock:
            frame = self._latest_preview_frame if preview else self._latest_raw_frame
            return None if frame is None else frame.copy()

    def _start_http_server(self) -> None:
        if self.http_port <= 0:
            return

        detector = self
        viewer_path = os.path.abspath(
            os.path.join(os.path.dirname(__file__), '../../../../vision_viewer.html')
        )

        class Handler(BaseHTTPRequestHandler):
            def do_GET(self):
                path = self.path.split('?', 1)[0]
                if path in ('/', '/index.html'):
                    # The single 8081 entry point is the operator dashboard.  Keep
                    # the standalone HTML as the source of truth so opening the
                    # file directly and opening http://board:8081 stay identical.
                    try:
                        with open(viewer_path, 'rb') as viewer_file:
                            body = viewer_file.read()
                    except OSError:
                        body = (
                        '<!doctype html><meta charset="utf-8"><title>Stage1 channel vision</title>'
                        '<style>body{background:#202124;color:#eee;font-family:sans-serif;margin:20px}'
                        '.frames{display:flex;gap:12px;flex-wrap:wrap}.frames figure{margin:0;max-width:48%}'
                        'img{width:100%;border:1px solid #555}#status{margin:10px 0;color:#8f8}</style>'
                        '<h2>Stage1 通道视觉</h2><div id="status">连接中...</div><div class="frames">'
                        '<figure><figcaption>Camera</figcaption><img src="/stream_raw.mjpg"></figure>'
                        '<figure><figcaption>YOLO</figcaption><img src="/stream.mjpg"></figure></div>'
                        '<script>const s=document.getElementById("status");async function h(){try{const r=await fetch("/health?t="+Date.now(),{cache:"no-store"});const d=await r.json();s.textContent="status="+d.status+" | yolo="+(d.inference_active?"on":"standby")+" | frames="+d.frame_count+" | age="+d.frame_age_sec+"s"}catch(e){s.textContent="HTTP waiting: "+e}}setInterval(h,500);h()</script>'
                        ).encode('utf-8')
                    self._send_bytes(body, 'text/html; charset=utf-8')
                    return
                if path == '/health':
                    with detector._lock:
                        data = {
                            'status': 'ok',
                            'stage': 'stage1_channel_yolo',
                            'inference_active': detector._active,
                            'detected': detector._detected,
                            'frame_count': detector._frame_count,
                            'frame_age_sec': (
                                round(time.time() - detector._last_frame_time, 2)
                                if detector._last_frame_time > 0.0 else None
                            ),
                            'uptime_sec': time.time() - detector._http_start_time,
                        }
                    self._send_bytes(json.dumps(data).encode('utf-8'), 'application/json')
                    return
                if path in ('/stream.mjpg', '/stream_raw.mjpg'):
                    preview = path == '/stream.mjpg'
                    self.send_response(200)
                    self.send_header('Content-Type', 'multipart/x-mixed-replace; boundary=frame')
                    self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                    self.send_header('Access-Control-Allow-Origin', '*')
                    self.end_headers()
                    try:
                        while True:
                            frame = detector._stream_frame(preview)
                            if frame is not None:
                                ok, encoded = cv2.imencode(
                                    '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80]
                                )
                                if ok:
                                    content = encoded.tobytes()
                                    self.wfile.write(b'--frame\r\nContent-Type: image/jpeg\r\n')
                                    self.wfile.write(
                                        f'Content-Length: {len(content)}\r\n\r\n'.encode('ascii')
                                    )
                                    self.wfile.write(content)
                                    self.wfile.write(b'\r\n')
                            time.sleep(0.10)
                    except (BrokenPipeError, ConnectionResetError):
                        pass
                    return
                image_paths = {
                    '/channel_raw.jpg': detector.raw_output_path,
                    '/channel_yolo.jpg': detector.jpeg_output_path,
                    '/vision_latest.jpg': detector.jpeg_output_path,
                }
                image_path = image_paths.get(path)
                if image_path is not None:
                    try:
                        with open(image_path, 'rb') as image_file:
                            self._send_bytes(image_file.read(), 'image/jpeg')
                    except FileNotFoundError:
                        self.send_error(404, 'image not ready')
                    return
                self.send_error(404)

            def _send_bytes(self, content, content_type):
                self.send_response(200)
                self.send_header('Content-Type', content_type)
                self.send_header('Content-Length', str(len(content)))
                self.send_header('Cache-Control', 'no-cache, no-store, must-revalidate')
                self.send_header('Access-Control-Allow-Origin', '*')
                self.end_headers()
                self.wfile.write(content)

            def log_message(self, *_args):
                pass

        def serve():
            try:
                ThreadingTCPServer.allow_reuse_address = True
                with ThreadingTCPServer(('0.0.0.0', self.http_port), Handler) as server:
                    server.daemon_threads = True
                    detector._node.get_logger().info(
                        f'[YOLO-{detector.target_name}] HTTP_READY port={detector.http_port}'
                    )
                    if hasattr(detector._node, 'log'):
                        detector._node.log.startup(
                            f'CHANNEL_HTTP_READY port={detector.http_port} '
                            f'raw=/channel_raw.jpg result=/channel_yolo.jpg'
                        )
                    server.serve_forever()
            except OSError as exc:
                detector._node.get_logger().error(
                    f'[YOLO-{detector.target_name}] HTTP_ERROR port={detector.http_port}: {exc}'
                )
                if hasattr(detector._node, 'log'):
                    detector._node.log.warn(
                        'CHANNEL_HTTP', f'HTTP_ERROR port={detector.http_port}: {exc}'
                    )

        threading.Thread(target=serve, daemon=True, name='Stage1ChannelHTTP').start()

    def set_inference_active(self, active: bool) -> None:
        active = bool(active)
        with self._lock:
            previous = self._active
            self._active = active
            if not active:
                self._detected = False
                self._confidence = 0.0
                self._bbox = None
                self._offset = 0.0
                self._fill_ratio = 0.0
                self._timestamp = time.time()
        if previous != active:
            state = 'enabled' if active else 'disabled'
            self._node.get_logger().info(f'[YOLO-{self.target_name}] inference {state}')

    def is_inference_active(self) -> bool:
        with self._lock:
            return bool(self._active)

    def get_detection(self):
        with self._lock:
            return (
                bool(self._detected),
                float(self._confidence),
                self._bbox,
                float(self._timestamp),
                float(self._offset),
                float(self._fill_ratio),
            )

    def get_detection_geometry(self):
        """Return the latest bbox and pinhole camera geometry for projection."""
        with self._lock:
            return {
                'detected': bool(self._detected),
                'bbox': self._bbox,
                'image_width': int(self._image_width),
                'image_height': int(self._image_height),
                'frame_id': str(self._latest_frame_id),
                'camera_info': self._camera_info,
                'timestamp': float(self._timestamp),
            }

    def _camera_info_cb(self, msg: CameraInfo) -> None:
        k = tuple(float(value) for value in msg.k)
        if len(k) != 9 or k[0] <= 0.0 or k[4] <= 0.0:
            return
        with self._lock:
            self._camera_info = {
                'fx': k[0],
                'fy': k[4],
                'cx': k[2],
                'cy': k[5],
                'frame_id': str(msg.header.frame_id),
            }

    def _image_cb(self, msg: Image) -> None:
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = img.shape[:2]
            self._save_jpeg(img, self.raw_output_path)
            with self._lock:
                self._image_width = int(w)
                self._image_height = int(h)
                self._latest_frame_id = str(msg.header.frame_id)
                active = bool(self._active)
                self._latest_raw_frame = img.copy()
                self._last_frame_time = time.time()
                self._frame_count += 1
            if not active:
                preview = img.copy()
                cv2.putText(preview, f'{self.target_name} YOLO standby', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 255), 2)
                self._save_preview(preview)
                with self._lock:
                    self._latest_preview_frame = preview
                return

            canvas, scale = self._letterbox(img, self.input_size)
            nv12 = self._bgr2nv12(canvas)
            outs = self.model.forward([nv12])
            if len(outs) != 6:
                raise ValueError(f'YOLO output count {len(outs)} != 6')
            if not self._layout_logged:
                self._node.get_logger().info(
                    f'[YOLO-{self.target_name}] layout: '
                    + ', '.join(f'{i}:{out.buffer.shape}' for i, out in enumerate(outs))
                )
                self._layout_logged = True

            output_pairs = self._resolve_output_pairs(outs)
            boxes_all, scores_all = [], []
            for si, stride in enumerate(self.strides):
                reg_output, cls_output = output_pairs[si]
                reg = reg_output.reshape(-1, 64)
                cls = self._score_from_cls_output(cls_output, reg.shape[0])
                hg, wg = reg_output.shape[1:3]
                sx, sy = np.meshgrid(np.arange(wg) + 0.5, np.arange(hg) + 0.5, indexing='xy')
                anchors = np.stack((sx.ravel(), sy.ravel()), axis=1).astype(np.float32)
                decoded = self._dfl_decode(reg) * stride
                x1 = anchors[:, 0] * stride - decoded[:, 0]
                y1 = anchors[:, 1] * stride - decoded[:, 1]
                x2 = anchors[:, 0] * stride + decoded[:, 2]
                y2 = anchors[:, 1] * stride + decoded[:, 3]
                boxes_all.append(np.column_stack([x1, y1, x2, y2]))
                scores_all.append(cls)

            bboxes = np.concatenate(boxes_all)
            scores = np.concatenate(scores_all)
            keep = scores > self.conf_thres
            bboxes = bboxes[keep]
            scores = scores[keep]

            detected = False
            confidence = 0.0
            best_bbox = None
            offset = 0.0
            fill_ratio = 0.0
            vis = img.copy()

            if len(bboxes) > 0:
                xywh = np.zeros_like(bboxes)
                xywh[:, 0] = bboxes[:, 0]
                xywh[:, 1] = bboxes[:, 1]
                xywh[:, 2] = bboxes[:, 2] - bboxes[:, 0]
                xywh[:, 3] = bboxes[:, 3] - bboxes[:, 1]
                idxs = cv2.dnn.NMSBoxes(
                    xywh.tolist(), scores.tolist(), self.conf_thres, self.iou_thres
                )
                if isinstance(idxs, np.ndarray):
                    idxs = idxs.flatten()
                elif isinstance(idxs, (list, tuple)):
                    idxs = np.array(idxs).flatten()
                else:
                    idxs = np.array([], dtype=int)
                if len(idxs) > 0:
                    best_i = int(idxs[0])
                    box = bboxes[best_i]
                    confidence = float(scores[best_i])
                    x1 = int(max(0, min(w - 1, box[0] / scale)))
                    y1 = int(max(0, min(h - 1, box[1] / scale)))
                    x2 = int(max(0, min(w - 1, box[2] / scale)))
                    y2 = int(max(0, min(h - 1, box[3] / scale)))
                    best_bbox = (x1, y1, x2, y2)
                    detected = True
                    cx = 0.5 * (x1 + x2)
                    offset = (cx - w * 0.5) / max(w * 0.5, 1.0)
                    fill_ratio = max(0.0, (x2 - x1) / max(float(w), 1.0))
                    cv2.rectangle(vis, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.circle(vis, (int(cx), int(0.5 * (y1 + y2))), 5, (0, 0, 255), -1)

            cv2.putText(
                vis,
                f'{self.target_name} det={detected} conf={confidence:.2f} off={offset:+.2f} fill={fill_ratio:.2%}',
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )
            self._save_preview(vis)
            with self._lock:
                self._detected = detected
                self._confidence = confidence
                self._bbox = best_bbox
                self._timestamp = time.time()
                self._offset = offset
                self._fill_ratio = fill_ratio
                self._latest_preview_frame = vis
        except Exception as exc:
            if self._frame_count % 60 == 0:
                self._node.get_logger().error(f'[YOLO-{self.target_name}] inference failed: {exc}')

    @staticmethod
    def _save_jpeg(img, path: str) -> None:
        if not path:
            return
        try:
            tmp = f'{path}.tmp.jpg'
            cv2.imwrite(tmp, img, [cv2.IMWRITE_JPEG_QUALITY, 80])
            os.replace(tmp, path)
        except Exception:
            pass

    def _save_preview(self, img) -> None:
        self._save_jpeg(img, self.jpeg_output_path)

    @staticmethod
    def _sigmoid(x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))

    def _resolve_output_pairs(self, outs):
        """Return [(reg, cls), ...] for common Horizon YOLO 6-output layouts.

        Some converted models export [cls80, reg80, ...], while
        best_rdk_tongdao.bin exports [reg80, cls80, ...].  Resolve by tensor
        channel count instead of hardcoding the order, so bbox decode and score
        vectors always have one item per grid anchor.
        """
        if self._output_pairs is not None:
            return [(outs[ri].buffer, outs[ci].buffer) for ri, ci in self._output_pairs]

        pairs = []
        for si in range(3):
            a_idx = si * 2
            b_idx = a_idx + 1
            a = outs[a_idx].buffer
            b = outs[b_idx].buffer
            a_is_reg = self._looks_like_reg_output(a)
            b_is_reg = self._looks_like_reg_output(b)
            if a_is_reg and not b_is_reg:
                pairs.append((a_idx, b_idx))
            elif b_is_reg and not a_is_reg:
                pairs.append((b_idx, a_idx))
            else:
                raise ValueError(
                    f'cannot resolve YOLO output pair {a_idx}/{b_idx}: '
                    f'shapes {a.shape} / {b.shape}'
                )
        self._output_pairs = pairs
        self._node.get_logger().info(
            f'[YOLO-{self.target_name}] resolved output pairs '
            + ', '.join(f's{si}:reg{ri}/cls{ci}' for si, (ri, ci) in enumerate(pairs))
        )
        return [(outs[ri].buffer, outs[ci].buffer) for ri, ci in pairs]

    @staticmethod
    def _looks_like_reg_output(arr) -> bool:
        shape = tuple(arr.shape)
        return len(shape) >= 3 and 64 in shape

    def _score_from_cls_output(self, cls_output, anchor_count: int):
        raw = np.asarray(cls_output)
        flat = raw.reshape(-1)
        if flat.size == anchor_count:
            return self._sigmoid(flat)

        # Multi-class/class-channel variants: reshape to anchors x classes and
        # use the best class confidence for the single target-centering use case.
        if anchor_count > 0 and flat.size % anchor_count == 0:
            cls = flat.reshape(anchor_count, flat.size // anchor_count)
            return self._sigmoid(cls).max(axis=1)

        raise ValueError(
            f'cls output shape {raw.shape} cannot match {anchor_count} anchors'
        )

    @staticmethod
    def _dfl_decode(reg):
        n = reg.shape[0]
        proj = np.arange(16, dtype=np.float32)
        reg = reg.reshape(n, 4, 16)
        sm = np.exp(reg - reg.max(axis=-1, keepdims=True))
        sm /= sm.sum(axis=-1, keepdims=True)
        return (sm @ proj).reshape(n, 4)

    def _letterbox(self, img, target_size):
        h0, w0 = img.shape[:2]
        scale = min(target_size / h0, target_size / w0)
        nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
        resized = cv2.resize(img, (nw, nh))
        canvas = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = resized
        return canvas, scale

    @staticmethod
    def _bgr2nv12(bgr):
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
