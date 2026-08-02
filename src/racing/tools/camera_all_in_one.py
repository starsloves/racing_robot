#!/usr/bin/env python3
"""
一键启动：相机驱动 + 推理 + Web 服务
运行：python3 camera_all_in_one.py
访问：http://<板子IP>:8080
"""
import os
import sys
import time
import signal
import subprocess
import threading
import argparse
from collections import deque
from io import BytesIO

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from hobot_dnn import pyeasy_dnn as dnn
from flask import Flask, Response, render_template_string


# ============================================================================
# 全局变量
# ============================================================================
camera_process = None  # 相机驱动子进程
inference_node = None  # 推理节点


# ============================================================================
# ROS 2 推理节点
# ============================================================================
class CameraInferenceNode(Node):
    """订阅相机 topic，推理，缓存结果图"""
    
    def __init__(self, model_path, conf_thres=0.25, iou_thres=0.45, crop_ratio=0.4):
        super().__init__('camera_all_in_one')
        
        self.conf_thres = conf_thres
        self.iou_thres = iou_thres
        self.crop_ratio = crop_ratio
        
        # 加载模型
        self.get_logger().info(f'[推理] 加载模型: {os.path.basename(model_path)}')
        models = dnn.load(model_path)
        self.model = models[0]
        self.input_size = 640
        self.REG_MAX = 16
        self.strides = [8, 16, 32]
        
        # ROS 订阅
        self.bridge = CvBridge()
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(
            Image,
            '/aurora/rgb/image_raw',
            self.image_callback,
            qos
        )
        
        # 共享缓存
        self.lock = threading.Lock()
        self.combined_frame = None
        self.fps_queue = deque(maxlen=30)
        self.last_time = time.perf_counter()
        self.frame_count = 0
        self.det_count = 0
        self.infer_time_ms = 0.0
        self.camera_connected = False
        
        # 定时检查相机连接
        self.create_timer(2.0, self.check_camera_connection)
        
        self.get_logger().info('[推理] 节点就绪，等待相机数据...')
    
    def check_camera_connection(self):
        """检查相机连接状态"""
        if not self.camera_connected and self.frame_count == 0:
            self.get_logger().warn('[推理] 等待相机 topic 数据...', throttle_duration_sec=10.0)
    
    def sigmoid(self, x):
        return 1.0 / (1.0 + np.exp(-np.clip(x, -20, 20)))
    
    def dfl_decode(self, reg):
        N = reg.shape[0]
        proj = np.arange(16, dtype=np.float32)
        reg = reg.reshape(N, 4, 16)
        sm = np.exp(reg - reg.max(axis=-1, keepdims=True))
        sm /= sm.sum(axis=-1, keepdims=True)
        return (sm @ proj).reshape(N, 4)
    
    def bgr2nv12(self, bgr, w=640, h=640):
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
    
    def letterbox(self, img, target_size=640):
        h0, w0 = img.shape[:2]
        scale = min(target_size / h0, target_size / w0)
        nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
        img_resized = cv2.resize(img, (nw, nh))
        canvas = np.full((target_size, target_size, 3), 114, dtype=np.uint8)
        canvas[:nh, :nw] = img_resized
        return canvas, scale
    
    def image_callback(self, msg):
        """ROS 图像回调"""
        try:
            if not self.camera_connected:
                self.camera_connected = True
                self.get_logger().info('[推理] 相机连接成功！开始推理...')
            
            # 1. 解码
            img_full = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h_full, w_full = img_full.shape[:2]
            
            # 2. 裁剪下方
            crop_start_row = int(h_full * (1 - self.crop_ratio))
            img_cropped = img_full[crop_start_row:, :].copy()
            h_crop, w_crop = img_cropped.shape[:2]
            
            frame_before = img_cropped.copy()
            cv2.putText(frame_before, f'INPUT (Bottom {int(self.crop_ratio*100)}%)', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            
            # 3. 预处理
            canvas, scale = self.letterbox(img_cropped, self.input_size)
            nv12 = self.bgr2nv12(canvas)
            
            # 4. 推理
            t0 = time.perf_counter()
            outs = self.model.forward([nv12])
            self.infer_time_ms = (time.perf_counter() - t0) * 1000
            
            # 5. 后处理
            all_bboxes, all_scores, all_masks_coeff = [], [], []
            for si, s in enumerate(self.strides):
                reg = outs[si*3].buffer.reshape(-1, 64)
                cls = self.sigmoid(outs[si*3+1].buffer.reshape(-1, 1))
                coeff = outs[si*3+2].buffer.reshape(-1, 32)
                hg, wg = outs[si*3].buffer.shape[1:3]
                sx, sy = np.meshgrid(np.arange(wg)+0.5, np.arange(hg)+0.5, indexing='xy')
                ap = np.stack((sx.ravel(), sy.ravel()), axis=1).astype(np.float32)
                box = self.dfl_decode(reg) * s
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
            
            # 6. 可视化
            frame_after = img_cropped.copy()
            self.det_count = 0
            
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
                
                for i in idxs:
                    box = bboxes[i]
                    x1 = max(0, min(w_crop-1, box[0]/scale))
                    y1 = max(0, min(h_crop-1, box[1]/scale))
                    x2 = max(0, min(w_crop-1, box[2]/scale))
                    y2 = max(0, min(h_crop-1, box[3]/scale))
                    
                    mc = masks_coeff[i]
                    mask_raw = mc @ protos_2d
                    mask_sig = self.sigmoid(mask_raw.reshape(proto_h, proto_w))
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
                    self.det_count += 1
            
            # FPS 统计
            now = time.perf_counter()
            self.fps_queue.append(now - self.last_time)
            self.last_time = now
            avg_fps = len(self.fps_queue) / sum(self.fps_queue) if self.fps_queue else 0
            
            cv2.putText(frame_after, f'OUTPUT (FPS: {avg_fps:.1f} | Infer: {self.infer_time_ms:.1f}ms)', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(frame_after, f'Detections: {self.det_count}', 
                       (10, h_crop-10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            # 拼接
            target_h = max(frame_before.shape[0], frame_after.shape[0])
            before_resized = cv2.resize(frame_before, 
                                       (int(frame_before.shape[1] * target_h / frame_before.shape[0]), target_h))
            after_resized = cv2.resize(frame_after, 
                                      (int(frame_after.shape[1] * target_h / frame_after.shape[0]), target_h))
            combined = np.hstack([before_resized, after_resized])
            
            mid_x = before_resized.shape[1]
            cv2.line(combined, (mid_x, 0), (mid_x, target_h), (255, 255, 255), 3)
            
            with self.lock:
                self.combined_frame = combined.copy()
                self.frame_count += 1
            
        except Exception as e:
            self.get_logger().error(f'[推理] 失败: {e}')
    
    def get_frame(self):
        """获取最新帧"""
        with self.lock:
            if self.combined_frame is not None:
                return self.combined_frame.copy()
            else:
                placeholder = np.zeros((480, 960, 3), dtype=np.uint8)
                cv2.putText(placeholder, 'Waiting for camera...', (300, 220),
                           cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
                cv2.putText(placeholder, 'Camera driver starting...', (300, 260),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.8, (200, 200, 200), 1)
                return placeholder


# ============================================================================
# Flask Web 服务
# ============================================================================
app = Flask(__name__)

HTML_TEMPLATE = """
<!DOCTYPE html>
<html>
<head>
    <title>实时相机推理 (All-in-One)</title>
    <meta charset="utf-8">
    <meta name="viewport" content="width=device-width, initial-scale=1">
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
        <h1>🚗 实时相机推理 (All-in-One)</h1>
        <div class="subtitle"><span class="status">LIVE</span> 相机 + 推理 + Web 一体化服务</div>
        <img src="/video_feed" alt="实时推理画面">
        <div class="info">
            <p><strong>功能说明：</strong></p>
            <p>• 左侧：裁剪后输入（下方 40%）</p>
            <p>• 右侧：检测结果（绿色 mask + bbox + 中心点 + 偏移）</p>
            <p>• 自动启动相机驱动 + BPU 推理 + Web 流传输</p>
            <p><strong>模型：</strong> bset.bin (YOLOv8-Seg)</p>
        </div>
    </div>
</body>
</html>
"""

@app.route('/')
def index():
    return render_template_string(HTML_TEMPLATE)

@app.route('/video_feed')
def video_feed():
    def generate():
        while True:
            if inference_node is None:
                time.sleep(0.1)
                continue
            
            frame = inference_node.get_frame()
            if frame is None:
                time.sleep(0.1)
                continue
            
            ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ret:
                continue
            
            frame_bytes = buffer.tobytes()
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.03)
    
    return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')


# ============================================================================
# 相机驱动管理
# ============================================================================
def start_camera_driver():
    """启动相机驱动子进程"""
    global camera_process
    
    print('[相机] 启动 Aurora 930 驱动...')
    
    # 构建 ROS 2 launch 命令
    cmd = [
        'bash', '-c',
        'source /opt/ros/humble/setup.bash && '
        'ros2 launch deptrum-ros-driver-aurora930 aurora930_launch.py'
    ]
    
    try:
        camera_process = subprocess.Popen(
            cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            preexec_fn=os.setsid  # 新进程组，便于清理
        )
        print('[相机] 驱动启动成功（PID: {}）'.format(camera_process.pid))
        time.sleep(3)  # 等待相机初始化
        return True
    except Exception as e:
        print(f'[相机] 启动失败: {e}')
        return False


def stop_camera_driver():
    """停止相机驱动"""
    global camera_process
    if camera_process is not None:
        print('[相机] 停止驱动...')
        try:
            os.killpg(os.getpgid(camera_process.pid), signal.SIGTERM)
            camera_process.wait(timeout=5)
        except Exception as e:
            print(f'[相机] 停止失败: {e}')
        camera_process = None


def signal_handler(sig, frame):
    """Ctrl+C 信号处理"""
    print('\n\n收到退出信号，清理资源...')
    stop_camera_driver()
    if inference_node is not None:
        inference_node.destroy_node()

    rclpy.shutdown()
    print('已退出。')
    sys.exit(0)


# ============================================================================
# 主程序
# ============================================================================
def main():
    global inference_node
    
    parser = argparse.ArgumentParser(description='一键启动：相机 + 推理 + Web')
    parser.add_argument('--model', type=str, 
                       default=os.path.expanduser('~/dev_ws/src/racing/racing_stage2/models/bset.bin'))
    parser.add_argument('--conf', type=float, default=0.25)
    parser.add_argument('--iou', type=float, default=0.45)
    parser.add_argument('--crop', type=float, default=0.4)
    parser.add_argument('--port', type=int, default=8080)
    parser.add_argument('--no-camera', action='store_true', help='不启动相机驱动（手动启动）')
    args = parser.parse_args()
    
    # 注册信号处理
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)
    
    print('\n' + '='*70)
    print('  🚀 All-in-One 实时相机推理')
    print('='*70)
    
    # 1. 启动相机驱动
    if not args.no_camera:
        if not start_camera_driver():
            print('\n❌ 相机驱动启动失败！')
            print('提示：可以手动启动相机后，用 --no-camera 参数运行本脚本')
            sys.exit(1)
    else:
        print('[相机] 跳过自动启动（需手动启动相机驱动）')
    
    # 2. 初始化 ROS 节点
    print('[推理] 初始化 ROS 2...')
    rclpy.init()
    inference_node = CameraInferenceNode(args.model, args.conf, args.iou, args.crop)
    
    def ros_spin():
        rclpy.spin(inference_node)
    
    ros_thread = threading.Thread(target=ros_spin, daemon=True)
    ros_thread.start()
    
    # 3. 启动 Web 服务
    print(f'\n{"="*70}')
    print(f'  ✅ 所有服务已启动')
    print(f'{"="*70}')
    print(f'  🌐 浏览器访问: http://0.0.0.0:{args.port}')
    print(f'  📷 相机 topic: /aurora/rgb/image_raw')
    print(f'  🤖 模型: {os.path.basename(args.model)}')
    print(f'  ✂️  裁剪: 下方 {int(args.crop*100)}%')
    print(f'  按 Ctrl+C 退出所有服务')
    print(f'{"="*70}\n')
    
    try:
        app.run(host='0.0.0.0', port=args.port, debug=False, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        signal_handler(None, None)


if __name__ == '__main__':
    main()
