#!/usr/bin/env python3
"""
batch_test_model.py — 新模型(YOLOv8-Det, NV12输入)批量推理测试
"""
import os, sys, random, time, glob
import cv2
import numpy as np

RAW_DIR = os.path.expanduser('~/dev_ws/log/debug/vision_preview/raw')
OUT_DIR = os.path.expanduser('~/dev_ws/log/debug/vision_preview/batch_test')
MODEL_PATH = os.path.expanduser(
    '~/dev_ws/src/racing/racing_stage2_param_test/models/saidao_seg_model_quant.bin')
SAMPLE_N = 100
CONF_THR = 0.5

def bgr2nv12(bgr, w=640, h=640):
    resized = cv2.resize(bgr, (w, h))
    bgr_planar = resized.transpose((2, 0, 1)).flatten()
    nv12 = np.empty(h * w * 3 // 2, dtype=np.uint8)
    r, g, b = bgr_planar[0::3], bgr_planar[1::3], bgr_planar[2::3]
    y = np.clip((77*r + 150*g + 29*b + 128) // 256 + 16, 16, 235).astype(np.uint8)
    nv12[:h*w] = y.ravel()
    u = np.clip((-43*r - 85*g + 128*b + 128) // 256 + 128, 16, 240).astype(np.uint8)
    v = np.clip((128*r - 107*g - 21*b + 128) // 256 + 128, 16, 240).astype(np.uint8)
    u_sub = u.reshape(h, w)[::2, ::2].ravel()
    v_sub = v.reshape(h, w)[::2, ::2].ravel()
    uv = np.empty(h * w // 2, dtype=np.uint8)
    uv[0::2] = u_sub
    uv[1::2] = v_sub
    nv12[h*w:] = uv
    return nv12

def decode_yolov8_det(det_raw, img_w=640, img_h=640, conf_thr=0.5):
    """(1,6,8400,1) -> list of [x1,y1,x2,y2,conf,cls]"""
    raw = det_raw.reshape(8400, 6)
    cx, cy = raw[:, 0], raw[:, 1]
    w, h_ = raw[:, 2], raw[:, 3]
    obj = 1.0 / (1.0 + np.exp(-np.clip(raw[:, 4], -20, 20)))
    cls = 1.0 / (1.0 + np.exp(-np.clip(raw[:, 5], -20, 20)))
    conf = obj * cls
    mask = conf > conf_thr
    if not mask.any():
        return []
    x1 = np.clip(cx[mask] - w[mask] / 2, 0, img_w)
    y1 = np.clip(cy[mask] - h_[mask] / 2, 0, img_h)
    x2 = np.clip(cx[mask] + w[mask] / 2, 0, img_w)
    y2 = np.clip(cy[mask] + h_[mask] / 2, 0, img_h)
    boxes = []
    for i in range(len(x1)):
        boxes.append([x1[i], y1[i], x2[i], y2[i], conf[mask][i], 0])
    # NMS
    boxes.sort(key=lambda b: -b[4])
    keep = []
    for b in boxes:
        ok = True
        for k in keep:
            xi1 = max(b[0], k[0]); yi1 = max(b[1], k[1])
            xi2 = min(b[2], k[2]); yi2 = min(b[3], k[3])
            inter = max(0, xi2-xi1) * max(0, yi2-yi1)
            area_b = (b[2]-b[0]) * (b[3]-b[1])
            area_k = (k[2]-k[0]) * (k[3]-k[1])
            iou = inter / (area_b + area_k - inter + 1e-6)
            if iou > 0.5:
                ok = False
                break
        if ok:
            keep.append(b)
    return keep

def draw_detections(bgr, dets, img_w=640):
    h, w = bgr.shape[:2]
    scale_x = w / img_w
    scale_y = h / img_w
    img = bgr.copy()
    best_box = None
    for d in dets:
        x1 = int(d[0] * scale_x); y1 = int(d[1] * scale_y)
        x2 = int(d[2] * scale_x); y2 = int(d[3] * scale_y)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.putText(img, f'{d[4]:.2f}', (x1, y1-5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        best_box = d  # last one (highest conf due to sorting)
    if best_box is not None:
        bcx = int((best_box[0] + best_box[2]) / 2 * scale_x)
        bcy = int((best_box[1] + best_box[3]) / 2 * scale_y)
        cv2.circle(img, (bcx, bcy), 5, (0, 0, 255), -1)
        cv2.line(img, (bcx, bcy), (w//2, bcy), (255, 0, 0), 2)
        offset = (bcx - w//2) / (w//2)
        cv2.putText(img, f'offset: {offset:+.3f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(img, f'dets: {len(dets)} conf:{best_box[4]:.2f}', (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
    else:
        cv2.putText(img, 'NO DETECTION', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    return img, best_box

def main():
    print('=' * 60)
    print('batch_test_model.py — YOLOv8-Det (NV12) 批量测试')
    print('=' * 60)
    print(f'Model:  {MODEL_PATH}')
    print(f'Input:  {RAW_DIR}')
    print(f'Output: {OUT_DIR}')
    print(f'Sample: {SAMPLE_N} 张')
    print()

    print('[1/4] 加载 BPU 模型...', end=' ')
    sys.stdout.flush()
    from hobot_dnn import pyeasy_dnn as dnn
    models = dnn.load(MODEL_PATH)
    model = models[0]
    print(f'OK — {model.name}')

    print('[2/4] 扫描 raw 图片...', end=' ')
    jpgs = sorted(glob.glob(os.path.join(RAW_DIR, '*.jpg')))
    samples = random.Random(42).sample(jpgs, min(SAMPLE_N, len(jpgs)))
    print(f'{len(jpgs)} 张, 抽 {len(samples)} 张')

    print('[3/4] 创建输出目录...', end=' ')
    os.makedirs(OUT_DIR, exist_ok=True)
    print('OK')

    print('[4/4] 批量推理中...')
    times, n_dets, offsets = [], [], []
    for idx, path in enumerate(samples):
        img = cv2.imread(path)
        if img is None:
            continue
        nv12 = bgr2nv12(img)
        t0 = time.perf_counter()
        try:
            outs = model.forward([nv12])
        except Exception as e:
            print(f'\nFAIL: {path}: {e}')
            continue
        dt = (time.perf_counter() - t0) * 1000.0
        times.append(dt)
        dets = decode_yolov8_det(outs[0].buffer, conf_thr=CONF_THR)
        n_dets.append(len(dets))
        overlay, best = draw_detections(img, dets)
        if best is not None:
            bcx = (best[0] + best[2]) / 2
            offset = (bcx / 320.0 - 1.0)
            offsets.append(offset)
        out_name = f'{idx:04d}_{os.path.basename(path)}'
        cv2.imwrite(os.path.join(OUT_DIR, out_name), overlay)
        if (idx + 1) % 10 == 0:
            print(f'{idx+1} ', end='', flush=True)
    print()

    n = len(times)
    print()
    print('=' * 60)
    print('  测试报告 (YOLOv8-Det)')
    print('=' * 60)
    if n == 0:
        print('  无有效推理结果！')
        return
    print(f'  处理图片:          {n} 张')
    print(f'  推理时间(ms):')
    print(f'    平均:            {np.mean(times):.1f} ± {np.std(times):.1f}')
    print(f'    中位数:          {np.median(times):.1f}')
    print(f'    范围:            [{np.min(times):.1f}, {np.max(times):.1f}]')
    print(f'  检测框数:')
    print(f'    平均:            {np.mean(n_dets):.1f}')
    print(f'    有检测图片:      {sum(1 for d in n_dets if d > 0)}/{n} ({sum(1 for d in n_dets if d > 0)/n*100:.1f}%)')
    if offsets:
        print(f'  中心偏移:')
        print(f'    平均:            {np.mean(offsets):+.4f} ± {np.std(offsets):.4f}')
        print(f'    范围:            [{np.min(offsets):+.4f}, {np.max(offsets):+.4f}]')
    print(f'  输出目录:          {OUT_DIR}')
    print('=' * 60)

if __name__ == '__main__':
    main()