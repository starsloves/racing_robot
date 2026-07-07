#!/usr/bin/env python3
"""远端 BPU 推理 — 所有 raw 图片 → viz_bin（取最大框）"""
import os, sys, glob, time, cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

def bgr2nv12(bgr, w=640, h=640):
    resized = cv2.resize(bgr, (w, h))
    bgr_planar = resized.transpose((2, 0, 1)).flatten()
    r, g, b = bgr_planar[0::3], bgr_planar[1::3], bgr_planar[2::3]
    nv12 = np.empty(h * w * 3 // 2, dtype=np.uint8)
    nv12[:h*w] = np.clip((77*r + 150*g + 29*b + 128)//256 + 16, 16, 235).astype(np.uint8)
    nv12[h*w:][0::2] = np.clip((-43*r - 85*g + 128*b + 128)//256 + 128, 16, 240).astype(np.uint8).reshape(h,w)[::2,::2].ravel()
    nv12[h*w:][1::2] = np.clip((128*r - 107*g - 21*b + 128)//256 + 128, 16, 240).astype(np.uint8).reshape(h,w)[::2,::2].ravel()
    return nv12

MODEL = os.path.expanduser('~/dev_ws/src/racing/racing_stage2_param_test/models/saidao_seg_model_quant.bin.NEW')
RAW = os.path.expanduser('~/dev_ws/log/debug/vision_preview/raw')
OUT = os.path.expanduser('~/dev_ws/log/debug/vision_preview/viz_bin')
os.makedirs(OUT, exist_ok=True)

models = dnn.load(MODEL)
m = models[0]
print(f'Model: {m.name}')
print(f'RAW:   {RAW}')
print(f'OUT:   {OUT}')

jpgs = sorted(glob.glob(os.path.join(RAW, '*.jpg')))
n = len(jpgs)
print(f'Total: {n} images')
print()

t_start = time.perf_counter()
for idx, path in enumerate(jpgs):
    name = os.path.basename(path)
    img = cv2.imread(path)
    if img is None:
        continue
    hi, wi = img.shape[:2]
    nv12 = bgr2nv12(img)
    outs = m.forward([nv12])
    det = outs[0].buffer.reshape(8400, 6)
    cx, cy, bw, bh = det[:, 0], det[:, 1], det[:, 2], det[:, 3]
    valid = (cx > 0) & (cx < 640) & (cy > 0) & (cy < 640) & (bw > 0) & (bh > 0)
    if valid.any():
        areas = bw[valid] * bh[valid]
        best = np.argmax(areas)
        vi = np.where(valid)[0][best]
        cxx = cx[vi] * wi / 640
        cyy = cy[vi] * hi / 640
        ww = bw[vi] * wi / 640
        hh = bh[vi] * hi / 640
        x1 = int(cxx - ww/2); y1 = int(cyy - hh/2)
        x2 = int(cxx + ww/2); y2 = int(cyy + hh/2)
        cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(img, (int(cxx), int(cyy)), 5, (0, 0, 255), -1)
        cv2.line(img, (int(cxx), int(cyy)), (wi//2, hi//2), (255, 0, 0), 2)
        cv2.putText(img, f'area={areas[best]:.0f} off={cxx/wi*2-1:+.3f}', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    else:
        cv2.putText(img, 'NO BOX', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
    cv2.imwrite(os.path.join(OUT, name), img)
    elapsed = time.perf_counter() - t_start
    avg = elapsed / (idx + 1)
    eta = avg * (n - idx - 1)
    if (idx + 1) % 50 == 0 or idx == 0:
        print(f'  [{idx+1}/{n}] avg:{avg*1000:.0f}ms/img ETA:{eta/60:.0f}min')

print(f'\nDone! {(time.perf_counter()-t_start)/60:.1f}min → {OUT}')