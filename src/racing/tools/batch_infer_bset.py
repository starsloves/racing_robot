#!/usr/bin/env python3
"""远端 bset.bin — YOLOv8-Seg 推理（letterbox + NMS）"""
import os, sys, glob, time, cv2
import numpy as np
from hobot_dnn import pyeasy_dnn as dnn

MODEL = os.path.expanduser('~/dev_ws/src/racing/racing_stage2_param_test/models/bset.bin')
RAW = os.path.expanduser('~/dev_ws/log/debug/vision_preview/raw')
OUT = os.path.expanduser('~/dev_ws/log/debug/vision_preview/viz_bset')
os.makedirs(OUT, exist_ok=True)

def sigmoid(x): return 1.0/(1.0+np.exp(-np.clip(x, -20, 20)))

def dfl_decode(reg):
    N = reg.shape[0]; proj = np.arange(16, dtype=np.float32)
    reg = reg.reshape(N, 4, 16)
    sm = np.exp(reg - reg.max(axis=-1, keepdims=True))
    sm /= sm.sum(axis=-1, keepdims=True)
    return (sm @ proj).reshape(N, 4)

def bgr2nv12(bgr, w=640, h=640):
    """BGR uint8 HWC → NV12 via OpenCV I420"""
    i420 = cv2.cvtColor(bgr, cv2.COLOR_BGR2YUV_I420).flatten()
    hw = h * w
    hw_uv = hw // 4
    nv12 = np.empty(hw * 3 // 2, dtype=np.uint8)
    nv12[:hw] = i420[:hw]  # Y plane
    uv = np.zeros(hw_uv * 2, dtype=np.uint8)
    uv[0::2] = i420[hw:hw+hw_uv]      # U at even
    uv[1::2] = i420[hw+hw_uv:hw+hw_uv*2]  # V at odd
    nv12[hw:] = uv
    return nv12

models = dnn.load(MODEL)
m = models[0]
input_size = 640
REG_MAX = 16
conf_thres = 0.01
iou_thres = 0.45
strides = [8, 16, 32]

jpgs = sorted(glob.glob(os.path.join(RAW, '*.jpg')))
n = len(jpgs)
print(f'Model: {m.name}, total: {n}')
print()

t_start = time.perf_counter()
for idx, path in enumerate(jpgs):
    name = os.path.basename(path)
    img = cv2.imread(path)
    if img is None: continue
    h0, w0 = img.shape[:2]

    scale = min(input_size / h0, input_size / w0)
    nh, nw = int(round(h0 * scale)), int(round(w0 * scale))
    img_small = cv2.resize(img, (nw, nh))
    canvas = np.full((input_size, input_size, 3), 114, dtype=np.uint8)
    canvas[:nh, :nw] = img_small

    nv12 = bgr2nv12(canvas)
    outs = m.forward([nv12])

    all_bboxes, all_scores, all_masks_coeff = [], [], []
    for si, s in enumerate(strides):
        reg = outs[si*3].buffer.reshape(-1, 64)
        cls = sigmoid(outs[si*3+1].buffer.reshape(-1, 1))
        coeff = outs[si*3+2].buffer.reshape(-1, 32)
        hg, wg = outs[si*3].buffer.shape[1:3]
        sx, sy = np.meshgrid(np.arange(wg)+0.5, np.arange(hg)+0.5, indexing='xy')
        ap = np.stack((sx.ravel(), sy.ravel()), axis=1).astype(np.float32)
        box = dfl_decode(reg) * s
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

    keep = scores > conf_thres
    bboxes = bboxes[keep]; scores = scores[keep]; masks_coeff = masks_coeff[keep]

    ov = img.copy()
    if len(bboxes) > 0:
        xywh = np.zeros_like(bboxes)
        xywh[:, 0] = bboxes[:, 0]
        xywh[:, 1] = bboxes[:, 1]
        xywh[:, 2] = bboxes[:, 2] - bboxes[:, 0]
        xywh[:, 3] = bboxes[:, 3] - bboxes[:, 1]
        idxs = cv2.dnn.NMSBoxes(xywh.tolist(), scores.tolist(), conf_thres, iou_thres)
        if isinstance(idxs, np.ndarray):
            idxs = idxs.flatten()
        elif isinstance(idxs, (list, tuple)):
            idxs = np.array(idxs).flatten()
        else:
            idxs = np.array([], dtype=int)

        proto = outs[9].buffer.squeeze()
        proto_h, proto_w = proto.shape[0], proto.shape[1]
        protos_2d = proto.reshape(proto_h*proto_w, -1).T
        mask_scale = proto_h / input_size

        for i in idxs:
            box = bboxes[i]
            x1 = max(0, min(w0-1, box[0]/scale))
            y1 = max(0, min(h0-1, box[1]/scale))
            x2 = max(0, min(w0-1, box[2]/scale))
            y2 = max(0, min(h0-1, box[3]/scale))
            mc = masks_coeff[i]
            mask_raw = mc @ protos_2d
            mask_sig = sigmoid(mask_raw.reshape(proto_h, proto_w))
            bx1 = int(max(0, box[0]*mask_scale))
            by1 = int(max(0, box[1]*mask_scale))
            bx2 = int(min(proto_w, box[2]*mask_scale))
            by2 = int(min(proto_h, box[3]*mask_scale))
            mcrop = mask_sig[by1:by2, bx1:bx2]
            if mcrop.size == 0: continue
            bw = int(x2 - x1); bh = int(y2 - y1)
            if bw <= 0 or bh <= 0: continue
            mr = cv2.resize(mcrop, (bw, bh))
            mb = (mr > 0.5).astype(np.uint8)
            mf = np.zeros((h0, w0), dtype=np.uint8)
            mf[int(y1):int(y1)+bh, int(x1):int(x1)+bw] = mb
            cm = np.zeros_like(ov); cm[mf > 0] = (0, 255, 0)
            ov = cv2.addWeighted(ov, 1.0, cm, 0.4, 0)
            ct, _ = cv2.findContours(mf, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(ov, ct, -1, (0, 255, 0), 2)
            cv2.rectangle(ov, (int(x1),int(y1)), (int(x2),int(y2)), (0, 255, 0), 2)
            cx = int((x1+x2)/2); cy = int((y1+y2)/2)
            cv2.circle(ov, (cx, cy), 5, (0, 0, 255), -1)
            cv2.line(ov, (cx, cy), (w0//2, h0//2), (255, 0, 0), 2)
            off = cx/(w0/2)-1.0
            cv2.putText(ov, f'bset {scores[i]:.2f} off={off:+.2f}', (10,30),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
    else:
        cv2.putText(ov, 'NO DET', (10,30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,0,255), 2)

    cv2.imwrite(os.path.join(OUT, name), ov)
    elapsed = time.perf_counter() - t_start
    avg = elapsed / (idx + 1)
    eta = avg * (n - idx - 1)
    if (idx+1) % 50 == 0:
        print(f'  [{idx+1}/{n}] avg:{avg*1000:.0f}ms ETA:{eta/60:.0f}min')

print(f'\nDone! {(time.perf_counter()-t_start)/60:.1f}min')