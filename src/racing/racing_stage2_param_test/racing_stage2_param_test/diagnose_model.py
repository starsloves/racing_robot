#!/usr/bin/env python3
"""诊断：检查 BPU 模型输出形状和数值"""
import os, sys
import numpy as np

model_path = os.path.expanduser('~/dev_ws/src/racing/racing_stage2_param_test/models/saidao_seg_model_quant.bin')

try:
    from hobot_dnn import pyeasy_dnn as dnn
    models = dnn.load(model_path)
    m = models[0]
    print(f'Model name: {m.name}')
    print(f'Model inputs: {len(m.inputs)}')
    for i, inp in enumerate(m.inputs):
        print(f'  input[{i}]: shape={inp.shape} type={inp.dtype} layout={inp.layout}')
    print(f'Model outputs: {len(m.outputs)}')
    for i, out in enumerate(m.outputs):
        print(f'  output[{i}]: name={out.name} shape={out.shape} type={out.dtype} layout={out.layout}')

    # 创建一个 dummy 输入（全灰 640x640）
    test_in = np.full(m.inputs[0].shape, 128, dtype=np.uint8)
    print(f'\nTest input shape: {test_in.shape}')

    outs = m.forward([test_in])
    for i, o in enumerate(outs):
        buf = o.buffer
        print(f'\noutput[{i}]: buffer shape={buf.shape} dtype={buf.dtype} '
              f'min={buf.min():.4f} max={buf.max():.4f} mean={buf.mean():.4f}')
        if buf.ndim == 3:
            print(f'  channels={buf.shape[0]} h={buf.shape[1]} w={buf.shape[2]}')
        elif buf.ndim == 4:
            print(f'  batch={buf.shape[0]} channels={buf.shape[1]} h={buf.shape[2]} w={buf.shape[3]}')

    # 特别检查 output0 检测头
    buf0 = outs[0].buffer
    print(f'\n--- output0 详细分析 ---')
    raw = buf0.reshape(-1, 8400).T if buf0.size >= 37*8400 else buf0
    print(f'After reshape: {raw.shape}')
    if raw.shape[1] >= 37:
        scores = 1.0/(1.0+np.exp(-np.clip(raw[:, 4], -20, 20)))
        best = np.argmax(scores)
        print(f'Best score idx={best} score={scores[best]:.4f}')
        print(f'Scores range: [{scores.min():.4f}, {scores.max():.4f}]')
        print(f'Scores > 0.3: {(scores > 0.3).sum()} / {len(scores)}')

    # 特别检查 output1 mask
    buf1 = outs[1].buffer
    print(f'\n--- output1 详细分析 ---')
    print(f'Shape: {buf1.shape}')
    print(f'Value range: [{buf1.min():.4f}, {buf1.max():.4f}]')
    print(f'Mean: {buf1.mean():.4f}')

    # 尝试不同 reshape 方式
    if buf1.ndim == 4:
        ch = buf1.shape[1] if buf1.shape[1] <= 32 else buf1.shape[-1]
        h, w = buf1.shape[2], buf1.shape[3]
        print(f'Candidate: batch={buf1.shape[0]} ch={ch} h={h} w={w}')
    elif buf1.ndim == 3:
        print(f'3D array: layout could be (ch,h,w) or (h,w,ch)')
        print(f'  If (32,160,160): ch={buf1.shape[0]} h={buf1.shape[1]} w={buf1.shape[2]}')
        print(f'  If (160,160,32): h={buf1.shape[0]} w={buf1.shape[1]} ch={buf1.shape[2]}')

except Exception as e:
    print(f'ERROR: {e}', file=sys.stderr)
    import traceback
    traceback.print_exc()