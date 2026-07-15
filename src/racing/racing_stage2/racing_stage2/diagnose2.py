#!/usr/bin/env python3
"""诊断：检查 BPU 模型输出形状和数值"""
import numpy as np

from hobot_dnn import pyeasy_dnn as dnn

model_path = '/home/sunrise/dev_ws/src/racing/racing_stage2_param_test/models/saidao_seg_model_quant.bin'
models = dnn.load(model_path)
m = models[0]
print('Model name:', m.name)

inp0 = m.inputs[0]
print('Input properties:', [x for x in dir(inp0) if not x.startswith('_')])
# Try to access properties
for attr in ['shape', 'dtype', 'layout', 'name', 'properties', 'data_shape', 'tensor_shape', 'ndim']:
    try:
        v = getattr(inp0, attr)
        print(f'  input.{attr} = {v}')
    except:
        pass

# Try NCHW first
for layout_name, inp_data in [('NCHW(1,3,640,640)', np.full((1,3,640,640), 128, dtype=np.uint8)),
                               ('NHWC(1,640,640,3)', np.full((1,640,640,3), 128, dtype=np.uint8))]:
    print(f'\n--- Testing {layout_name} ---')
    try:
        outs = m.forward([inp_data])
        print(f'  Outputs: {len(outs)}')
        for i, o in enumerate(outs):
            b = o.buffer
            print(f'  output[{i}]: shape={b.shape} dtype={b.dtype} '
                  f'min={b.min():.4f} max={b.max():.4f} mean={b.mean():.4f}')
            if b.ndim == 4:
                print(f'    layout: N={b.shape[0]} C={b.shape[1]} H={b.shape[2]} W={b.shape[3]}')
                # Maybe NHWC?
                print(f'    alt layout: N={b.shape[0]} H={b.shape[1]} W={b.shape[2]} C={b.shape[3]}')
            if b.size >= 32 * 160 * 160:
                # Reshape as 32ch proto
                proto = b.reshape(32, 160, 160)
                print(f'    As 32ch proto: min={proto.min():.4f} max={proto.max():.4f}')
            elif b.size == 160 * 160:
                print(f'    Single channel 160x160 mask')
    except Exception as e:
        print(f'  Failed: {e}')