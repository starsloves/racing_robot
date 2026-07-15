# 赛道分割模型 — saidao_seg_model_quant.bin

## 概述

本模型用于图像语义分割，识别 **赛道（saidao）** 区域。

> **⚠️ 这不是标准 ONNX 文件**，而是**地平线（Horizon Robotics）BPU 编译后的量化模型**（.hbm 格式），只能在地平线 X3/J5 芯片的 BPU 上运行，**无法用 onnxruntime 加载**。

## 技术细节

| 项目 | 内容 |
|---|---|
| 模型文件 | `saidao_seg_model_quant.bin` |
| 原始模型 | YOLOv8-Seg（`best.onnx` → HBDK 编译） |
| 标签 | `saidao`（赛道分割，单类） |
| 架构层数 | model.0 ~ model.22（23 层 backbone + neck + head） |
| 推理芯片 | 地平线 bayes-e 架构（J5 / X5 系列） |
| 量化精度 | INT8 + INT16 混合（uint8, 16BIT_QUANTIZE, PER_CHANNEL） |
| 文件大小 | 4.9 MB |
| HBDK 版本 | 3.49.15（runtime 3.15.55.0） |
| 预处理 | 关闭（`no_preprocess`），BGR → RGB 需调用方处理 |

## 输入输出

| 方向 | 节点名 | 形状 | 布局 | 说明 |
|---|---|---|---|---|
| 输入 | `images` | 1×3×640×640 | NHWC | 8-bit 量化 RGB 图像 |
| 输出0 | `output0` | 1×?×? | NHWC | 检测头：边界框 / 类别 / mask 系数 |
| 输出1 | `output1` | 1×160×160 | NHWC | 分割掩码（Sigmoid 后，单通道概率图） |

### 输出后处理

模型内部已完成 `Sigmoid`，**`output1` 直接是 0~1 的概率图**，无需再应用 sigmoid：

```
mask = output1 > 0.5  # 直接二值化即可得到赛道区域
```

## 模型结构（YOLOv8-Seg）

```
Backbone (model.0 ~ model.9):
  Conv + C2f blocks，含 SPPF（model.9 MaxPool 三路）

Neck (model.10 ~ model.20):
  FPN + PAN 多尺度特征融合（Resize / Concat / C2f）

Head (model.22):
  ├── cv2.{0,1,2}  → 检测框 + 分类（P3/P4/P5 三个尺度）
  ├── cv3.{0,1,2}  → 类别
  ├── cv4.{0,1,2}  → mask 系数（每个框 32 维）
  ├── dfl           → DFL 边界框回归
  ├── proto         → 原型掩码生成
  │   └── cv1/cv2/cv3 + upsample → 160×160 proto mask
  ├── Sigmoid       → 掩码激活
  └── Mul + Add     → mask 系数 × proto → 实例分割（但仅 saidao 一类）
```

## 运行时要求

### 在地平线开发板上运行

```bash
# 需要地平线 BPU 推理 SDK（hb_dnn / libdnn）
# 或在 ROS 节点中使用地平线提供的 Python API
python3 -c "import hb_dnn"  # 地平线 BPU 推理库
```

### 在 x86 PC 上运行（模拟）

需要地平线工具链（`hb_mapper` + 仿真器），不推荐，建议直接在开发板上运行。

## 文件结构

```
models/
├── saidao_seg_model_quant.bin   # 地平线 BPU 量化模型
└── README.md                    # 本文件
```

## 使用场景

- 赛道区域分割，判断可行驶区域（saidao 像素概率图）
- 结合 Stage2 避障模块辅助识别障碍物与赛道边界
- 为导航提供视觉语义信息（如赛道中心线提取）

## 相关文件

- 原始 ONNX 源：`/workspace/input/best.onnx`（编译时路径）
- HBDK 编译中间产物：`main_graph_subgraph_0.hbir`, `main_graph_subgraph_1.hbir`
