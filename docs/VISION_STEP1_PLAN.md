# 视觉系统实现规划 — Step 1：视觉预览（纯观察，不动车）

> **版本**：v1.0  
> **状态**：规划中，待实现  
> **实现者**：参见任务分配

---

## 1. 概述

### 1.1 目标

在板端（地平线 RDK X5）上启动摄像头 + 加载 BPU 分割模型 + 运行推理 + 可视化输出。**不发出任何 `/cmd_vel` 运动指令。**

### 1.2 验收标准

| # | 检查项 | 通过条件 |
|---|--------|---------|
| 1 | 编译通过 | `colcon build` exit code 0 |
| 2 | 模型加载 | 日志出现 `[VisionEngine] loaded: saidao_seg_model_quant` |
| 3 | 相机有图 | `/lane_seg_viz` 话题以 ~8-12Hz 发布 |
| 4 | 推理速度 | 单帧推理耗时 ≤ 30ms（rgb_fps=10 时） |
| 5 | 检测正常 | 正对赛道时 `det=True`，`offset ∈ [-1, 1]` |
| 6 | 不动车 | `ros2 topic echo /cmd_vel` 无任何发布 |
| 7 | 保存帧（可选） | 启用 `save_frames:=true` 时 JPG 写入磁盘 |

### 1.3 不做的范围

- 不发出任何运动指令
- 不涉及避障逻辑
- 不涉及段序列/矩形赛道导航
- 不修改 `direct_inertial_tester.py`（方案A）

---

## 2. 开发位置

### 2.1 新建文件

| 文件 | 远端路径 | 说明 |
|------|---------|------|
| 主节点 | `~/dev_ws/src/racing/racing_stage2_param_test/racing_stage2_param_test/vision_preview.py` | ROS 节点 + 推理引擎 + 后处理 |
| Launch | `~/dev_ws/src/racing/racing_stage2_param_test/launch/vision_preview.launch.py` | 启动文件 |

### 2.2 修改文件

| 文件 | 改动内容 |
|------|---------|
| `~/dev_ws/src/racing/racing_stage2_param_test/setup.py` | `entry_points['console_scripts']` 追加 `vision_preview` |

### 2.3 不动文件

| 文件 | 原因 |
|------|------|
| `direct_inertial_tester.py` | 方案A，不受影响 |
| `bak/lane_follow.py` | 仅作为后处理函数的代码来源 |
| `package.xml` | 依赖不变 |
| `models/saidao_seg_model_quant.bin` | 已有，不动 |
| `config/` 下所有文件 | 方案A 配置 |

---

## 3. 详细设计

### 3.1 架构数据流

```
┌─────────────┐    /aurora/rgb/image_raw    ┌──────────────────┐
│  aurora930   │ ──────────────────────────→ │  VisionPreview   │
│  摄像头节点   │   (sensor_msgs/Image bgr8)  │   ROS Node       │
└─────────────┘                               │                  │
                                              │  1. cv_bridge    │
                                              │  2. resize 640   │
                                              │  3. BPU forward  │
                                              │  4. seg mask     │
                                              │  5. center_off   │
                                              │  6. viz overlay  │
                                              └───────┬──────────┘
                                                      │
                                          ┌───────────┴───────────┐
                                          ▼                       ▼
                                    /lane_seg_viz          /lane_seg_mask
                                    (Image overlay)        (Image binary)
```

### 3.2 `vision_preview.py` — 文件结构

文件内部按以下顺序组织（从上到下）：

#### 3.2.1 后处理纯函数（~60 行）

从 `bak/lane_follow.py` **原样复制**，不做任何改动：

| 函数 | 签名 | 说明 |
|------|------|------|
| `_sigmoid(x)` | `float → float` | sigmoid 激活函数 |
| `_init_grids()` | `→ None` | 初始化全局网格缓存（640×640 检测头解码） |
| `_compute_seg_mask(output0, output1, conf_thr, mask_thr)` | `(ndarray, ndarray, float, float) → Optional[ndarray]` | YOLOv8-Seg 后处理：检测头解码 → mask 重构 → 二值化。返回 `(160,160) bool` 或 `None` |
| `_center_offset(binary, roi_bottom)` | `(ndarray, float) → Optional[float]` | 从二值 mask 底部 ROI 计算赛道中心偏差。返回 `[-1, 1]`，正值=赛道偏右需右转 |

#### 3.2.2 `VisionLaneEngine` 类（~60 行）

从 `bak/lane_follow.py` **原样复制**，不做改动：

**构造函数**：
```
__init__(model_path: str, conf_thr=0.3, mask_thr=0.5, roi_bottom=0.35, logger=None)
```

**属性**：
- `ready: bool` — 模型是否成功加载

**方法**：
- `_init()` — 内部调用 `hobot_dnn.pyeasy_dnn.load(model_path)`，取 `models[0]`
- `process(bgr_image: ndarray) → dict` — 核心推理接口
  - 预处理：BGR→RGB → resize(640,640) → transpose(2,0,1) → expand_dims → uint8
  - 推理：`self.model.forward([inp])` → `[output0(1,37,8400), output1(1,32,160,160)]`
  - 后处理：调用 `_compute_seg_mask` + `_center_offset`
  - 返回 dict: `{binary, center_offset, has_detection, viz_overlay, viz_mask}`

#### 3.2.3 可视化辅助函数（~50 行）

从 `bak/lane_follow.py` **原样复制**：

| 函数 | 说明 |
|------|------|
| `_no_det(bgr)` | 无检测时的 fallback（红色 "NO DETECTION" 文字） |
| `_viz_mask(binary)` | 绿色 mask 可视化 `(160,160,3)` |
| `_viz_overlay(bgr, binary, offset, roi_bottom)` | 叠加图：原图×0.6 + 绿色 mask×0.4 + 蓝色边界线 + 红色中心偏差线 + offset 文字 |

#### 3.2.4 `VisionPreview(Node)` 类（~90 行）

**构造函数 `__init__()`**：

1. 声明 ROS 参数（见 3.3 参数表）
2. 读取参数到成员变量
3. 初始化 `CvBridge`、`self._engine = None`（懒加载）、`self._frame_count = 0`
4. 若 `save_frames=True`，创建保存目录
5. 订阅相机话题（`BEST_EFFORT`, `depth=1`, `ReentrantCallbackGroup` 可选）
6. 创建发布者：`/lane_seg_viz` (Image)、`/lane_seg_mask` (Image)
7. 日志输出初始化完成

**`_cam_cb(msg: Image)`** — 相机回调：

1. **懒加载**：首次调用时创建 `VisionLaneEngine` 实例。若加载失败 → 打印 error 并 return（不再处理后续帧）
2. **cv_bridge 解码**：`imgmsg_to_cv2(msg, 'bgr8')`
3. **BPU 推理**：`self._engine.process(cv_img)`，计时
4. **日志**（throttle 1.0s）：`infer={ms}ms det={True/False} offset={±X.XXXX}`
5. **发布可视化**：`viz_overlay` → `/lane_seg_viz`，`viz_mask` → `/lane_seg_mask`
6. **可选保存**：若 `save_frames=True`，`cv2.imwrite()` 到 `save_dir`

**`main()`** 入口：
```python
def main(args=None):
    rclpy.init(args=args)
    node = VisionPreview()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
```

### 3.3 ROS 参数

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `camera_topic` | string | `/aurora/rgb/image_raw` | 相机话题 |
| `model_path` | string | `""`（自动 → `models/saidao_seg_model_quant.bin`） | seg 模型路径，空则自动定位 |
| `conf_threshold` | double | `0.3` | 检测置信度阈值，低于此不输出 mask |
| `mask_threshold` | double | `0.5` | 分割二值化阈值 |
| `roi_bottom` | double | `0.35` | 底部 ROI 比例（用于 center_offset 计算） |
| `save_frames` | bool | `false` | 是否保存 JPG 到磁盘 |
| `save_dir` | string | `""`（自动 → `~/dev_ws/log/debug/vision_preview/`） | JPG 保存目录 |

### 3.4 模型路径自动定位逻辑

当 `model_path` 为空字符串时，自动拼接：

```python
model_path = os.path.normpath(os.path.join(
    os.path.dirname(os.path.dirname(os.path.realpath(__file__))),
    'models', 'saidao_seg_model_quant.bin'
))
```

即从 `vision_preview.py` 所在目录向上两级 → `models/saidao_seg_model_quant.bin`。

### 3.5 订阅/发布话题

| 方向 | 话题 | 类型 | QoS | 说明 |
|------|------|------|-----|------|
| 订阅 | `/aurora/rgb/image_raw` | `sensor_msgs/Image` | BEST_EFFORT, VOLATILE, depth=1 | 相机原始帧 |
| 发布 | `/lane_seg_viz` | `sensor_msgs/Image` | depth=10 | 分割叠加图（含 center line 和 offset 文字） |
| 发布 | `/lane_seg_mask` | `sensor_msgs/Image` | depth=10 | 二值 mask 图（绿色=赛道区域） |

### 3.6 日志规范

正常运行时每 ~1s 输出一条（`throttle_duration_sec=1.0`）：

```
[VisionPreview] infer=12ms det=True offset=+0.2345
[VisionPreview] infer=10ms det=True offset=-0.1234
[VisionPreview] infer=8ms det=False offset=N/A
```

首次加载时：
```
[VisionEngine] loaded: saidao_seg_model_quant
[VisionPreview] 相机=/aurora/rgb/image_raw 就绪
```

---

## 4. Launch 文件设计

### 4.1 `launch/vision_preview.launch.py`

#### 可配置参数

| 参数 | 默认值 | 类型 | 说明 |
|------|--------|------|------|
| `include_camera` | `true` | bool | 启动 aurora930 相机节点 |
| `include_bringup` | `true` | bool | 启动底盘驱动（给相机供电必需） |
| `camera_topic` | `/aurora/rgb/image_raw` | string | 传递给 vision_preview 节点 |
| `conf_threshold` | `0.3` | double | |
| `mask_threshold` | `0.5` | double | |
| `roi_bottom` | `0.35` | double | |
| `save_frames` | `false` | bool | |

#### 启动拓扑

```
vision_preview.launch.py
├── [Include] racing_stage2/launch/competition_support.launch.py
│   ├── include_camera:  true
│   ├── include_bringup: true
│   ├── include_lidar:   false
│   ├── include_bno055:  false
│   └── rgb_fps:         10
│
└── [Node] vision_preview (racing_stage2_param_test)
    ├── camera_topic:    /aurora/rgb/image_raw
    ├── conf_threshold:  0.3
    ├── mask_threshold:  0.5
    ├── roi_bottom:      0.35
    └── save_frames:     false
```

**注意**：`include_bringup=true` 是必需的——aurora930 摄像头通过底盘 USB 供电，不启动底盘驱动则相机无电。

---

## 5. 编译与部署

### 5.1 编译命令

```bash
cd ~/dev_ws
rm -rf build/racing_stage2_param_test install/racing_stage2_param_test
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racing_stage2_param_test
```

### 5.2 预期编译结果

```
Starting >>> racing_stage2_param_test
Finished <<< racing_stage2_param_test [~7s]

Summary: 1 package finished
```

---

## 6. 验证流程

### 6.1 启动预览

```bash
source ~/dev_ws/install/setup.bash
ros2 launch racing_stage2_param_test vision_preview.launch.py
```

### 6.2 验证话题通断

```bash
# 新终端
ros2 topic list | grep lane_seg
# 应输出:
#   /lane_seg_viz
#   /lane_seg_mask

ros2 topic hz /lane_seg_viz
# 应 ~8-12Hz

ros2 topic echo /lane_seg_mask --once
# 应有 Image 数据
```

### 6.3 验证不出 cmd_vel

```bash
ros2 topic echo /cmd_vel
# 应无任何消息（或只看到其他节点发出的停车指令）
```

### 6.4 验证模型推理日志

观察启动终端日志：
```
[VisionEngine] loaded: saidao_seg_model_quant
[VisionPreview] infer=12ms det=True offset=+0.2345
```

如果持续出现 `det=False`：
- 检查摄像头是否正对赛道
- 降低 `rgb_fps` 到 5（减少处理负载）
- 检查光照条件

### 6.5 保存帧验证（可选）

```bash
ros2 launch racing_stage2_param_test vision_preview.launch.py save_frames:=true
# 查看 ~/dev_ws/log/debug/vision_preview/ 下生成的 JPG 文件
```

### 6.6 rviz2 可视化（可选）

```bash
rviz2
# Add → By topic → /lane_seg_viz → Image
# Add → By topic → /lane_seg_mask → Image
```

---

## 7. 依赖与注意事项

### 7.1 运行时依赖

| 依赖 | 说明 |
|------|------|
| `hobot_dnn` | 地平线 BPU 推理库（预装于 RDK X5） |
| `cv_bridge` | ROS ↔ OpenCV 图像转换 |
| `sensor_msgs` | Image 消息类型 |
| `deptrum-ros-driver-aurora930` | 相机驱动（已安装） |

### 7.2 已知风险

| 风险 | 影响 | 缓解措施 |
|------|------|---------|
| `hobot_dnn.pyeasy_dnn` API 变更 | 模型加载/推理失败 | 使用 `try/except` 包裹，失败时快速失败并打 error |
| 推理耗时 > 50ms | 帧率下降，控制延迟 | 调低 `rgb_fps` 到 5，或减小输入分辨率 |
| 相机无图（底盘未供电） | 节点无输出 | `include_bringup=true` 保障供电 |
| 弱光/逆光导致 seg 失效 | `det=False` 持续 | 日志 warning 提示光照问题 |

### 7.3 模型文件

| 属性 | 值 |
|------|-----|
| 文件名 | `saidao_seg_model_quant.bin` |
| 大小 | ~4.9 MB |
| 位置 | `~/dev_ws/src/racing/racing_stage2_param_test/models/` |
| 架构 | YOLOv8-Seg（best.onnx → HBDK 编译） |
| 芯片 | 地平线 bayes-e（J5 / X5） |
| 量化方式 | INT8 + INT16 混合 |
| 输入 | `1×3×640×640` NHWC uint8 |
| 输出0 | 检测头（bbox/category/mask coeff），`(1,37,8400)` |
| 输出1 | 原型 mask 概率图，`(1,32,160,160)` |

---

## 8. 代码来源说明

**后处理 + `VisionLaneEngine` 的源码位置**：

```
bak/lane_follow.py
  ├── 后处理纯函数  → 复制到 vision_preview.py（原样）
  ├── VisionLaneEngine → 复制到 vision_preview.py（原样）
  └── LaneFollowNode   → 不复制（Step 2 再使用其控制逻辑）
```

**选择依据**：`bak/lane_follow.py` 中的推理路径已通过 2026-03 ~ 2026-06 的 3 个月场测验证，`hobot_dnn` API 和模型输入/输出格式稳定。

---

## 9. Step 2 规划前瞻（仅参考，待 Step 1 验收后详细规划）

| 项目 | 内容 |
|------|------|
| **文件** | `vision_lane_controller.py`（新建） |
| **基础** | 复制 `VisionPreview` 作为起点 |
| **新增** | `_control_timer()` 以固定频率（20Hz）发布 `/cmd_vel` |
| **控制** | PID：`kp=1.2, kd=0.4, ki=0.02, max_angular=0.6` |
| **安全** | 赛道丢失 0.6s 持续 → 停车 |
| **速度** | 固定 `linear_speed=0.15 m/s` |
| **避障** | 暂不涉及，纯直道验证 |

---

## 附录 A：实现检查清单

实现者可逐项勾选确认：

- [ ] `vision_preview.py` 创建
  - [ ] 后处理函数复制（`_sigmoid`, `_init_grids`, `_compute_seg_mask`, `_center_offset`）
  - [ ] `VisionLaneEngine` 类复制
  - [ ] 可视化辅助函数复制（`_no_det`, `_viz_mask`, `_viz_overlay`）
  - [ ] `VisionPreview(Node)` 类实现（构造函数、`_cam_cb`、`main`）
- [ ] `launch/vision_preview.launch.py` 创建
- [ ] `setup.py` 追加 `vision_preview` entry_point
- [ ] 编译通过
- [ ] 板端验证通过
- [ ] 更新 `~/dev_ws/log/CHANGELOG.md`
