# Pure Pursuit 路径跟踪控制说明

## 改造概述

将原来基于 PD + 曲率的视觉纠偏系统改造为 **Pure Pursuit（纯跟踪）** 几何路径跟踪控制。

---

## 主要变化

### 1. 控制模式切换
新增参数 `vision_use_pure_pursuit` 控制模式：
- **True**：使用 Pure Pursuit 几何跟踪（推荐，更稳定）
- **False**：使用传统 PD + 曲率控制（旧方案）

### 2. Pure Pursuit 参数

```yaml
# 在 stage2_inertial_navigator 的参数中添加：
vision_use_pure_pursuit: true              # 启用 Pure Pursuit
vision_lookahead_distance_m: 0.35          # 前瞻距离（米）
vision_wheelbase_m: 0.15                   # 车辆轴距（米）
vision_pursuit_kp: 1.8                     # Pure Pursuit 增益
```

**参数说明**：
- `vision_lookahead_distance_m`：前瞻距离，越大转弯越平滑但响应慢
  - 推荐范围：0.25m ~ 0.50m
  - 直道多用大值（0.40~0.50m）
  - 弯道多用小值（0.25~0.35m）
  
- `vision_wheelbase_m`：车辆轴距，影响转向灵敏度
  - 实际测量车辆前后轮距离
  - 典型值：0.10m ~ 0.20m
  
- `vision_pursuit_kp`：总增益，放大/缩小控制量
  - 推荐范围：1.5 ~ 2.5
  - 车速快时用小值，车速慢时用大值

---

## 工作原理

### Pure Pursuit 几何公式

```
α = atan2(x, y)              # x=横向偏移, y=前瞻距离
ω = (2 * V * sin(α)) / L     # V=速度, L=轴距
```

### 视觉处理流程

1. **Seg 分割**：YOLOv8-Seg 提取赛道 mask
2. **多行采样**：在 mask 上采样 9 行，提取左/右边界和中线
3. **坐标转换**：将像素坐标转换为世界坐标（相对车体）
   ```python
   # 图像底部 = 车前 0.1m，图像顶部 = 车前 2.5m
   # 视野宽度在前瞻距离处约 1.2m
   ```
4. **前瞻点选择**：在引导中线上取前瞻点（默认 62% 远处）
5. **Pure Pursuit 计算**：基于前瞻点坐标计算所需角速度

### 可视化增强

访问 `http://100.114.34.86:8082/vision_latest.jpg` 可看到：
- 🔴 **左边界**：红色粗线
- 🔵 **右边界**：蓝色粗线
- 💛 **引导中线**：黄色粗线（小车应该沿此线行驶）
- 💗 **前瞻目标点**：洋红色大圆（Pure Pursuit 跟踪点）

---

## 调参指南

### 场景1：直道抖动
**现象**：直道上小车左右摆动

**解决**：
```yaml
vision_lookahead_distance_m: 0.45    # 增大前瞻距离
vision_pursuit_kp: 1.5               # 降低增益
vision_deadband: 0.05                # 增大死区
```

### 场景2：转弯切内/外
**现象**：转弯时偏离中线

**解决（切内侧）**：
```yaml
vision_lookahead_distance_m: 0.30    # 减小前瞻距离
vision_pursuit_kp: 2.0               # 增大增益
```

**解决（切外侧）**：
```yaml
vision_lookahead_distance_m: 0.40    # 增大前瞻距离
vision_pursuit_kp: 1.6               # 降低增益
```

### 场景3：转弯响应慢
**现象**：转弯时反应不及时

**解决**：
```yaml
vision_lookahead_distance_m: 0.25    # 减小前瞻距离（更激进）
vision_pursuit_kp: 2.2               # 增大增益
vision_lookahead_ratio: 0.50         # 减小前瞻比例（采样点更靠前）
```

### 场景4：高速不稳定
**现象**：速度快时震荡

**解决**：
```yaml
vision_lookahead_distance_m: 0.50    # 增大前瞻（平滑轨迹）
vision_pursuit_kp: 1.4               # 降低增益（降低响应）
vision_max_angular: 0.30             # 降低最大角速度限幅
```

---

## 与旧方案对比

| 特性 | Pure Pursuit | PD + 曲率 |
|------|-------------|-----------|
| 参数数量 | 3 个 | 5+ 个 |
| 稳定性 | 更稳定 | 易震荡 |
| 调参难度 | 简单 | 复杂 |
| 物理意义 | 清晰（几何） | 模糊 |
| 适应性 | 自动适应弯道 | 需手动调 |

---

## 回退到旧方案

如果 Pure Pursuit 有问题，可随时切回：

```yaml
vision_use_pure_pursuit: false       # 禁用 Pure Pursuit
vision_angular_kp: 1.25              # 旧 PD 参数
vision_angular_kd: 0.20
vision_curvature_kp: 0.45
```

---

## 相机标定

Pure Pursuit 依赖相机几何标定，当前假设：
- 图像底部对应车前 **0.1m**
- 图像顶部对应车前 **2.5m**
- 视野宽度在前瞻距离处约 **1.2m**

**如需调整**，修改 `vision_lane_centering.py` 中的：
```python
near_distance = 0.1   # 底部距离
far_distance = 2.5    # 顶部距离
fov_width_at_lookahead = 1.2  # 视野宽度
```

---

## 典型配置

### 保守配置（稳定优先）
```yaml
vision_use_pure_pursuit: true
vision_lookahead_distance_m: 0.45
vision_wheelbase_m: 0.15
vision_pursuit_kp: 1.5
vision_max_angular: 0.30
```

### 激进配置（响应优先）
```yaml
vision_use_pure_pursuit: true
vision_lookahead_distance_m: 0.28
vision_wheelbase_m: 0.15
vision_pursuit_kp: 2.2
vision_max_angular: 0.40
```

### 平衡配置（推荐）
```yaml
vision_use_pure_pursuit: true
vision_lookahead_distance_m: 0.35
vision_wheelbase_m: 0.15
vision_pursuit_kp: 1.8
vision_max_angular: 0.35
```

---

## 测试检查清单

- [ ] 直道：中线保持稳定，无明显抖动
- [ ] 入弯：提前响应，不冲出赛道
- [ ] 弯中：沿引导线行驶，不切内/外
- [ ] 出弯：平滑回正，不过冲
- [ ] 可视化：引导线清晰，前瞻点合理
- [ ] 边界保护：接近边界时降权生效

---

## 常见问题

**Q: Pure Pursuit 和 PD 能同时用吗？**  
A: 不能。通过 `vision_use_pure_pursuit` 二选一。

**Q: 如何判断前瞻距离是否合适？**  
A: 观察可视化图像中的洋红色前瞻点：
- 太近 → 转弯激进，易震荡
- 太远 → 转弯迟缓，易冲出

**Q: 为什么有时还是会偏离？**  
A: 检查：
1. Seg 分割质量（绿色 mask 是否准确）
2. 边界检测（红蓝线是否正确）
3. 引导线连续性（黄线是否断裂）
4. IMU 融合权重（是否 IMU 干扰了视觉）

**Q: 速度变化影响大吗？**  
A: Pure Pursuit 会根据速度自动调整。当前默认速度 0.25 m/s，实际应从父节点获取 `self.current_speed`。

---

## 进阶优化

### 1. 自适应前瞻距离
根据速度动态调整：
```python
lookahead = base_distance + k * speed
```

### 2. 曲率预测
结合 mask 边界曲率，提前调整：
```python
alpha_adjusted = alpha + k_curve * curve
```

### 3. 多目标点融合
使用多个前瞻点加权平均，增强鲁棒性。

---

## 日志说明

启动时看到：
```
[视觉] Pure Pursuit控制启用 lookahead=0.35m wheelbase=0.15m kp=1.8 max_ω=0.35 ...
```

运行时看到（每 0.5 秒）：
```
VISION_CTRL VIS_PRIMARY e=+0.123 curve=-0.045 rows=8 conf=0.85 lat=+0.087m ...
```

其中 `lat=横向误差（米）` 是实际物理偏移，不再是归一化的 [-1,1]。
