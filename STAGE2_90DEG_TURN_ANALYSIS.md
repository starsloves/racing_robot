# Stage2 惯导 90° 转角分析报告

> 生成时间: 2026-06-06
> 覆盖版本: 覆盖此前所有分析
> 分析范围: `racing_stage2_param_test` 包 + `lane_follow.py`

---

## 一、摘要

**问题：矩形赛道转弯角度不准，90° 转角实际只转了约 35°。**

从 `latest.log` 实际运行数据确认:
- 第 0 段 `rect_enter_align` 目标转 +90° (0.1° -> 90.1°)
- 执行 12.98 秒后超时退出，此时实测 `yaw=35.2°`，仅完成 39%
- 轮速 `wheel_v.angular.z=0.080 rad/s`，而指令 `cmd_v.angular.z=0.750 rad/s`，**执行效率仅 10.7%**

---

## 二、核心根因

### 问题 1: 角速度指令与实际执行严重不匹配

| 参数 | 值 |
|---|---|
| `turn_angular_speed` | **0.75 rad/s (43°/s)** |
| 实测轮速角速度 | **0.08 rad/s (4.6°/s)** |
| 执行效率 | **10.7%** |

**结论**: 底盘电机驱动无法以 0.75 rad/s 的指令速度完成差速转向。可能是:
- **物理原因**: 地面摩擦不足 / 轮胎打滑 / 电机扭矩不够
- **驱动原因**: `origincar_base` 有内部加速度限幅 / PID 饱和 / 速度缩放
- **指令原因**: `/stage2_cmd_vel` -> `twist_cmd_relay` -> `/cmd_vel` 链路中值被缩放

### 问题 2: 超时机制导致段航向污染

```python
# direct_inertial_tester.py control_loop()
if now_sec - self.segment_started_at > self.segment_timeout:
    # 直接跳到下一段
    self.start_segment(self.plan_index + 1)
```

当转弯超时后，`_unify_segment_pose()` 将 `segment_heading` 设为 `current_yaw=35.2°` (未完成的当前朝向)。

后续的 `rect_first_leg` 底边段本该航向 ~90° (朝左)，实际 `segment_heading=35.2°`，**整个矩形坐标系被扭曲**。

### 问题 3: 轮速里程计角速度跳变

日志显示 wheel yaw 从 35.2° -> 86.4° 仅用 0.03 秒 (一帧)，不符合物理规律。指示 `/odom` 话题可能存在:
- 时间戳错位
- 多源传感器融合延迟
- 坐标变换问题

### 问题 4: heading_tolerance 累积误差

配置文件 `inertial_stage2.yaml` 中 `heading_tolerance_deg: 3.5°`，但 `latest.log` 实际显示:
```
head_tol=4.0deg
```
来自 `direct_inertial_test.yaml` 未覆盖，使用基类默认值。

4 个拐角累积误差 = 4 x 4.0° = **16°**。

---

## 三、详细参数对比

### 配置文件参数

| 参数 | inertial_stage2.yaml | direct_inertial_test.yaml | 生效值 |
|---|---|---|---|
| `turn_angular_speed` | 0.65 | - | 0.65 |
| `turn_kp` | 1.8 | - | 1.8 |
| `heading_tolerance_deg` | 3.5 | - | 3.5 (日志显示4.0) |
| `segment_timeout` | 25.0 | - | 25.0 |
| `turn_linear_speed` | 0.08 | - | 0.08 |
| `ring_linear_speed` | 0.24 | - | 0.24 |
| `rectangle_first_leg_m` | - | 1.10 (启动默认) | 实际=1.20 |
| `rectangle_side_leg_m` | - | 0.50 (启动默认) | 实际=0.60 |
| `rectangle_top_leg_m` | - | 2.80 (启动默认) | 实际=2.80 |

注意: 启动命令中覆盖了矩形参数，与 YAML 默认值不同。

### build_ring_plan() 生成的段序列 (顺时针)

```
[0]  turn +90.0deg   rect_enter_align      <- 入口对齐 (从通道方向左转90°)
[1]  move  L=1.20m   rect_first_leg        <- 底边向左
[2]  turn -90.0deg   rect_corner_1         <- 左下拐角
[3]  move  L=0.60m   rect_side_1           <- 左边向上
[4]  turn -90.0deg   rect_corner_2         <- 左上拐角
[5]  move  L=2.80m   rect_top              <- 顶边向右
[6]  turn -90.0deg   rect_corner_3         <- 右上拐角
[7]  move  L=0.60m   rect_side_2           <- 右边向下
[8]  turn -90.0deg   rect_corner_4         <- 右下拐角
[9]  move  L=1.20m   rect_return_origin    <- 底边回起点
```

矩形几何:
```
        2.80m (rect_top)
    +-----------------------+
    |                       |
0.6m|                       |0.6m
    |                       |
    +-----------------------+
    <- 1.20m ->   <- 1.20m ->
   (first_leg)   (return_origin)
```

---

## 四、位姿源分析

### 当前架构

```
导航 (direct_inertial_tester)
  +-- 位姿源: navigation_pose_source='wheel'
  +-- 轮速话题: wheel_odom_topic='/odom'
  +-- EKF话题: odom_topic='/odom_combined' (仅日志对比)
  |
  +-- current_yaw <-- 来自 _sync_unified_pose_from_wheel()
  |   +-- current_wheel_yaw <-- /odom 的 orientation
  |
  +-- current_position <-- 来自 /odom 的 position
  |
  +-- turn 控制: error = target_yaw - navigation_yaw()
  |   +-- navigation_yaw() 返回 current_yaw
  |
  +-- move 控制: angular = heading_kp x heading_error
      +-- heading_error = segment_heading - current_yaw
```

### 潜在问题

`/odom` 话题在 `origincar` 平台上通常由 `robot_pose_ekf` 发布 (融合 IMU + 轮速编码器)，**并非纯轮速里程计**。如果 EKF 的角速度权重偏低，会导致转向时 yaw 变化跟不上指令。

建议验证:
1. `ros2 topic echo /odom --once` 检查 frame_id 和 twist
2. 对比 `/odom` 与原始 `/odom_combined` 的 yaw 变化速率
3. 检查 `origincar_base` 的速度控制模式 (是否有 `max_angular_vel` 限幅)

---

## 五、lane_follow.py 视觉车道节点

### 架构

```
Camera(/aurora/rgb/image_raw)
  -> LaneFollowNode
    -> /lane_cmd_vel (PID 控制)
    -> /lane_seg_viz (可视化叠加)
    -> /lane_seg_mask (二值掩码)
```

### 推理流程

1. **预处理**: BGR -> RGB -> resize 640x640 -> transpose CHW
2. **BPU 推理**: YOLOv8-seg 模型 `saidao_seg_model_quant.bin`
3. **后处理**:
   - `_decode_bboxes()`: 从 37x8400 张量解码边界框 + mask 系数
   - `_compute_seg_mask()`: NMS 选择最佳检测 -> mask 系数 x proto 特征 -> sigmoid -> 二值
   - `_center_offset()`: 取 ROI 底部 35% 区域车道线左右边界 -> 归一化偏移量 [-1, +1]
4. **PID 控制**: `kp=0.8, kd=0.3, ki=0.01, max_angular=0.6`

### 控制公式

```python
offset > 0 -> 赛道在图像右侧 -> angular.z = -(kp*offset + kd*deriv + ki*integral)
linear = linear_speed * (1.0 - |angular|/max_angular * 0.5)
```

### 与惯导的关系

- 话题隔离: lane_follow 发 `/lane_cmd_vel`，惯导发 `/stage2_cmd_vel`
- 通过 `twist_cmd_relay` 转发到 `/cmd_vel` (谁最后发谁覆盖)
- **无法同时运行**两者 (会争抢 `/cmd_vel`)

---

## 六、改进建议

### 优先级 P0 (90° 转角问题)

1. **降低 `turn_angular_speed`** 从 0.75 -> **0.30 rad/s**，匹配底盘实际执行能力
2. **增加 `segment_timeout`** 从 12.0s -> **30.0s** (至少在修好角速度前)
3. **转角段关闭前向速度**: `turn_linear_speed: 0.08 -> 0.00` (原地转向，避免转弯时位移)

### 优先级 P1 (控制改进)

4. **添加角速度闭环监控**: 在 `run_turn_segment()` 中检测 `cmd_angular` 与实际 `wheel_v.angular.z` 的偏差，若持续 >50% 则报警
5. **减小 `heading_tolerance_deg`**: 4.0° -> **2.0°**，减少累积误差
6. **超时后记录完整诊断**: 在超时日志中加入 `wheel_v / cmd_v` 对比

### 优先级 P2 (车道视觉)

7. **PID 参数标定**: 当前 kp=0.8 偏大 (max_angular=0.6 -> 0.8x1.0=0.8 > 0.6)，减小到 **kp=0.5**
8. **lost_timeout 保护**: 丢失检测后立即停车，当前 0.5s 太长 -> **0.2s**

---

## 七、日志关键行摘录

```
[TIMEOUT] 段超时 rect_enter_align | t=... seg_t=12.98s
  yaw=35.2 yaw_wheel=35.2 yaw_imu=43.2 yaw_ekf=42.5
  yaw_seg0=0.1 yaw_tgt=90.1
  turn_err=54.9
  wheel_v=(0.085,0.000,0.080) cmd_v=(0.080,0.750)
  # 轮速角速度 0.080 vs 指令角速度 0.750 => 仅 10.7%
```

```
[ODOM_ANCHOR] desc=rect_first_leg
  start=(0.247,0.085) yaw=35.2deg yaw_imu=43.2deg L=1.20m
  # 段航向被污染为 35.2°，而非预期的 ~90°
```

---

*文档结束 -- 覆盖此前所有版本*
