# 新避障方案失败分析与建议

## 执行摘要

**结论：三阶段几何闭环避障方案（lane_change_feedback）在当前条件下不可行。**

建议回退到更简单、更鲁棒的方案：**固定时间开环避障（fixed_timing）+ 参数精调**。

---

## 问题现状

### 实测表现（2026-07-09 测试）

| 指标 | 目标 | 实测 | 状态 |
|------|------|------|------|
| 绕障成功 | 不撞障碍物 | ❌ 横偏失控飙到 -72cm | **失败** |
| 回归原轨 | 横偏 ≤5cm，航向 ≤5° | ❌ 横偏 -72cm，航向 -7.2° | **失败** |
| 投影距离 | ≤ 1m | ✅ 锁定在 0.178m | **成功** |
| 左右对称 | 逻辑一致 | ⚠️ 只测了右绕 | **未验证** |

### 关键失败点

**1. SHIFT_OUT 阶段（建立横偏）**
- 耗时：1.80s
- 横偏变化：-0.2cm → -21.0cm（**达标**）
- 航向变化：-2.8° → -34.3°（**超调 13°**）
- **问题**：航向超调严重，但横偏勉强达标

**2. BYPASS_HOLD 阶段（保持通过）**
- 耗时：0.35s（**太短**）
- 横偏变化：-21.0cm → -33.1cm（**越来越偏**）
- 距离判据：`dist_to_obs=0.57m >= pass_margin=0.20m` ✅
- **问题**：
  - 横偏继续增大而不是保持
  - 通过判据过早触发（0.57m 还没真正通过）

**3. MERGE_BACK 阶段（回归轨道）**
- 耗时：2.44s
- 横偏变化：-33.1cm → -71.8cm（**完全失控**）
- **问题**：控制律完全失效，横偏发散

---

## 根本原因分析

### 1. 控制律方向性问题

**当前控制律**：
```python
omega_cmd = k_ψ * e_ψ + k_y * e_y
```

其中：
- `e_ψ = target_heading - current_heading`（航向误差）
- `e_y = target_cross - current_cross`（横偏误差）

**问题**：
- **横偏误差不能直接产生角速度**
- 正确的车辆运动学模型是：`dy/dt = v * sin(ψ)`
- 横偏的修正需要通过**航向偏置**实现，而不是直接叠加到角速度

**实际效果**：
- SHIFT_OUT：目标横偏 -22cm，当前 -0.2cm，误差 -21.8cm
- `omega_cmd = 1.8 * heading_err + 2.5 * (-0.218) = ... - 0.545`
- 理论上应该右转（负角速度），但实际横偏方向和控制方向不匹配

### 2. 坐标系与符号定义不一致

**横偏符号定义混乱**：
- 你们的 `cross_track` 正值可能是右偏，也可能是左偏
- `omega_sign`：左绕 = +1，右绕 = -1
- 但实际 omega 指令和横偏变化方向不匹配

**日志证据**：
```
往右侧绕行（omega_sign = -1）
目标横偏 = -1 * 0.22 = -0.22m
结果：横偏从 -0.2cm 变成 -72cm（越来越负 = 越来越右偏）
说明：车一直在往右偏，但目标就是右偏，为什么还停不住？
```

**结论**：控制律的符号、横偏定义、omega 方向三者不自洽。

### 3. 缺少实际运动学约束

**理想换道模型 vs 实际**：

| 假设 | 理想 | 你们的车 |
|------|------|----------|
| 控制频率 | ≥20Hz | 5Hz（实测） |
| 转向响应 | 无延迟 | 0.15-0.20s 延迟 |
| 横偏测量 | 准确 | 基于轮速里程计，累积误差 |
| 航向测量 | 准确 | IMU 有噪声（±2°） |
| 速度恒定 | 是 | 转弯时会降速 |

**实际约束导致**：
- 横偏误差累积 → 控制律给出大角速度 → 车转过头 → 横偏超调
- 5Hz 控制 → 每步 0.2s → 车已前进 0.1m → 修正滞后
- 斜率限制 10 rad/s² → 虽然快，但仍有 0.1s 加速期

### 4. 通过判据设计缺陷

**当前判据**：`distance(robot, obstacle) >= 0.20m`

**问题**：
- 障碍物坐标是触发时计算的，存在雷达测距误差
- 车绕行时，距离先增大后减小
- 0.57m 就判定通过，但此时车可能还在障碍物侧面

**正确做法应该是**：
- 判断车体中心是否已经**超过**障碍物在轨道方向的投影
- 不是简单的欧几里得距离

---

## 为什么会选择这个方案

### 初衷（正确）

1. 对标用户4条要求
2. 避免固定时间的盲目性
3. 追求几何精确和闭环反馈
4. 参考了学术文献的换道控制

### 但忽略了（致命）

1. **你们的实际硬件条件不支持**：5Hz 控制，轮速里程计，IMU噪声
2. **缺少完整的状态估计**：没有 Kalman 滤波，没有轨道跟踪，横偏测量不可靠
3. **控制律推导不严谨**：直接用横偏误差产生角速度，违反车辆运动学
4. **参数空间太大**：13个参数，相互耦合，调参地狱

---

## 推荐方案：回退到 fixed_timing + 精调

### 为什么 fixed_timing 更适合

**优势**：
1. **开环鲁棒**：不依赖横偏测量精度
2. **参数少**：只需调 2 个角度 + 2 个时间
3. **可预测**：每次避障轨迹接近
4. **硬件友好**：不需要高频反馈

**你们已有的基础**：
- Stage1 的避障就是 fixed_timing，已经跑通
- simple_avoid_test 的 spiral_avoider 已经实现了 fixed_timing 模式
- 只需调参，不需要重写

### fixed_timing 参数调整建议

**当前 fixed_timing 的问题（从旧日志）**：
- Phase2 写死了 35° 过冲
- Phase1 和 Phase2 时间不对称

**推荐参数**：

```yaml
avoidance_mode: 'fixed_timing'

# Phase1: 右转建立侧移
avoidance_turn_angle_deg: 25.0       # 目标航向偏移（比35°保守）
avoidance_phase1_duration_s: 1.2     # 持续时间（比2.5s短，更紧凑）
spiral_linear_speed: 0.20            # 避障速度（比0.25慢，更稳）

# Phase2: 对称回正
avoidance_phase2_duration_s: 1.2     # 与Phase1对称
avoidance_heading_kp: 2.5            # 航向控制增益（提高响应）

# 触发
avoidance_trigger_distance_m: 0.70   # 触发距离（比0.55保守）
avoidance_cooldown_sec: 2.0          # 冷却时间
```

**调参流程**：

1. **先调触发距离**：确保不撞
   - 从 0.70m 开始
   - 如果还撞 → 增大到 0.80m
   - 如果太早 → 减小到 0.65m

2. **再调 Phase1 角度**：确保绕过
   - 从 25° 开始
   - 如果还撞 → 增大到 30°
   - 如果偏太远 → 减小到 20°

3. **调 Phase1 时间**：匹配角度
   - 目标：Phase1 结束时横偏约 20cm
   - 计算：`横偏 ≈ v * t * sin(angle) ≈ 0.20 * 1.2 * sin(25°) ≈ 0.10m`
   - 如果横偏不够 → 增加时间
   - 如果横偏太大 → 减小时间

4. **Phase2 完全对称**：
   - 时间 = Phase1 时间
   - 角度方向相反
   - 不要过冲

5. **验证左右对称**：
   - 左绕和右绕用同一套参数
   - 只改符号

### 如果还要尝试闭环方案

**最低要求**：

1. **先实现可靠的横偏测量**
   - 轮速里程计 + IMU 融合（EKF）
   - 或者加视觉识别中线
   - 精度要求：±2cm

2. **控制频率提升到 20Hz**
   - 当前 5Hz 太低
   - 或者用预测控制（MPC）补偿

3. **先做离线仿真验证**
   - 建立车辆运动学模型
   - 仿真验证控制律收敛性
   - 再上实车

4. **分阶段验证**
   - 先验证横偏控制（直道跟中线）
   - 再验证避障触发和切换
   - 最后验证回归

---

## 技术债务与遗留问题

### 已实现但不可用

- `spiral_avoider.py` 中的 `lane_change_feedback` 模式（1027-1200行）
- `simple_avoid_tester.py` 中的 `projected_distance()` 锁定逻辑
- `avoidance_config.yaml` 中的 13 个新参数

### 为什么不删除

1. 作为反面教材保留
2. 如果未来硬件升级（更高频控制、更好传感器），可以重启
3. 部分辅助函数（如 `_clamp`、日志输出）可以复用

### 建议归档

```bash
# 创建归档分支
git branch archive/lane-change-feedback-failed

# 回退主分支到 fixed_timing
git revert <lane_change_feedback 的提交>
```

---

## 最终建议

### 短期（本周内）

1. **停止调试 lane_change_feedback**
2. **切换到 fixed_timing 模式**：
   ```yaml
   avoidance_mode: 'fixed_timing'
   ```
3. **按上面的参数表逐步调参**
4. **优先保证不撞，再优化回归**

### 中期（比赛前）

1. **用 fixed_timing 完成比赛**
2. **记录每次避障的最终横偏和航向**
3. **如果横偏残留大，可以增加 Phase3 微调**：
   ```yaml
   enable_phase3: true
   avoidance_phase3_turn_angle_deg: 5.0
   avoidance_phase3_duration_s: 0.5
   ```

### 长期（比赛后）

1. **升级控制频率到 20Hz**（修改 stage2_inertial_navigator 的定时器）
2. **集成视觉中线检测**（替代轮速横偏估计）
3. **重新实现闭环避障**，但用 **Pure Pursuit** 而不是自创控制律
4. **离线仿真先验证再上车**

---

## 附录：控制律推导（正确做法）

### 车辆运动学模型

```
ẋ = v * cos(ψ)
ẏ = v * sin(ψ)
ψ̇ = ω
```

### 横偏控制的正确方法

**方法1：Pure Pursuit**
```
lookahead_point = (x_ref + d_ahead, y_ref)
α = atan2(lookahead_point.y - y, lookahead_point.x - x) - ψ
ω = 2 * v * sin(α) / d_ahead
```

**方法2：Stanley Controller**
```
e_ψ = ψ_ref - ψ            # 航向误差
e_y = y_ref - y            # 横偏误差
ψ_target = ψ_ref + atan(k * e_y / v) # 航向修正
ω = K_ψ * (ψ_target - ψ)    # 角速度
```

**方法3：LQR**
```
状态: x = [e_y, ψ_err, e_y_dot, ψ_dot]^T
控制: u = -K * x
K 通过求解 Riccati 方程获得
```

### 为什么你们的控制律错了

**错误**：`ω = k_ψ * e_ψ + k_y * e_y`

**问题**：
- `e_y` 的单位是米
- `ω` 的单位是 rad/s
- `k_y` 的单位必须是 (rad/s)/m
- 但横偏 1cm → ω = 0.025 rad/s **没有考虑速度**

**正确**：横偏修正必须通过航向偏置，且与速度相关
```
ψ_correction = atan(k * e_y / v)
ω = K * (ψ_ref + ψ_correction - ψ)
```

---

## 总结

**lane_change_feedback 方案失败的核心原因**：

1. ❌ 控制律违反车辆运动学
2. ❌ 硬件条件不支持（5Hz，轮速里程计）
3. ❌ 参数空间太大（13个参数）
4. ❌ 缺少仿真验证就直接上车

**推荐回退到 fixed_timing 的理由**：

1. ✅ 开环鲁棒，不依赖测量精度
2. ✅ 参数少，易调试
3. ✅ Stage1 已验证可行
4. ✅ 满足比赛基本要求

**如果坚持闭环，需要先做**：

1. 提升控制频率到 20Hz
2. 集成可靠的横偏测量
3. 离线仿真验证
4. 使用成熟的控制律（Pure Pursuit / Stanley / LQR）

---

## 推荐新方案：Local Path Pure Pursuit（2026-07-09）

### 方案概述

**核心思想**：先构造一条几何明确的局部避障路径，再用真实 Pure Pursuit 跟踪。

**与 lane_change_feedback 的本质区别**：
- `lane_change_feedback`：直接用横偏误差 `e_y` 叠加到角速度，违反车辆运动学
- `local_path_pure_pursuit`：先规划路径点，再用 Pure Pursuit 跟踪，符合工程实践

### 方案设计

#### 1. 局部路径构造（在轨道坐标系中）

**坐标系定义**：
- `x` 轴：原轨道方向（触发避障时的航向）
- `y` 轴：垂直于轨道（左正右负）
- 原点：触发避障时车辆位置

**路径点**：
```
P0 = (0, 0)                          起点（触发位置）
P1 = (s1, y_clear * sign)            侧移点
P2 = (s_obs + s_pass, y_clear * sign) 旁路点
P3 = (s_obs + s3_margin, 0)           回归点
```

**参数说明**：
- `s1 = 0.35m`：开始侧移的纵向距离
- `y_clear = 0.20m`：旁路横向偏移（车宽半径 0.13 + 余量）
- `s_pass = 0.20m`：P2 超过障碍物的纵向余量
- `s3_margin = 0.55m`：P3 在障碍物之后的距离
- `sign`：右绕 = -1，左绕 = +1

**投影距离控制**：
```
总投影距离 = s_obs + s3_margin ≈ 0.70 + 0.55 = 1.25m
```
（假设触发距离 `s_obs ≈ 0.70m`）

可以通过调整 `s3_margin` 来满足 `≤ 1m` 的约束。

#### 2. Pure Pursuit 控制

**控制律**：
```python
# 在路径上找预瞄点（距离车 lookahead 远）
target_s = robot_s + lookahead

# 插值得到目标点 (target_x, target_y)
target_x, target_y = interpolate_path(target_s)

# 计算预瞄向量在轨道系中的角度
dx = target_x - robot_s
dy = target_y - robot_y
alpha = atan2(dy, dx) - heading_error

# 角速度指令
omega = K_heading * alpha
omega = clamp(omega, omega_max)
```

**参数建议**：
- `lookahead = 0.30m`
- `K_heading = 1.5`
- `omega_max = 0.40 rad/s`

#### 3. 完成判据（三条同时满足）

**不再用欧氏距离判断"通过障碍"**，改用轨道投影：

1. **轨道投影超过 P3**：`robot_s >= p3_s`
2. **横偏回归**：`|robot_y| <= 0.05m`
3. **航向回正**：`|heading_error| <= 5°`

### 优势分析

| 指标 | lane_change_feedback | local_path_pure_pursuit |
|------|----------------------|-------------------------|
| 控制律物理正确性 | ❌ 横偏直接叠加到角速度 | ✅ Pure Pursuit 标准控制律 |
| 左右对称性 | ✅ 逻辑对称 | ✅ 几何对称 + 逻辑对称 |
| 投影距离可控 | ⚠️ 取决于横偏建立速度 | ✅ 由 `s3_margin` 显式控制 |
| 通过障碍判据 | ❌ 欧氏距离，易误判 | ✅ 轨道投影，精确 |
| 回归原轨 | ⚠️ 依赖横偏闭环质量 | ✅ P3 点显式定义回归目标 |
| 调参复杂度 | ⚠️ 13个参数，耦合严重 | ✅ 5个核心参数，独立 |
| 硬件友好性 | ⚠️ 需要高频 + 准确横偏 | ✅ 5Hz 可用，横偏容错 |

### 实现要点

**1. 横偏测量来源**

虽然方案不直接用横偏做反馈，但 Pure Pursuit 需要知道车相对路径的偏移。

来源：
- 主控提供的 `on_cross_error(cross_error_m)`
- 基于轮速里程计的累积横偏

容错性：
- 横偏误差不直接影响角速度大小
- 只影响预瞄点选择，误差 ±5cm 时控制仍稳定

**2. 路径插值**

实现简化为线性插值：
```python
def interpolate_path(target_s):
    # 在相邻路径点间线性插值
    for i in range(len(waypoints) - 1):
        s0, y0 = waypoints[i]
        s1, y1 = waypoints[i + 1]
        if s0 <= target_s <= s1:
            ratio = (target_s - s0) / (s1 - s0)
            y = y0 + ratio * (y1 - y0)
            return (target_s, y)
```

**3. 障碍物位置记录**

触发时记录：
```python
s_obs = front_distance * cos(front_angle_rad)
```

后续用于：
- 构造 P2、P3 点位置
- 判断是否通过障碍（可选，主要靠 P3 判据）

### 配置参数

**配置文件**：`config/avoidance_config.yaml`

```yaml
avoidance_mode: 'local_path_pure_pursuit'
spiral_linear_speed: 0.18

# Local Path Pure Pursuit 参数
lpp_s1: 0.35
lpp_y_clear: 0.20
lpp_s_pass: 0.20
lpp_s3_margin: 0.55
lpp_lookahead: 0.30
lpp_heading_kp: 1.5
lpp_max_omega: 0.40
lpp_finish_heading_tol_deg: 5.0
lpp_finish_cross_tol_m: 0.05
```

### 调参指南

**如果绕障不够（撞障碍物）**：
1. 增大 `lpp_y_clear`（如 `0.20 → 0.22`）
2. 增大触发距离 `avoidance_trigger_distance_m`（如 `0.65 → 0.75`）

**如果投影距离超 1m**：
1. 减小 `lpp_s3_margin`（如 `0.55 → 0.45`）
2. 减小 `lpp_s1`（如 `0.35 → 0.30`）

**如果回归不佳（横偏大、航向偏）**：
1. 增大 `lpp_s3_margin`（给更多距离回归）
2. 增大 `lpp_heading_kp`（加快航向修正）
3. 增大 `lpp_lookahead`（更平滑跟踪）

**如果左右不对称**：
- 检查 `omega_sign` 逻辑（右绕 = -1，左绕 = +1）
- 检查横偏符号定义（左正右负）

### 验收清单

| 要求 | 实现方式 | 验证方法 |
|------|----------|----------|
| ✅ 绕障成功 | 路径点 P1、P2 显式定义侧移 | 触发后车不撞障碍物 |
| ✅ 回归原轨 | P3 点定义回归目标 + 双判据 | `\|cross\| <= 5cm`, `\|heading\| <= 5°` |
| ✅ 左右对称 | 所有参数共用，只翻转符号 | 左绕/右绕用同一套参数 |
| ✅ 硬件友好 | Pure Pursuit 平滑，`omega_max` 限幅 | 不频繁打轮 |
| ✅ 投影距离 ≤ 1m | `s3_margin` 显式控制 | 实测投影距离 |
| ✅ 术语规范 | 轨道坐标系、轨道投影判据 | 日志输出 `robot_s`, `robot_y` |

### 与其他方案对比

| 方案 | 优点 | 缺点 | 推荐度 |
|------|------|------|--------|
| **local_path_pure_pursuit** | 控制律正确、左右对称、投影可控、工程成熟 | 需要横偏估计（但容错） | ⭐⭐⭐⭐⭐ |
| fixed_timing | 开环鲁棒、参数少 | Phase2 写死过冲，回归不佳 | ⭐⭐⭐ |
| lane_change_feedback | 闭环反馈 | 控制律错误、横偏闭环不稳 | ⭐（已失败） |
| 双圆弧/定曲率 | 转向连续、参数少 | 偏开环，适应性一般 | ⭐⭐⭐⭐ |

### 下一步

1. ✅ 代码已实现（`spiral_avoider.py:1275-1470`）
2. ✅ 配置已更新（`config/avoidance_config.yaml`）
3. ⏳ 推送到远端并编译
4. ⏳ 实车测试并调参
5. ⏳ 记录测试数据到本文档

---

**文档编写时间**：2026-07-09  
**状态**：
- `lane_change_feedback` 已失败，已归档  
- `local_path_pure_pursuit` 已实现，待测试  
**下一步**：远端编译 → 实车测试 → 调参优化
