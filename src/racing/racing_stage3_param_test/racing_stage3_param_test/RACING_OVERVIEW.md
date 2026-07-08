# Racing 三阶段方案总览

> **编辑约束**：本文档位于 `src/racing/RACING_OVERVIEW.md`。
> Stage 2 和 Stage 3 的开发代码分别在 `racing_stage2_param_test` 和 `racing_stage3_param_test` 测试包。
> **本文档必须随阶段方案变更同步更新。**

---

## 1. 系统架构总览

### 1.1 三阶段流转

```
competition_controller.py（Stage1 主控）
  │
  │  phase=1 ──── Stage1: 通道导航 + QR 扫码
  │                │
  │                ├── 盲驱前进 0.2 m/s
  │                ├── 激光聚类避障（4态状态机）
  │                └── QR 码扫描 → phase=2
  │
  ├── phase=2 ──── Stage2: 矩形赛道惯性导航
  │                │ 测试包：racing_stage2_param_test
  │                ├── [方案A] DirectInertialTester — 轮速里程计 dead reckoning
  │                └── [方案B] VisionInertialTester — 视觉 YOLOv8-Seg 赛道跟随
  │
  └── phase=3 ──── Stage3: 返程导航
                   │ 测试包：racing_stage3_param_test
                   ├── map 系航点追踪
                   ├── Pure Pursuit + 可选 A*
                   └── 终点 P 点 (0.20, 0.20)
```

### 1.2 关键 Topic 拓扑

| Topic | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/cmd_vel` | Twist | competition_controller (phase1) / Stage3 | phase1/3 控制输出 |
| `/stage2_cmd_vel` | Twist | DirectInertialTester / VisionInertialTester | phase2 独立控制 |
| `/odom` | Odometry | origincar_base | 轮速里程计（编码器）— Stage2 主位姿源 |
| `/odom_combined` | PoseWithCovarianceStamped | robot_localization EKF | IMU+轮速融合 — Stage3 使用 |
| `/map` | OccupancyGrid | SLAM Toolbox | 全局地图 — Stage3 使用 |
| `/scan` | LaserScan | 激光雷达 | 避障输入 |
| `/aurora/rgb/image_raw` | Image | 摄像头 | BPU 视觉输入 |
| `competition_phase` | Int32 | competition_controller | 阶段序号（1/2/3） |
| `stage2_state` | String | Stage2 各方案 | Stage2 内部状态 |
| `stage3_state` | String | Stage3ReturnNavigator | Stage3 内部状态 |
| `qr_scan_result` | String | qr_scanner | QR 解码结果 |
| `competition_qr_task` | String | competition_controller | QR 扫描方向指令 |
| `sign4return` | Int32 | competition_controller | 返程 AI 触发信号 |
| `/image` | CompressedImage | 摄像头 | AI 输入（vision_ai） |

### 1.3 坐标系

| 坐标系 | 类型 | 说明 |
|---|---|---|
| `/odom` | 局部里程计 | 轮速编码器积分，**Stage2 测试包主位姿源** |
| `/odom_combined` | 局部里程计 | EKF 融合 IMU+轮速，Stage3 使用 |
| `/map` | 全局地图 | SLAM 建图坐标系，Stage3 返程路点在此系 |

---

## 2. Stage 1: 通道导航 + QR 扫码

**包**：`racing_stage1`（只读）
**文件**：`racing_stage1/competition_controller.py`

### 2.1 控制流

```
timer_callback()
  ├── update_phase()      # 从 phase_topic 读取当前阶段
  ├── process_phase()
  │   ├── phase=1: process_phase1()  ← 盲驱 + 避障
  │   ├── phase=2: process_phase2_supervisor()  ← 转发 /stage2_cmd_vel
  │   └── phase=3: process_phase3_supervisor()  ← 转发 Stage3 cmd
  └── publish_control() + publish_feedback()
```

### 2.2 盲驱行为

- 固定线速度 0.2 m/s，不转向（`blind_angular_speed=0`）
- 无位姿依赖，纯开环直行

### 2.3 避障状态机（4态）

```
FORWARD → 障碍物 detected → AVOID_START → 达最小转向角 → AVOID_TURN
  → 窗口无障≥0.25s → AVOID_HOLD → 障碍物清除 → AVOID_RECOVER+counter_steer → FORWARD
```

**聚类算法**：
1. 提取 `phase1_window`(x:0.18~0.85m, ±y:0.22m) 内点云
2. 角度排序，gap > 0.12 rad 切分
3. 过滤 <3 点 / <0.06m / >0.40m 聚类
4. 取最近 x 聚类，**固定左转绕行**

**Recovery + Counter-steer**：
- Recovery：P 控制器（kp=2.4）回正航向，限幅 0.5~1.1 rad/s
- Counter-steer：反方向短时转向抵消惯性

### 2.4 QR 扫码

**包**：`qr_scanner`（辅助）
**后端**：WeChat CV（OpenCV contrib）
**流程**：车到通道特定位置 → 扫描二维码 → 解析方向指令 → 发布到 `qr_scan_result` → competition_controller 设置 phase=2

### 2.5 阶段切换

- Stage1 完成 → `competition_phase` 发布 phase=2
- competition_controller 进入 `process_phase2_supervisor()`：监听 `/stage2_cmd_vel` 并转发到 `/cmd_vel`
- `stage2_cmd_timeout`=0.5s，超时停车

---

## 3. Stage 2: 矩形赛道惯性导航

**包**：`racing_stage2_param_test`（可编辑）

Stage 2 有**两套方案**，均在 `racing_stage2_param_test` 包内：

| 方案 | 核心文件 | 导航方式 | 避障 | 控制输出 |
|---|---|---|---|---|
| **A. DirectInertialTester** | `direct_inertial_tester.py` | 轮速 dead reckoning | S1 6态 | `/stage2_cmd_vel` |
| **B. VisionInertialTester** | `vision_inertial_tester.py` | BPU 视觉 PID 居中 | S1 6态（继承 A） | `/stage2_cmd_vel` |

---

### 方案 A: DirectInertialTester（主方案）

**文件**：`direct_inertial_tester.py`（78 KB, 1772 行）
**启动**：`launch/direct_inertial_test.launch.py`

| 要点 | 说明 |
|---|---|
| **位姿源** | `/odom` 轮速编码器（避免 IMU 漂移） |
| **导航方式** | 开环段序 dead reckoning（里程计+航向积分） |
| **避障** | S1 6态（聚类+边转边绕） |
| **转弯** | 每角独立角度可配（yaml），P 控制追赶 |

#### 配置参数

**文件**：`config/direct_inertial_test.yaml`

| 参数 | 值 | 说明 |
|---|---|---|
| `wheel_odom_topic` | `/odom` | 轮速里程计话题 |
| `navigation_pose_source` | `wheel` | `wheel` / `ekf` |
| `heading_tolerance_deg` | 1.5 | 转向航向容差 |
| `turn_angular_speed` | 0.80 rad/s | 转弯角速度 |
| `turn_min_angular_speed` | 0.18 rad/s | 最小角速度 |
| `turn_kp` | 2.0 | 转弯 P 增益 |
| `rect_enter_align_deg` | 95.0 | 入口对齐角（CW 正转） |
| `rect_corner_1_deg` | 80.0 | 第 1 转角（CW 反转） |
| `rect_corner_2_deg` | 85.0 | 第 2 转角 |
| `rect_corner_3_deg` | 85.0 | 第 3 转角 |
| `rect_corner_4_deg` | 85.0 | 第 4 转角 |
| `avoid_leg2_distance_m` | 0.40 m | 避障 leg2 距离 |

#### 矩形 8 边段序

矩形赛道 **3.42m（长边）× 1.08m（短边）**。

**顺时针段序**：
```
  0. enter_align:  转 rect_enter_align_deg(+95)
  1. leg1:         沿长边 3.42m
  2. corner1:      转 rect_corner_1_deg(-80)
  3. leg2:         沿短边 1.08m
  4. corner2:      转 rect_corner_2_deg(-85)
  5. leg3:         沿长边 3.42m
  6. corner3:      转 rect_corner_3_deg(-85)
  7. leg4:         沿短边 1.08m
  8. corner4:      转 rect_corner_4_deg(-85)
  9. [循环] ... N. exit
```

**逆时针**：所有转角符号相反。

#### S1 6态避障

```
IDLE → 障碍物 → TURN_LEG1(转α) → SETTLE_LEG1 → DRIVE_LEG1(绕行)
  → TURN_LEG2(回正β) → SETTLE_LEG2 → DRIVE_LEG2(回轨) → IDLE
```

#### 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 转弯不足 | `heading_tolerance_deg` | ↓ |
| 转弯过冲 | `turn_kp` | ↓ |
| 某角欠转 | 对应 `rect_corner_N_deg` | ↑ |
| 某角过转 | 对应 `rect_corner_N_deg` | ↓ |
| 入口不对齐 | `rect_enter_align_deg` | ↑↓ |
| 避障不足 | `avoid_leg2_distance_m` | ↑ |

#### 调参历史（2026-06-06）

- 4×90 转弯每角少转 3-5 → 累积偏出赛道
- `heading_tolerance_deg` 4.0→1.5，`turn_angular_speed` 0.75→0.80，`turn_kp` 1.8→2.0
- `rect_enter_align_deg` 90→95，`rect_corner_1_deg` 90→80，其余 90→85

---

### 方案 B: VisionInertialTester（视觉赛道跟随）

**文件**：`vision_inertial_tester.py`（11 KB）
**启动**：`launch/vision_inertial_test.launch.py`

继承自 DirectInertialTester（方案 A）。

| 场景 | 控制方式 | 说明 |
|---|---|---|
| **直线段** | BPU YOLOv8-Seg → 赛道概率图二值化 → 中心线 PID | `lane_follow.py` |
| **转弯段** | 开环定时，`turn_angular_speed`=0.75 rad/s | 与方案 A 相同 |
| **避障** | S1 6态（继承自方案 A） | 与方案 A 相同 |

**模型**：`models/saidao_seg_model_quant.bin`（4.9 MB）

| 项目 | 内容 |
|---|---|
| 架构 | YOLOv8-Seg（best.onnx → HBDK 编译） |
| 芯片 | 地平线 bayes-e（J5 / X5） |
| 量化 | INT8 + INT16 混合 |
| 输入 | 1×3×640×640 NHWC uint8 |
| 输出0 | 检测头（bbox/category/mask coeff） |
| 输出1 | 1×160×160 概率图（已 sigmoid） |
| 后处理 | `mask = output1 > 0.5` → 二值化 → 水平中线 → 偏差 → PID |

**启动**：
```bash
ros2 launch racing_stage2_param_test vision_inertial_test.launch.py
ros2 launch racing_stage2_param_test vision_inertial_test.launch.py test_direction:=counterclockwise
```

---



## 4. Stage 3: 返程导航

**包**：`racing_stage3_param_test`（可编辑）

| 核心文件 | 说明 |
|---|---|
| `stage3_return_navigator.py` | 测试方案主体：Pure Pursuit + 可选 A* |
| `global_path_planner.py` | A* 规划 mixin（TF 转换 / occupancy grid / scan overlay） |
| `phase3_test_trigger.py` | 独立测试时发布 competition_phase=3 |
| `return_track.py` | YAML 路点加载器（备用，目前主流程使用 JSON 参数） |
| `config/return_stage3.yaml` | 参数配置 |

### 4.1 返程路径

路点通过 **JSON 参数** 传入（类似 Stage2 corridor_waypoints_json），分三个来源：

| 参数 | 格式 | 说明 |
|---|---|---|
| `return_start_json` | `[{"x":2.38,"y":3.32,"speed":0.12,"yaw_deg":180.0}]` | 起点（空 = 使用 phase=3 时当前位置） |
| `return_waypoints_json` | `[{"x":1.5,"y":2.5,"speed":0.15},...]` | 可选中间路点（空 = 纯 A*） |
| `return_goal_json` | `[{"x":0.20,"y":0.20,"speed":0.10,"yaw_deg":100.0}]` | P 点终点 |

**默认起点** `(2.38, 3.32) @ yaw=180°` 为 Stage2 整圈终点，速度逐渐衰减到 P 点 `(0.20, 0.20) @ yaw=100°`。

### 4.2 核心参数

| 参数 | 值 | 说明 |
|---|---|---|
| `global_frame_id` | `map` | 全局坐标系 |
| `pure_pursuit_lookahead_m` | 0.45 m | 预瞄距离 |
| `pure_pursuit_linear_speed` | 0.18 m/s | PP 线速度 |
| `pure_pursuit_heading_stop_deg` | 70.0 | 航向差大于此值则原地转向 |
| `pure_pursuit_turn_kp` | 1.8 | PP 转向 P 增益 |
| `return_waypoint_tolerance` | 0.18 m | 中间路点容差 |
| `return_goal_tolerance` | 0.10 m | 目标点容差 |
| `return_goal_yaw_tolerance_deg` | 8.0 | 目标航向容差 |
| `use_occupancy_grid_planner` | true | 启用 A* |
| `planner_replan_period_sec` | 0.25 | A* 重规划周期 |
| `planner_dynamic_obstacle_range_m` | 2.5 | 动态障碍检测范围 |

### 4.3 控制流

```
phase=3 收到
  └─ start_delay_sec 后 → start_return_path()
      └─ control_loop → run_return_path_stage()
          ├─ maybe_advance_waypoint()   // 距当前路点 < tolerance → index++
          ├─ 非末段：
          │   ├─ [A* 启用] plan_global_path() → select_path_lookahead_point()
          │   └─ Pure Pursuit: 航向差 > heading_stop → 原地转; 否则曲率控制
          └─ 末段 + 距目标 < goal_tolerance：
              ├─ 航向差 > goal_yaw_tolerance → 原地对齐
              └─ → finish_mission()
```

### 4.4 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 路点到不了 | `return_waypoint_tolerance` | ↑ |
| P 点停不准 | `return_goal_tolerance` | ↓ |
| P 点航向靠不拢 | `return_goal_yaw_tolerance_deg` | ↑ 放宽 |
| 震荡 | `pure_pursuit_turn_kp` | ↓ |
| A* 频繁重规划耗资源 | `planner_replan_period_sec` | ↑ |
| 无 map 时 A* 阻塞 | `use_occupancy_grid_planner` | false（切纯路点模式）|

---

## 5. 辅助模块

| 模块 | 包 | 功能 |
|---|---|---|
| QR Scanner | `qr_scanner` | WeChat CV 二维码扫描，解析顺/逆时针方向 |
| Racing Vision AI | `racing_vision_ai` | `sign4return=9` 触发→火山引擎大模型图生文 |
| Voice Driver | `voice_driver` | MAE01 模块 + API TTS 语音播报 |

---

## 6. 启动方式汇总

```bash
# 总启动
ros2 launch racing_bringup competition_total.launch.py

# ── Stage2 方案 A: 惯导测试 ──
colcon build --symlink-install --packages-select racing_stage2_param_test
source install/setup.bash
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py test_direction:=counterclockwise

# ── Stage2 方案 B: 视觉赛道跟随 ──
ros2 launch racing_stage2_param_test vision_inertial_test.launch.py
ros2 launch racing_stage2_param_test vision_inertial_test.launch.py test_direction:=counterclockwise

# ── Stage3 返程测试 ──
colcon build --symlink-install --packages-select racing_stage3_param_test
source install/setup.bash
ros2 launch racing_stage3_param_test direct_return_test.launch.py

# 紧急停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## 7. 已知问题

| # | 问题 | 涉及 | 说明 |
|---|---|---|---|
| 1 | `/odom` 轮速 yaw 长距离漂移 | Stage2 A/B | 定期重置航向 |
| 2 | 视觉模型光照敏感，暗场分割不稳定 | Stage2 B | 待优化 |
| 3 | 避障抖动（过渡态） | Stage2 A/B | 调 settle 时间 |
| 4 | A* 频繁重规划耗资源 | Stage3 | 可降频或关闭 |
| 5 | P 点航向靠不拢 | Stage3 | 待调试 |
| 6 | Phase 2→3 切换 cmd_vel 冲突 | 全局 | `phase3_external_control=true` |
| 7 | QR 受光照/角度影响 | Stage1 | 增加重试 |
| 8 | IMU 漂移大 | Stage2 | 已切轮速 `/odom` |
