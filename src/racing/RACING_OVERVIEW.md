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
  │                ├── DirectInertialTester — 轮速里程计 dead reckoning
  │                ├── VisionLaneCentering — 视觉车道居中（mixin）
  │                └── AvoidController — 独立避障模块
  │
  └── phase=3 ──── Stage3: 返程导航
                   │ 测试包：racing_stage3_param_test
                   ├── EnhancedReturnNavigator — Pure Pursuit + A*
                   ├── Stage1 4态避障复用
                   └── 终点 P 点 (0.20, 0.20)
```

### 1.2 关键 Topic 拓扑

| Topic | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/cmd_vel` | Twist | competition_controller (phase1) / Stage3 / twist_cmd_relay | phase1/3 控制输出 |
| `/stage2_cmd_vel` | Twist | DirectInertialTester | phase2 独立控制 → twist_cmd_relay → /cmd_vel |
| `/odom` | Odometry | origincar_base | 轮速里程计（编码器）— **Stage2 主位姿源** |
| `/odom_combined` | PoseWithCovarianceStamped | robot_localization EKF | IMU+轮速融合 — Stage2 仅诊断日志 / Stage3 使用 |
| `/map` | OccupancyGrid | map_overlay | 全局地图 — Stage3 使用 |
| `/scan` | LaserScan | 激光雷达 | 避障输入 |
| `/imu/data` | Imu | BNO055 | **航向角（yaw）来源** — Stage2 角度基准 |
| `competition_phase` | Int32 | competition_controller | 阶段序号（1/2/3） |
| `stage2_state` | String | DirectInertialTester | Stage2 内部状态 |
| `stage3_state` | String | EnhancedReturnNavigator | Stage3 内部状态 |
| `qr_scan_result` | String | qr_scanner | QR 解码结果 |
| `competition_qr_task` | String | competition_controller | QR 扫描方向指令 |
| `sign4return` | Int32 | competition_controller | 返程 AI 触发信号 |
| `/stage2_obstacle_markers` | MarkerArray | DirectInertialTester | 障碍物聚类可视化（rviz2） |
| `/vision_debug` | Image | DirectInertialTester | 视觉车道居中矫正图（rviz2） |

### 1.3 坐标系

| 坐标系 | 类型 | 说明 |
|---|---|---|
| `/odom` | 局部里程计 | 轮速编码器积分，**Stage2 主位姿源**（xy + yaw + 计程 + 控制同源） |
| `/odom_combined` | 局部里程计 | EKF 融合 IMU+轮速，Stage2 仅诊断、Stage3 使用 |
| `/map` | 全局地图 | SLAM 建图坐标系，map→odom 静态变换 (2.50, 2.80) @ 90° |

### 1.4 位姿源规则

- **里程计（`/odom`）**：仅用于位置/距离计数（`x`、`y`、位移）
- **IMU（`/imu/data`）**：用于提供航向角（`yaw`），同时为激光雷达提供角度基准
- **禁止**使用 `/odom` 的角度参与导航计算，角度来源必须为 IMU
- `/odom_combined`（EKF 融合）仅作诊断日志，不参与 Stage2 导航

### 1.5 Package 总览

| 包 | 阶段 | 类型 | 说明 |
|---|---|---|---|
| `racing_stage1` | Stage1 | 只读 | 官方通道导航 + QR 扫码 |
| `racing_stage2` | Stage2 | 只读 | 官方惯导导航基类（`Stage2InertialNavigator`） |
| `racing_stage2_param_test` | Stage2 | **可编辑** | Stage2 参数测试主包 |
| `racing_stage2_field_record` | Stage2 | 辅助 | 场测数据记录 |
| `racing_stage2_param_vision_test` | Stage2 | 辅助 | 视觉参数测试 |
| `racing_stage3` | Stage3 | 只读 | 官方返程导航 |
| `racing_stage3_param_test` | Stage3 | **可编辑** | Stage3 返程测试 |
| `racing_common` | 通用 | 工具 | RacingLogger, ObstacleMarkerPublisher 等 |
| `qr_scanner` | Stage1 | 辅助 | WeChat CV 二维码扫描 |
| `racing_vision_ai` | Stage3 | 辅助 | `sign4return=9` 触发→火山引擎大模型图生文 |
| `voice_driver` | 通用 | 辅助 | MAE01 模块 + API TTS 语音播报 |
| `simple_avoidance` | 通用 | 实验 | 简易避障实验包 |

---

## 2. Stage 1: 通道导航 + QR 扫码

**包**：`racing_stage1`（只读）
**文件**：`racing_stage1/competition_controller.py`（924 行）

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

Stage 2 为**单一 DirectInertialTester**（继承 `Stage2InertialNavigator`），通过 `direct_inertial_tester_vision.py` mixin 可选集成视觉车道居中。VisionInertialTester 方案已废弃（代码移入 `bak/`）。

| 模块 | 核心文件 | 功能 |
|---|---|---|
| **导航** | `direct_inertial_tester.py` | 主控：YAML field_track + odom 欧氏距离 |
| **场测赛道** | `field_track.py` | YAML 赛道段序加载 |
| **避障** | `avoid_controller.py` | 独立 6 态闭环避障控制器 |
| **避障几何** | `avoid_geometry.py` | 绕行路径规划（转向角 + 两脚距离） |
| **雷达处理** | `scan_processor.py` | 前方/侧方障碍检测 |
| **视觉修正** | `direct_inertial_tester_vision.py` | Vision + IMU 融合 mixin |
| **视觉检测** | `vision_lane_centering.py` | BPU YOLOv8-Seg 赛道分割 |
| **日志** | `session_file_log.py` | 文件会话日志 |
| **CSV 记录** | `data_recorder.py` | 遥测 CSV 记录 |
| **指令中继** | `twist_cmd_relay.py` | `/stage2_cmd_vel` → `/cmd_vel` |

### 3.1 配置参数

参数分散在 **3 个 YAML 文件**（加载顺序：`inertial_stage2.yaml` → `direct_inertial_test.yaml` → `avoid_controller.yaml`，后加载覆盖前）：

#### inertial_stage2.yaml — 基础参数（45+ 项）

| 参数 | 值 | 说明 |
|---|---|---|
| `ring_linear_speed` | 0.5 m/s | 环形赛道直行速度 |
| `corridor_linear_speed` | 0.14 m/s | 通道段直行速度 |
| `turn_linear_speed` | 0.10 m/s | 转弯时前向速度 |
| `turn_angular_speed` | 0.65 rad/s | 被 test.yaml 覆盖为 0.80 |
| `turn_kp` | 1.8 | 被 test.yaml 覆盖为 2.0 |
| `heading_kp` | 1.0 | 直行航向保持比例增益 |
| `distance_tolerance` | 0.04 m | 直行到位判据 |
| `heading_tolerance_deg` | 3.5° | 被 test.yaml 覆盖 |
| `segment_timeout` | 25.0 s | 单段超时时间 |
| `corridor_goal` | (2.50, 3.20) @ 90° | 通道终点（入口坐标） |
| `detour_enabled` | true | 避障开关 |
| `fusion_mode_enabled` | true | 纠偏总开关 |
| `fusion_weight_imu` | 0.3 | IMU 融合权重 |
| `fusion_weight_vision` | 0.7 | 视觉融合权重 |
| `vision_model_path` | models/bset.bin | BPU 模型路径 |

#### direct_inertial_test.yaml — 测试覆盖参数

| 参数 | 值 | 说明 |
|---|---|---|
| `navigation_pose_source` | wheel | 位姿源 = 轮速 `/odom` |
| `wheel_odom_topic` | `/odom` | 轮速里程计话题 |
| `heading_tolerance_deg` | 3.0° | 转向航向容差（提前停让惯性自然冲到）|
| `turn_angular_speed` | 0.80 rad/s | 转弯角速度 |
| `turn_min_angular_speed` | 0.30 rad/s | 最小角速度 |
| `turn_kp` | 2.0 | 转弯 P 增益 |
| `turn_slowdown_threshold_deg` | 10.0° | 转弯减速阈值 |
| `turn_min_speed_ratio` | 0.5 | 转弯末端最小角速度比例 |
| `imu_heading_deadzone_deg` | 0.3° | IMU 航向死区 |
| `move_accel_ramp_sec` | 0.5 s | 转弯后加速渐变时长 |

#### avoid_controller.yaml — 避障参数

| 参数 | 值 | 说明 |
|---|---|---|
| `avoid_turn_away_deg` | 30.0° | 从原航向转开角度（第一拐）|
| `avoid_turn_back_deg` | 40.0° | 回原航向另一侧角度（第二拐）|
| `avoid_recover_deg` | 40.0° | 回正转角 |
| `avoid_leg1_distance_m` | 0.22 m | leg1 直行长度 |
| `avoid_leg2_distance_m` | 0.22 m | leg2 直行长度 |
| `side_detour_threshold_m` | 0.18 m | 侧边触发阈值 |
| `avoider_heading_tolerance_deg` | 1.5° | 避障转弯到位精度 |
| `detour_obstacle_distance` | 0.52 m | 直行段前方障碍触发距离 |

### 3.2 赛道段序（field_track YAML）

通过 `field_track_*.yaml` 加载，支持相对坐标（odom 欧氏距离）和世界坐标两种模式。

**顺时针段序**（`field_track_clockwise.yaml`，相对坐标 88° 转角补偿版）：

```
  0. rect_enter_align:   左转 88°（实际 ≈ 90°）
  1. settle:             停稳 0.05s
  2. rect_first_leg:     向左直行 1.05m
  3. settle:             停稳 0.05s
  4. rect_corner_1:     右转 88°（左下拐角）
  5. settle:             停稳 0.05s
  6. rect_side_1:         向上直行 0.40m
  7. settle:             停稳 0.05s
  8. rect_corner_2:     右转 88°（左上拐角）
  9. settle:             停稳 0.05s
  10. rect_top:           向右直行 2.90m
  11. settle:             停稳 0.05s
  12. rect_corner_3:    右转 88°（右上拐角）
  13. settle:             停稳 0.05s
  14. rect_side_2:        向下直行 0.40m
  15. settle:             停稳 0.05s
  16. rect_corner_4:    右转 88°（右下拐角）
  17. settle:             停稳 0.05s
  18. rect_return_origin: 向左直行 1.05m（回起点）
```

**世界坐标版**（`field_track_clockwise_world.yaml`）：标准 90°转角，`rect_side=0.80m`，`rect_return_origin=1.85m`。实际比赛使用。

**导航方式**：
- 相对坐标模式：odom 帧欧氏距离判断直行完成。段起点 = 当前 odom 位置，目标距离 = YAML 的 `distance_m`，完成判据 = `projected_distance() >= target - distance_tolerance`
- 直行航向从 YAML 的 `heading_deg`（正交矩形边方向：90°/0°/-90°/180°）或上一个转弯目标 yaw 继承
- **不需要世界坐标系**：全程在 odom 帧完成，避免 map→odom 变换误差

### 3.3 转弯系统

**闭环 P 控制**：
```
angular = clamp(turn_kp * error, turn_angular_speed)
if abs(error) < turn_min_angular_speed:
    angular = copysign(turn_min_angular_speed, error)
```

**转弯减速**（避免过冲）：
- 剩余角度 < `turn_slowdown_threshold_deg`（10°）时线性衰减至 `turn_min_speed_ratio`（50%）

**加速渐变**（转弯后平滑过渡）：
- 转弯完成 → 0.5s 内线速度从 `turn_linear_speed` 线性渐变到目标速度

**转弯障碍检测**：
- `turn_obstacle_stop_m`（0.25m）：前方过近 → 蠕行转弯（0.02 m/s）
- `corner_approach_m`（0.15m）：段末接近拐角时切换探测距离，避免雷达扫边误触发

**转角补偿**：
- `turn_inertia_compensation_deg`：惯性补偿（提前停）
- `turn_angle_compensation_deg`：系统性转角补偿（IMU/机械零点偏差）

### 3.4 避障控制器（AvoidController）

**独立模块**，6 态闭环：

```
idle → turn_away(转α, 30°) → leg1(直行0.22m) → turn_back(转β, 40°)
  → leg2(直行0.22m) → turn_recover(回正γ, 40°) → fine_align → idle
```

**触发条件**（在 move 段）：
1. 前方障碍 < `detour_obstacle_distance`（0.52m）
2. 侧边空间 < `side_detour_threshold_m`（0.18m）→ 自动触发侧边避障
3. 航向与段航向差 ≤ 12°（否则认为是转弯段不触发）
4. 冷却时间 `detour_cooldown_sec`（3.0s）内不重复触发

**绕行方向**：根据 `front_angle_deg` + 侧边空间选择左/右绕

**leg2 视觉修正**：leg2 直行段全程启用视觉车道居中修正（`_get_vision_angular_for_avoider`）

| 问题 | 参数 | 方向 |
|---|---|---|
| 避障触过早 | `detour_obstacle_distance` | ↓ |
| 绕行幅度不够 | `avoid_leg1/2_distance_m` | ↑ |
| 绕行角度不够 | `avoid_turn_away/back_deg` | ↑ |
| 转弯到位不准 | `avoider_heading_tolerance_deg` | ↑ 放宽 / ↓ 收窄 |
| 侧边误触发 | `side_detour_threshold_m` | ↓ |
| 避障频繁触发 | `detour_cooldown_sec` | ↑ |

### 3.5 视觉车道居中（mixin）

**文件**：`direct_inertial_tester_vision.py` — `DirectInertialTesterVisionMixin`

作为 mixin 混入 `DirectInertialTester`（多重继承），通过 `fusion_mode_enabled` 开关控制：

| 模式 | 策略 | 说明 |
|---|---|---|
| `FUSION` | IMU 30% + Vision 70% | 正常行驶 |
| `IMU_ONLY(VIS_ZERO)` | IMU 100% | 视觉 offset 在死区内 |
| `VIS_ONLY` | Vision 100% | IMU 异常跳变 |
| `IMU_ONLY` | IMU 100% | 视觉检测失效/超时 |
| `LOST` | ω=0 | 全失效，保持直行 |

**后端**：`vision_lane_centering.py` — BPU YOLOv8-Seg → 赛道概率图二值化 → 水平中线偏差 → PID

| 项目 | 内容 |
|---|---|
| 模型 | `models/bset.bin`（地平线 bayes-e BPU 量化） |
| 输入 | 1×640×640 uint8 |
| 输出 | 检测头 + 1×160×160 赛道概率图 |
| 后处理 | `mask = prob > 0.5` → 二值化 → 水平中线 → 偏差 → ω |
| 滑动平均 | 最近 5 帧均值（防止单帧跳变） |

### 3.6 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 直行不到位 | `distance_tolerance` | ↑ |
| 转弯过冲 | `heading_tolerance_deg` / `turn_min_speed_ratio` | ↑（提前停） |
| 转弯不足 | `turn_kp` / `turn_angle_compensation_deg` | ↑ |
| 某角欠转 | `field_track_*.yaml` 对应 `angle_deg` | ↑ |
| 转弯后加速突兀 | `move_accel_ramp_sec` | ↑ |
| 视觉修正打晃 | `vision_offset_kp` / `fusion_weight_imu` | ↓ / ↑ |
| 视觉检测 timeout | `vision_timeout_sec` | ↑ |
| IMU 漂移误检 | `imu_heading_deadzone_deg` | ↑ |
| 避障不足 | `avoid_leg1/2_distance_m` | ↑ |
| 避障抖动 | `avoider_heading_tolerance_deg` | ↑ 放宽 |

---

## 4. Stage 3: 返程导航

**包**：`racing_stage3_param_test`（可编辑）

| 核心文件 | 行数 | 说明 |
|---|---|---|
| `enhanced_return_navigator.py` | 906 | 测试方案主体：Pure Pursuit + A* + Stage1 4态避障 |
| `global_path_planner.py` | — | A* 规划 mixin（TF 转换 / occupancy grid / scan overlay）|
| `phase3_test_trigger.py` | — | 独立测试时发布 competition_phase=3 |
| `stage3_test_simulator.py` | — | 返程测试仿真器 |

**官方包**：`racing_stage3`（只读）

### 4.1 返程路径

路点通过 **JSON 参数** 传入（map 全局坐标系）：

| 参数 | 格式 | 说明 |
|---|---|---|
| `return_start_json` | `[{"x":2.38,"y":3.32,"speed":0.12,"yaw_deg":180.0}]` | 起点（空 = 使用 phase=3 时当前位置）|
| `return_waypoints_json` | `[{"x":1.5,"y":2.5,"speed":0.15},...]` | 可选中间路点（空 = 纯 A*）|
| `return_goal_json` | `[{"x":0.20,"y":0.20,"speed":0.10,"yaw_deg":100.0}]` | P 点终点 |

**默认起点** `(2.38, 3.32) @ yaw=180°` 为 Stage2 整圈终点，速度逐渐衰减到 P 点 `(0.20, 0.20) @ yaw=100°`。

### 4.2 核心参数

| 参数 | 值 | 说明 |
|---|---|---|
| `pursuit_lookahead_m` | 0.45 m | 预瞄距离 |
| `pursuit_linear_speed` | 0.18 m/s | PP 线速度 |
| `pursuit_heading_stop_deg` | 70.0° | 航向差大于此值则原地转向 |
| `pursuit_turn_kp` | 1.8 | PP 转向 P 增益 |
| `waypoint_tolerance` | 0.18 m | 中间路点容差 |
| `goal_tolerance` | 0.10 m | 目标点容差 |
| `goal_yaw_tolerance_deg` | 8.0° | 目标航向容差 |
| `use_occupancy_grid_planner` | true | 启用 A* |
| `planner_replan_period_sec` | 0.25 s | A* 重规划周期 |
| `planner_dynamic_obstacle_range_m` | 2.5 m | 动态障碍检测范围 |

### 4.3 控制流

```
phase=3 收到
  └─ start_delay_sec 后 → start_return_path()
      └─ control_loop → run_return_path_stage()
          ├─ maybe_advance_waypoint()   // 距当前路点 < tolerance → index++
          ├─ 非末段：
          │   ├─ [A* 启用] plan_global_path() → select_path_lookahead_point()
          │   ├─ Pure Pursuit: 航向差 > heading_stop → 原地转; 否则曲率控制
          │   └─ [避障] Stage1 4态聚类避障（interrupt running）
          └─ 末段 + 距目标 < goal_tolerance：
              ├─ 航向差 > goal_yaw_tolerance → 原地对齐
              └─ → finish_mission()
```

### 4.4 状态机

```
idle → armed → running(PurePursuit + A*) → align_yaw → complete
  ↑         running 时可中断为：
  └── avoiding → countersteer → recovering → running
```

### 4.5 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 路点到不了 | `waypoint_tolerance` | ↑ |
| P 点停不准 | `goal_tolerance` | ↓ |
| P 点航向靠不拢 | `goal_yaw_tolerance_deg` | ↑ 放宽 |
| 震荡 | `pursuit_turn_kp` | ↓ |
| A* 频繁重规划耗资源 | `planner_replan_period_sec` | ↑ |
| 无 map 时 A* 阻塞 | `use_occupancy_grid_planner` | false（切纯路点模式）|
| 避障误触发 | `avoid_min_turn_angle_deg` / `safe_distance` | 调阈值 |

---

## 5. 辅助模块

| 模块 | 包 | 功能 |
|---|---|---|
| QR Scanner | `qr_scanner` | WeChat CV 二维码扫描，解析顺/逆时针方向 |
| Racing Vision AI | `racing_vision_ai` | `sign4return=9` 触发→火山引擎大模型图生文 |
| Voice Driver | `voice_driver` | MAE01 模块 + API TTS 语音播报 |
| RacingLogger | `racing_common` | 统一日志工具（所有节点共用） |
| ObstacleMarkerPublisher | `racing_common` | 障碍物可视化 marker（rviz2） |

---

## 6. 启动方式汇总

```bash
# 总启动
ros2 launch racing_bringup competition_total.launch.py

# ── Stage2 惯导测试 ──
colcon build --symlink-install --packages-select racing_common racing_stage2_param_test
source install/setup.bash
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py test_direction:=counterclockwise

# ── Stage2 带视觉车道居中 ──
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py \
  vision_camera:=true

# ── Stage3 返程测试 ──
colcon build --symlink-install --packages-select racing_stage3_param_test
source install/setup.bash
ros2 launch racing_stage3_param_test enhanced_return_test.launch.py

# ── 可视化调试 ──
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py \
  enable_rviz:=true

# 紧急停车
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### Stage2 启动参数

| 参数 | 默认 | 说明 |
|---|---|---|
| `test_direction` | clockwise | 行驶方向 |
| `test_start_mode` | auto | 启动模式 |
| `field_track_yaml` | '' | 自定义赛道 YAML 路径 |
| `include_support` | true | 是否启动底层支持（轮速/IMU/雷达）|
| `include_bringup` | true | 是否启动 origincar_bringup |
| `include_lidar` | true | 是否启动激光雷达 |
| `include_camera` | false | 是否启动普通摄像头 |
| `include_recorder` | true | 是否启动 CSV 记录 |
| `vision_camera` | true | 是否启动 Aurora 930 视觉相机 |
| `enable_cmd_relay` | true | 是否启动 cmd_vel 中继（stage2→主控）|
| `enable_rviz` | false | 是否启动 rviz2 |
| `carto_slam` | false | 是否启动 Cartographer SLAM |

---

## 7. 已知问题

| # | 问题 | 涉及 | 说明 |
|---|---|---|---|
| 1 | corner_2/3 转弯过冲（多转 1.6-2.7°） | Stage2 | 惯性造成，`heading_tolerance_deg`=3.0° 提前停，`turn_min_speed_ratio`=0.5 减缓末端 |
| 2 | 避障 after-avoid 直行段抖动 | Stage2 | `avoider_heading_tolerance_deg` 放宽可缓解；leg2 视觉修正可能引入反相修正 |
| 3 | 视觉模型光照敏感，暗场分割不稳定 | Stage2 Vision | 待优化；IMU 100% 降级策略缓解 |
| 4 | A* 频繁重规划耗资源 | Stage3 | 可降频或关闭 `use_occupancy_grid_planner` |
| 5 | P 点航向靠不拢 | Stage3 | 放宽 `goal_yaw_tolerance_deg` |
| 6 | Phase 2→3 切换 cmd_vel 冲突 | 全局 | twist_cmd_relay 和 competition_controller 的 supervisor 机制需协调 |
| 7 | QR 受光照/角度影响 | Stage1 | 增加重试机制 |
| 8 | odom 轮速滑移误差累积 | Stage2 | 短段（0.40m）影响大，长段（2.90m）可控；`distance_tolerance` 权衡 |
| 9 | 赛道 rect_side 0.40m 过短，避障空间不足 | Stage2 | 考虑加长侧边或进赛道前预判 |
| 10 | 视觉模型路径 `bset.bin` 硬编码 | Stage2 Vision | `vision_model_path` 参数可覆盖，但默认路径依赖文件存在 |