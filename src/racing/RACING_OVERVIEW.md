# Racing 三阶段方案总览

> **编辑约束**：本文档位于 `src/racing/RACING_OVERVIEW.md`。
> Stage 2 / Stage 3 **官方生产代码**在 `racing_stage2` / `racing_stage3`。
> `racing_stage2_param_test` / `racing_stage3_param_test` 仅用于独立调参，不作为 total 正式入口。
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
  │                │ 官方包：racing_stage2
  │                ├── Stage2InertialNavigator — 轮速惯导 + field_track
  │                ├── Stage2VisionMixin — 视觉中线跟随
  │                └── AvoidController — 独立避障模块
  │
  └── phase=3 ──── Stage3: 返程导航
                   │ 官方包：racing_stage3
                   ├── Stage3ReturnNavigator — Pure Pursuit + A* + P 视觉
                   ├── Stage1 4态避障复用
                   └── 终点 P 点区域
```

### 1.2 关键 Topic 拓扑

| Topic | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/cmd_vel` | Twist | competition_controller (phase1) / Stage3 / twist_cmd_relay | phase1/3 控制输出 |
| `/stage2_cmd_vel` | Twist | Stage2InertialNavigator | phase2 独立控制 → Stage1 转发 /cmd_vel |
| `/odom` | Odometry | origincar_base | 轮速里程计（编码器）— **Stage2 主位姿源** |
| `/odom_combined` | PoseWithCovarianceStamped | robot_localization EKF | IMU+轮速融合 — Stage2 仅诊断日志 / Stage3 使用 |
| `/map` | OccupancyGrid | map_overlay | 全局地图 — Stage3 使用 |
| `/scan` | LaserScan | 激光雷达 | 避障输入 |
| `/imu/data` | Imu | BNO055 | **航向角（yaw）来源** — Stage2 角度基准 |
| `competition_phase` | Int32 | competition_controller | 阶段序号（1/2/3） |
| `stage2_state` | String | Stage2InertialNavigator | Stage2 内部状态 |
| `stage3_state` | String | Stage3ReturnNavigator | Stage3 内部状态 |
| `qr_scan_result` | String | qr_scanner | QR 解码结果 |
| `competition_qr_task` | String | competition_controller | QR 扫描方向指令 |
| `sign4return` | Int32 | competition_controller | 返程 AI 触发信号 |
| `/stage2_obstacle_markers` | MarkerArray | Stage2InertialNavigator | 障碍物聚类可视化（rviz2） |
| `/vision_debug` | Image | Stage2InertialNavigator | 视觉车道居中矫正图（rviz2） |

### 1.3 坐标系

| 坐标系 | 类型 | 说明 |
|---|---|---|
| `/odom` | 局部里程计 | 轮速编码器积分，**Stage2 主位姿源**（xy + yaw + 计程 + 控制同源） |
| `/odom_combined` | 局部里程计 | EKF 融合 IMU+轮速，Stage2 仅诊断、Stage3 使用 |
| `/map` | 全局地图 | 全局地图坐标系；`map→odom_combined` 静态变换由 launch 参数注入（默认 0.50, 0.20 @ ~10°） |

### 1.4 位姿源规则

- **Stage1 通道导航位置使用 TF `map <- base_footprint`**（目标点是 map 坐标，不能直接拿 `/odom_combined` xy）
- **里程计（`/odom`）**：仅用于位置/距离计数（`x`、`y`、位移）
- **IMU（`/imu/data`）**：用于提供航向角（`yaw`），同时为激光雷达提供角度基准
- **禁止**使用 `/odom` 的角度参与导航计算，角度来源必须为 IMU
- `/odom_combined`（EKF 融合）仅作诊断日志，不参与 Stage2 导航

### 1.5 Package 总览

| 包 | 阶段 | 类型 | 说明 |
|---|---|---|---|
| `racing_stage1` | Stage1 | 只读 | 官方通道导航 + QR 扫码 |
| `racing_stage2` | Stage2 | **官方生产** | 正式惯导+视觉节点（`Stage2InertialNavigator`） |
| `racing_stage2_seg_follow` | Stage2 | 独立实验 | 一键启动底盘+相机的 SEG 中线跟线包，网页显示裁剪ROI，按左右边线中点发布低速控制 |
| `racing_stage2_param_test` | Stage2 | 独立调参 | Stage2 参数测试包（非 total 入口） |
| `racing_stage2_field_record` | Stage2 | 辅助 | 场测数据记录 |
| `racing_stage2_param_vision_test` | Stage2 | 辅助 | 视觉参数测试 |
| `racing_stage3` | Stage3 | **官方生产** | 正式返程节点（`Stage3ReturnNavigator`） |
| `racing_stage3_param_test` | Stage3 | 独立调参 | Stage3 参数测试包（非 total 入口） |
| `racing_common` | 通用 | 工具 | RacingLogger, ObstacleMarkerPublisher 等 |
| `racing_tools` | 通用 | 诊断工具 | 数据记录、相机录像、初始 scan-to-map 位姿估计 |
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

### 2.5 通道导航（map 自由空间区域进入）

- 位姿：`TF map <- base_footprint`（xy）+ IMU yaw
- 目标：入口区域中心（当前默认 map `(2.50, 2.00)`，到“2 米杠/通道口”即放行），**不要求精准到点 / 精准 90°**
- 规划：默认 `use_corridor_planner=true`，占用膨胀 + A* 规划自由空间路径；失败回退直线
- 跟踪：Pure Pursuit 跟踪规划路径（可斜穿）；`left_recover` 仅在 map_x 过大时介入
- 放行：进入入口区域半径（当前 `0.10m`）即切 Stage2；默认不强制航向，可选 `corridor_require_yaw_for_release`
- 超时：`corridor_timeout_sec` 到时策略放行 Stage2
- 日志：`~/dev_ws/log/competition_stage1/latest.log` 输出 plan refresh / region_entry 细节

### 2.6 阶段切换

- Stage1 完成 → `competition_phase` 发布 phase=2
- competition_controller 进入 `process_phase2_supervisor()`：监听 `/stage2_cmd_vel` 并转发到 `/cmd_vel`
- `stage2_cmd_timeout`=0.5s，超时停车

---

## 3. Stage 2: 矩形赛道惯性导航

**包**：`racing_stage2`（官方生产）

Stage 2 为**单一 Stage2InertialNavigator**（继承 `Stage2InertialBase` + 视觉 mixin），通过 `direct_inertial_tester_vision.py` mixin 可选集成视觉车道居中。VisionInertialTester 方案已废弃（代码移入 `bak/`）。

| 模块 | 核心文件 | 功能 |
|---|---|---|
| **导航** | `stage2_inertial_navigator.py` | 主控：YAML field_track + odom 欧氏距离 |
| **场测赛道** | `field_track.py` | YAML 赛道段序加载 |
| **避障** | `avoid_controller.py` | 独立 6 态闭环避障控制器 |
| **避障几何** | `avoid_geometry.py` | 绕行路径规划（转向角 + 两脚距离） |
| **雷达处理** | `scan_processor.py` | 前方/侧方障碍检测 |
| **视觉修正** | `stage2_vision_mixin.py` | 视觉中线主 + IMU 兜底 mixin |
| **视觉检测** | `vision_lane_centering.py` | BPU YOLOv8-Seg + 多行中线/前瞻/曲率 |
| **SEG跟线实验** | `racing_stage2_seg_follow` | 独立包，启动底盘+相机，网页只显示裁剪ROI，用左右赛道边线中点跟线，默认低速发布 `/cmd_vel` |
| **日志** | `session_file_log.py` | 文件会话日志 |
| **CSV 记录** | `data_recorder.py` | 遥测 CSV 记录 |
| **指令中继** | `twist_cmd_relay.py` | `/stage2_cmd_vel` → `/cmd_vel` |

### 3.1 配置参数

参数分散在 **3 个 YAML 文件**（加载顺序：`inertial_stage2.yaml` → `direct_inertial_test.yaml` → `avoid_controller.yaml`，后加载覆盖前）：

#### inertial_stage2.yaml — 基础参数（45+ 项）

| 参数 | 值 | 说明 |
|---|---|---|
| `ring_linear_speed` | 0.32 m/s | 环形赛道直行速度（当前降速纯惯导） |
| `corridor_linear_speed` | 0.14 m/s | 通道段直行速度 |
| `turn_linear_speed` | 0.07 m/s | 转弯时前向速度 |
| `entry_45_arc.steering_angle_deg` | 顺时针 +8° / 逆时针 -8° | 通道口先拐入短直道的圆弧舵角 |
| `entry_45_arc.duration_sec` | 4.0 s | 通道口先拐入短直道的圆弧时间（调成约45°） |
| `entry_45_arc.speed` | 0.16 m/s | 通道口先拐入短直道的圆弧速度 |
| `entry_short_straight.distance_m` | 0.25 m | 45°圆弧后短直道距离 |
| `entry_semicircle.steering_angle_deg` | 顺时针 -8° / 逆时针 +8° | 入口连续半圆弧舵角 |
| `entry_semicircle.duration_sec` | 8.0 s | 入口连续半圆弧持续时间 |
| `entry_semicircle.speed` | 0.18 m/s | 入口连续半圆弧速度 |
| `heading_kp` | 1.0 | 直行航向保持比例增益 |
| `distance_tolerance` | 0.04 m | 直行到位判据 |
| `heading_tolerance_deg` | 3.5° | 被 test.yaml 覆盖 |
| `segment_timeout` | 25.0 s | 单段超时时间 |
| `corridor_goal` | (2.50, 3.20) @ 90° | 通道终点（入口坐标） |
| `detour_enabled` | true | 避障开关 |
| `vision_offset_correction_enabled` | false | 当前先不启用 SEG 中线横向修正 |
| `vision_length_correction_enabled` | false | 当前先不启用 SEG 纵向定长/剩余距离修正 |
| `vision_turn_assist_enabled` | false | 当前先不启用 SEG 拐弯完成辅助 |
| `fusion_mode_enabled` | true | 保留 IMU 航向修正链路，视觉关闭时为 IMU-only |
| `fusion_weight_imu` | 0.75 | IMU/惯导主控权重 |
| `fusion_weight_vision` | 0.25 | 视觉轻量修正权重 |
| `vision_model_path` | models/bset.bin | BPU 模型路径 |

#### direct_inertial_test.yaml — 测试覆盖参数

| 参数 | 值 | 说明 |
|---|---|---|
| `navigation_pose_source` | wheel | 位姿源 = 轮速 `/odom` |
| `wheel_odom_topic` | `/odom` | 轮速里程计话题 |
| `heading_tolerance_deg` | 3.0° | 转向航向容差（提前停让惯性自然冲到）|
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

**顺时针段序**（`field_track_clockwise.yaml`，固定舵角圆弧入口版）：

```
  0. entry_45_arc:         从 map(2.50,2.00) 先圆弧拐入短直道（约45°）
  1. entry_short_straight: 在短直道上走一小段
  2. entry_semicircle:     连续小舵角半圆弧，直接揉进长直道
  3. rect_top:             长直道
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

### 3.5 视觉中线跟随 + 圆弧过角

**文件**：
- `stage2_vision_mixin.py` — `Stage2VisionMixin`
- `vision_lane_centering.py` — BPU Seg + 多行中线
- `field_track_*.yaml` — `move`/`arc` 段序（已去掉角落 pause 原地拧）

#### 视觉算法（正式主策略）
1. ROI：下方 `vision_crop_ratio` + 左右裁 `vision_crop_side_ratio`
2. BPU YOLOv8-Seg → 赛道 mask
3. 多行采样中点拟合中心线，无效行剔除
4. 前瞻点误差 `e` + 近远场曲率 `curve`
5. 控制：`ω = -(Kp*e + Kd*ė + Kc*curve)`，`v` 随误差/曲率降速（move 段）
6. 丢线：IMU 段航向兜底；默认**不因短预算永久闭嘴**

| 模式 | 策略 | 说明 |
|---|---|---|
| `VIS_PRIMARY` | 视觉 100% | 中线有效（主模式） |
| `IMU_ONLY(VIS_CENTER/ZERO)` | IMU 100% | 已居中或视觉ω≈0 |
| `IMU_ONLY(VIS_LOST)` | IMU 100% | 视觉失效/超时 |
| `FUSION` | 可配权重 | `vision_primary_control=false` 时 |
| `LOST` | ω=0 | 视觉与 IMU 都不可用 |

#### 段模型
- `move`：视觉中线主，轮速里程判段完成
- `arc`：定时舵角段，仅使用 YAML 的 `steering_angle_deg` 和 `duration_sec`；生产直连底盘将舵角值原样下发，固定舵角保持到计时完成，然后发送 `0°` 回正，不使用车体角速度、半径或弧长判定
- 短边 `rect_side_*` 保留为短 `move`
- 入口 `rect_enter_align` 与四个角位均为 `±15° / 6.0s`；弧段结束立即回正，线速度保持 `turn_linear_speed`

#### 日志排查
- Stage2 会话日志：`~/dev_ws/log/competition_stage2/latest.log`
- 关键 tag：`SEGMENT` / `PLAN` / `VISION_CTRL` / `FUSION_STATE` / `ARC_SETUP` / `ARC_TIMER` / `ARC_COMPLETE` / `TELEM`
- 视觉预览：HTTP `:8082` + `/vision_debug`

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

**包**：`racing_stage3`（官方生产）

| 核心文件 | 行数 | 说明 |
|---|---|---|
| `stage3_return_navigator.py` | — | 官方返程主体：Pure Pursuit + A* + Stage1 4态避障 + P 视觉 |
| `global_path_planner.py` | — | A* 规划 mixin（TF 转换 / occupancy grid / scan overlay）|
| `phase3_test_trigger.py` | — | 独立测试工具，正式 total 不启动 |
| `stage3_test_simulator.py` | — | 返程测试工具，正式 total 不启动 |

**独立调参包**：`racing_stage3_param_test`

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
          └─ 仅当 map y < p_vision_enable_y_max(默认 2.0) 后启动 P 视觉：
              ├─ YOLO 连续检测到 P 点
              ├─ p_approaching：按 P 框中心低通纠偏并加速接近
              ├─ bbox fill 达阈值：沿当前行驶方向额外前进 0.50m
              └─ → finish_mission()
```

### 4.4 状态机

```
idle → armed → running(PurePursuit + A*) → p_approaching → p_extra_forward → complete
  ↑         running 时可中断为：
  └── avoiding → countersteer → recovering → running
```

### 4.5 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 路点到不了 | `waypoint_tolerance` | ↑ |
| P 点停不准 | `goal_tolerance` | ↓ |
| P 点航向靠不拢 | `goal_yaw_tolerance_deg` | ↑ 放宽 |
| 过早误检 P / 太晚才开视觉 | `p_vision_enable_y_max` | ↑ 更早开 / ↓ 更晚开 |
| P 点视觉接近太慢 | `p_approach_linear_speed` | ↑ |
| P 点接近左右抖 | `p_approach_angular_kp` / `p_approach_angular_deadband` | ↓ / ↑ |
| P 点停车过早 | `p_extra_forward_distance_m` | ↑ |
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
| InitialScanMapLocalizer | `racing_tools` | `/scan` 与 `/map` 边缘匹配，输出初始 `(x,y,yaw,confidence)` 和 RViz marker；默认不发布 TF |

---

## 6. 启动方式汇总

```bash
# 总启动
ros2 launch racing_bringup competition_total.launch.py

# ── Stage2 惯导测试 ──
colcon build --symlink-install --packages-select racing_common racing_stage2_param_test
source install/setup.bash
ros2 launch racing_stage2 competition_stage2.launch.py
ros2 launch racing_stage2 competition_stage2.launch.py test_direction:=counterclockwise

# ── Stage2 带视觉车道居中 ──
ros2 launch racing_stage2 competition_stage2.launch.py \
  vision_camera:=true

# ── Stage3 返程测试 ──
colcon build --symlink-install --packages-select racing_stage3_param_test
source install/setup.bash
ros2 launch racing_stage3 competition_stage3.launch.py

# ── 可视化调试 ──
ros2 launch racing_stage2 competition_stage2.launch.py \
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

## 2026-07-17 场测修复要点
- Stage1：A* 条件重规划 + 占用栅格缓存；区域半径约 0.40m；不要求 90° 精对准。
- Stage2：角落 `arc` 以航向进度结束；短边保留；视觉中线主控带 conf/rows 质量门，弯中不抢舵。
- 排查日志：`~/dev_ws/log/competition_stage1/latest.log`、`~/dev_ws/log/competition_stage2/latest.log`。
