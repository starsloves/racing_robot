# Racing 三阶段方案总览

> **编辑约束**：本文档位于 `src/racing/RACING_OVERVIEW.md`。
> Stage 2 / Stage 3 **官方生产代码**在 `racing_stage2` / `racing_stage3`。
> `racing_stage2_param_test` / `racing_stage3_param_test` 仅用于独立调参，不作为 total 正式入口。
> **本文档必须随阶段方案变更同步更新。**

---

## 1. 系统架构总览

### 1.1 HTTP 监控端口互斥规则

三个阶段虽然由总启动常驻加载，但视觉 HTTP 服务严格随 `competition_phase` 互斥：Stage1 仅在 phase=1 绑定 **8081**，Stage2 仅在 phase=2 绑定 **8082**，Stage3 仅在 phase=3 绑定 **8083**。阶段切换时，离开阶段会先停止 HTTP server 并释放其端口；Stage3 的通道 YOLO 只做内部重定位，**不提供 8081 HTTP 服务**。根路径均提供同一 `vision_viewer.html`，页面只轮询当前访问的端口并只显示对应阶段。

### 1.2 三阶段流转

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
  │                ├── Stage2InertialNavigator — /odom_combined 距离 + IMU 转角轨迹控制
  │                ├── Stage2VisionMixin — 视觉中线跟随
  │                └── AvoidController — 独立避障模块
  │
  └── phase=3 ──── Stage3: 返程导航
                   │ 官方包：racing_stage3
                   ├── Stage3ReturnNavigator — 通道对中 + 地图粗导航 + P 视觉最终到达
                   ├── Stage1 4态避障复用
                   └── 终点 P 点区域
```

### 1.3 关键 Topic 拓扑

| Topic | 类型 | 发布者 | 说明 |
|---|---|---|---|
| `/cmd_vel` | Twist | competition_controller (phase1) / Stage3 / twist_cmd_relay | phase1/3 控制输出 |
| `/stage2_cmd_vel` | Twist | Stage2InertialNavigator | phase2 独立控制 → Stage1 转发 /cmd_vel |
| `/odom` | Odometry | origincar_base | 轮速里程计；Stage2 仅用于启动静止判定和诊断 |
| `/odom_combined` | Odometry | robot_localization EKF | Stage2 里程距离源（仅相邻 xy 欧氏距离）及 Stage3 定位源 |
| `/map` | OccupancyGrid | map_overlay | 全局地图 — Stage3 使用 |
| `/scan` | LaserScan | 激光雷达 | 避障输入 |
| `/imu/data` | Imu | BNO055 | **航向角（yaw）来源** — Stage2 角度基准 |
| `competition_phase` | Int32 | competition_controller | 阶段序号（1/2/3） |
| `stage2_state` | String | Stage2InertialNavigator | Stage2 内部状态 |
| `stage3_state` | String | Stage3ReturnNavigator | Stage3 内部状态 |
| `stage2_ai_capture` | Empty | Stage2InertialNavigator | 长直道末端图像分析一次性触发 |
| `ai_description` | String | racing_vision_ai | 云端图生文结果，供语音节点异步播报 |
| `stage2_ai_status` | String | racing_vision_ai | 截帧/请求/结果状态诊断 |
| `qr_scan_result` | String | qr_scanner | QR 解码结果 |
| `competition_qr_task` | String | competition_controller | QR 扫描方向指令 |
| `sign4return` | Int32 | competition_controller | 返程 AI 触发信号 |
| `/stage2_obstacle_markers` | MarkerArray | Stage2InertialNavigator | 障碍物聚类可视化（rviz2） |
| `/vision_debug` | Image | Stage2InertialNavigator | 视觉车道居中矫正图（rviz2） |

### 1.4 坐标系

| 坐标系 | 类型 | 说明 |
|---|---|---|
| `/odom` | 局部里程计 | 轮速编码器积分，仅用于 Stage2 启动静止判定和诊断 |
| `/odom_combined` | 局部里程计 | EKF 融合 IMU+轮速；Stage2 仅使用相邻 xy 的欧氏距离，不使用其 orientation |
| `/map` | 全局地图 | 全局地图坐标系；`map→odom_combined` 静态变换由 launch 参数注入（默认 0.50, 0.20 @ ~10°） |

### 1.5 位姿源规则

- **Stage1 通道导航位置使用 TF `map <- base_footprint`**（目标点是 map 坐标，不能直接拿 `/odom_combined` xy）
- **里程距离（`/odom_combined`）**：仅使用相邻位置的欧氏位移累计距离；不得使用其 orientation
- **IMU（`/imu/data`）**：用于提供航向角（`yaw`），同时为激光雷达提供角度基准。Stage1 首帧原始 IMU yaw 映射到 YAML `imu_initial_map_yaw_deg`（当前 10°），后续仅累计原始 IMU 相对转角；所有 Stage1 状态使用此处理后的 yaw。
- **禁止**使用 `/odom` 的角度参与导航计算，角度来源必须为 IMU
- `/odom` 的 orientation 与 `/odom_combined` 的 orientation 均不参与 Stage2 导航

### 1.6 Package 总览

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

- 默认固定线速度 0.2 m/s、不转向（`blind_angular_speed=0`）盲驱。
- 若二维码仍未识别且 `/odom_combined` 的 `odom_x > 3.5m`，进入二维码左侧搜索：降速至
  `0.12m/s` 并以 `+0.55rad/s` 左转；二维码回调后立即退出该搜索分支。
- 左侧搜索只作用于 Stage1 `forward` 状态，不影响避障、后退和通道导航。

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
- 目标与门线：Stage1 -> Stage2 的交接目标、横向窗口、门线前后范围和航向容差均只读取 `stage1_controller.yaml`；总览不固化具体坐标。相机/YOLO 仅可辅助通道居中，不能替代 YAML 门线交接判定。
- 规划：默认 `use_corridor_planner=true`，占用膨胀 + A* 规划自由空间路径；失败回退直线
- 跟踪：Pure Pursuit 跟踪规划路径（可斜穿）；`left_recover` 仅在 map_x 过大时介入
- 地图通道视觉居中：倒退结束进入 A* + Pure Pursuit 后，YOLO 保持推理；仅对新鲜且置信度达标的 bbox 水平偏移叠加死区、低通和限幅后的微小角速度。路径、速度和门线交权仍完全由地图 + IMU 决定，视觉不能单独切 Stage2。
- 通道 YOLO 交接：二维码触发后，倒退阶段立即启用 YOLO；连续 `channel_yolo_confirm_frames` 个有效框后才接管，防止单帧误检。接管先以 `channel_yolo_align_speed` 和 IMU 对齐 `channel_handoff_yaw_deg`，误差小于 `channel_yolo_align_tolerance_deg` 后才以 `channel_yolo_chase_speed` 快速沿 +Y 接近；YOLO 仅提供水平误差，IMU 是唯一 yaw 来源。
- 末端交接：通道导航先满足 `corridor_entry_region_radius_m` 圆形区域，再要求 `map_y >= corridor_release_min_y_m` 才允许交给 Stage2；这条 Y 门线防止圆形半径在目标前提前触发交权。航向是否参与交权由 `corridor_require_yaw_for_release` 控制，坐标和门限均以 `stage1_controller.yaml` 为唯一来源。
- 倒退：二维码回调后立即进入记录路径倒退，不再先发送零速度制动。路径记录和倒序路径前瞻追踪均使用 `/odom_combined` 的位置；车尾追踪来时轨迹上的前瞻点，IMU 仅计算该几何目标对应的车头反向航向，禁止直接锁定历史记录 yaw。`back_target_x` 的截止坐标系以 `stage1_controller.yaml` 和运行日志为准。
- 图像监控：通道 YOLO 在且仅在 `competition_phase=1` 时绑定 YAML `channel_yolo_http_port`（默认 8081）；离开 Stage1 立即关闭服务并释放端口。`/channel_raw.jpg`、`/channel_yolo.jpg` 为单帧，`/stream_raw.mjpg`、`/stream.mjpg` 为实时流，`/health` 提供帧数、帧龄和推理状态。根路径网页同时显示原图与检测流。旧分割模块不再抢占 8081。模型、速度、门线、容差和图像路径均以 `stage1_controller.yaml` 为唯一来源。
- 相机信息话题、停车距离、位置容差、速度、角速度和航向增益均以 `stage1_controller.yaml` 为准。
- 超时：`corridor_timeout_sec` 到时停车等待，不绕过 map 目标直接放行 Stage2
- 日志：`~/dev_ws/log/competition_stage1/latest.log` 输出 plan refresh / region_entry 细节

### 2.6 阶段切换

- Stage1 完成 → `competition_phase` 发布 phase=2
- competition_controller 进入 `process_phase2_supervisor()`：监听 `/stage2_cmd_vel` 并转发到 `/cmd_vel`
- `stage2_cmd_timeout`=0.5s，超时停车
- Stage2 `start_delay_sec`=0.0s、`start_stationary_hold_sec`=0.0s；phase=2 后输入齐全且底盘未明显运动即启动，不再额外 0.5s 停顿。
- Stage2 若在 phase=2 已发布后才启动或重启，会从 latched phase=2 进入恢复武装态；仍须等到二维码方向、IMU 和轮速输入齐全才实际出车。

---

## 3. Stage 2: 矩形赛道惯性导航

**包**：`racing_stage2`（官方生产）

Stage 2 为**单一 Stage2InertialNavigator**（继承 `Stage2InertialBase` + 视觉 mixin），通过 `direct_inertial_tester_vision.py` mixin 可选集成视觉车道居中。VisionInertialTester 方案已废弃（代码移入 `bak/`）。

| 模块 | 核心文件 | 功能 |
|---|---|---|
| **导航** | stage2_inertial_navigator.py | 主控：生产段序固定在 Stage2TrackController；stage2_controller.yaml 的 track_* 为唯一调参入口 |
| **避障** | `avoid_controller.py` | 独立 6 态闭环避障控制器 |
| **避障几何** | `avoid_geometry.py` | 绕行路径规划（转向角 + 两脚距离） |
| **雷达处理** | `scan_processor.py` | 前方/侧方障碍检测 |
| **SEG 适配** | `stage2_vision_mixin.py` | BPU 分割模型初始化与中线状态输出 |
| **生产控制器** | `stage2_hybrid_controller.py` | SEG 连续帧证据 + IMU 相对转角的唯一控制权状态机 |
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
| `turn_obstacle_stop_m` | 0.0 m | Stage2 生产轨迹 front_obstacle 硬停车阈值；0 表示关闭，避免圆弧扫边误停 |
| `detour_obstacle_distance` | 0.52 m | 直行段前方障碍触发距离 |

### 3.2 赛道段序（生产 Stage2TrackController）

Stage2 生产段序固定在 Stage2TrackController 中，所有可调参数统一写在
src/racing/racing_stage2/config/stage2_controller.yaml 的 track_* 段。
旧顺/逆时针赛道 YAML 已删除，生产不再读取赛道 YAML。

固定段序：

```
entry_arc → entry_medium → left_side_arc → top_long → right_side_arc → exit_medium
```

关键调参入口：
- track_entry_arc_complete_lead_deg：入口 90°弯提前切段角。
- track_entry_medium_distance_m：第一个 180° 前短直距离，控制 entry_medium → left_side_arc 的触发点。
- track_top_long_distance_m：第二个 180° 前长直距离，控制 top_long → right_side_arc 的触发点。
- track_exit_medium_distance_m：最后出口直线距离，控制 exit_medium 完成点。
- track_max_speed：直线速度。
- track_corner_speed：后续 180°弯线速度。
- track_entry_angular：入口 90°弯角速度。
- track_corner_angular：后续 180°弯角速度。
- track_corner_arc_complete_lead_deg：后续 180°弯提前切段角。

导航方式：
- 距离：/odom_combined 相邻 xy 欧氏位移累计。
- 转角：/imu/data 相对 yaw 累计。
- 禁止使用 /odom 或 /odom_combined 的 orientation 参与航向控制。

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

**TrackController 圆弧结束判据（当前生产实现）**：
- 转弯完成以 IMU 相对转角为主判据；`/odom_combined` 相邻 xy 累计弧长只作为“已走过足够距离”和“严重失配”保护，不要求弧长与角度在同一采样点同时命中。
- 当弧长达到目标弧长的 `track_arc_min_completion_ratio`（默认 70%）后，若 IMU 相对转角已进入容差，或按当前 yaw-rate 预判 `track_arc_finish_predict_sec`（默认 0.18s）内会越过目标角，即立即移交下一段。
- 若弧长超过目标弧长 + overrun，但 IMU 角度仍落后超过 `track_arc_mismatch_angle_deg`（默认 14°），才判定 `*_distance_yaw_mismatch` 停车；否则切入下一段并由直线航向/yaw-rate 阻尼吸收余摆。
- `entry_align` 是非零速度的短距离视觉参考校正段；视觉不能在距离上限内完全居中时，使用最佳视觉误差对应的 IMU 航向继续进入 `entry_medium`，不再因为未居中而长时间卡住。

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

### 3.5 SEG-IMU 混合闭环过角（生产策略）

**文件**：
- `stage2_hybrid_controller.py` — 生产唯一控制权状态机
- `stage2_vision_mixin.py` / `vision_lane_centering.py` — BPU Seg + 多行中线状态输出
- `stage2_controller.yaml` — Stage2 生产唯一参数源，`track_*` 段控制赛道速度、距离、角速度和提前切段

#### 状态机与职责

```
ENTRY_COMMIT → EXIT_CAPTURE → STRAIGHT_TRACK → PRETURN → TURN_COMMIT
       ↑             │                             │            │
       └─────────────┴─────────────────────────────┴────────────┘
                                      │
                               VISION_HOLD → SAFE_STOP
```

1. `STRAIGHT_TRACK`：SEG 多行中线的误差/曲率只做小幅跟踪，保持巡航速度。
2. `PRETURN`：远端截断、前边界或曲率证据连续 `hybrid_turn_confirm_frames` 帧才提前掺入固定环向前馈；证据消失立即撤销。
3. `TURN_COMMIT`：以入口或环弯的相对 IMU 转角为硬界限。SEG 不能单独结束转弯；到 `hybrid_brake_start_deg` 后按 yaw-rate 主动减角速度，抑制惯性过冲。
4. `EXIT_CAPTURE`：达到最小转角后，必须连续 `hybrid_exit_confirm_frames` 帧捕获新直道，且 IMU 误差进入窗口才恢复巡航。
5. `VISION_HOLD`：短时丢线仅低速保持，不发起/结束转弯；直道或弯道超过对应丢线时限进入 `SAFE_STOP`。
6. `SAFE_STOP`：IMU 未到、弯中 IMU 不更新、出弯长期未重新捕获赛道或任务超时均停车，禁止盲走。

SEG 的作用是提前看见路线和回收新直线，IMU 的作用是约束相对转角；轮速 `/odom` 仅累计路径长度和最短直线距离，符合位姿源规则。

#### 日志排查
- Stage2 会话日志：`~/dev_ws/log/competition_stage2/latest.log`
- 关键 tag：`HYBRID_START` / `HYBRID_STATE` / `HYBRID_CTRL` / `HYBRID_SAFE_STOP` / `HYBRID_DONE` / `TELEM`
- 视觉预览：仅在 `competition_phase=2` 时提供 HTTP `:8082`；切离 Stage2 即关闭。`/vision_debug` 话题不受 HTTP 生命周期影响。

#### 长直道异步图生文与语音

- 触发点：`top_long` 段距离终点默认剩余 `0.50m`，顺逆时针共用该判定；方向只影响弯道转向符号。
- Stage2 只发布 `stage2_ai_capture`（`std_msgs/Empty`），不等待摄像头、云端 API 或语音。
- `racing_vision_ai` 缓存 `/aurora/rgb/image_raw` 最新帧，在后台线程调用 Ark，发布 `ai_description`。
- `voice_broadcast_node` 异步订阅 `ai_description` 并播报；相关日志包含 `[AI_CAPTURE]`、`[VISION_AI]` 和 `[VOICE_BROADCAST]`。
- 配置参数：`stage2_ai_capture_enabled`、`stage2_ai_capture_lead_m`、`stage2_ai_trigger_topic`。

### 3.6 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 直行不到位 | `distance_tolerance` | ↑ |
| 转弯过冲 | `heading_tolerance_deg` / `turn_min_speed_ratio` | ↑（提前停） |
| 转弯不足 | `turn_kp` / `turn_angle_compensation_deg` | ↑ |
| 某角欠转 | `stage2_controller.yaml` 中对应 `track_*_complete_lead_deg` 或角速度 | 提前角↓ / 角速度↑ |
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
| `stage3_return_navigator.py` | — | 官方返程主体：前置通道 YOLO 对中 + 地图粗导航 + P 视觉最终到达 + Stage1 4态避障 |
| `global_path_planner.py` | — | A* 规划 mixin（TF 转换 / occupancy grid / scan overlay）|
| `phase3_test_trigger.py` | — | 独立测试工具，正式 total 不启动 |
| `stage3_test_simulator.py` | — | 返程测试工具，正式 total 不启动 |

**独立调参包**：`racing_stage3_param_test`

### 4.1 返程路径

路点通过 **JSON 参数** 传入（map 全局坐标系），仅用于尚未识别 P 时的粗导航；最终到达完全由 P 视觉决定：

| 参数 | 当前值 | 说明 |
|---|---|---|
| `return_waypoints_json` | `[{"x":0.20,"y":0.15,"speed":0.15,"description":"P_region_center"}]` | 搜索 P 的地图粗导航目标，不作最终完成判据 |
| `goal_box_x_min/max` | `0.1 / 0.3` | P 矩形区域 X 边界 |
| `goal_box_y_min/max` | `0.1 / 0.2` | P 矩形区域 Y 边界 |
| `goal_center_stop_distance_m` | `0.10` | 保留历史参数；生产 Stage3 不再使用地图中心判定完成 |

Phase3 启动后先执行前置通道 YOLO 对中并重置内部 map 初始值，再以地图目标 `0.35 m/s` 粗导航并全程检测 P。连续确认 P 后切入视觉伺服，以 `0.50 m/s` 和 bbox 水平偏差控制；视觉接管后 P 丢失或检测超时即发布 `v=0`、完成 Stage3，让底盘惯性自然滑入 P，不再以 map 位姿结束。

### 4.2 核心参数

| 参数 | 值 | 说明 |
|---|---|---|
| `pursuit_lookahead_m` | 0.45 m | 预瞄距离 |
| `pursuit_linear_speed` | 0.35 m/s | 未识别 P 时的地图粗导航速度 |
| `p_approach_linear_speed` | 0.50 m/s | 连续识别 P 后的视觉终段速度 |
| `p_detection_timeout_sec` | 0.35 s | 视觉终段检测超时后发布零速度并完成 |
| `pursuit_heading_stop_deg` | 70.0° | 航向差大于此值则进入低速正向重定向 |
| `pursuit_turn_kp` | 1.2 | PP 转向 P 增益 |
| `waypoint_tolerance` | 0.18 m | 中间路点容差 |

### 4.3 控制流

```
phase=3 收到
  └─ start_delay_sec 后 → pre_return_channel_yolo → reset_stage3_map_origin
      └─ control_loop
          ├─ emergency_stop
          ├─ [避障] Stage1 4态聚类避障（仅地图粗导航态）
          └─ map_search_p：地图粗导航 + P 检测连续确认
              └─ p_approach：视觉伺服 0.50m/s；P 丢失/超时 → v=0 → complete
```

### 4.4 状态机

```
idle → armed → pre_return_channel_yolo → running(map_search_p) → p_approach → complete
  ↑         running 时可中断为：
  └── avoiding → countersteer → recovering → running
```

### 4.5 调参速查

| 问题 | 参数 | 方向 |
|---|---|---|
| 路点到不了 | `waypoint_tolerance` | ↑ |
| P 视觉终段过早/过晚 | `p_approach_consecutive_hits` / `p_detection_timeout_sec` | ↑确认帧 / 调整超时 |
| 震荡 | `pursuit_turn_kp` | ↓ |
| 避障误触发 | `avoid_min_turn_angle_deg` / `avoid_safe_distance` | 调阈值 |

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
| `enable_test_publisher` | true | 单启动默认发布隔离测试 phase=2 和方向；总启动显式关闭 |
| `include_bringup` | true | 单启动默认启动 origincar_bringup，提供底盘、EKF 和 /odom_combined；总启动显式关闭 |
| `include_lidar` | true | 单启动默认启动激光雷达；总启动显式关闭 |
| `include_camera` | false | 是否启动普通摄像头 |
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
