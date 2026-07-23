# 生产流程实际使用线速度定位

范围：按 `competition_total.launch.py` 正式总启动链路统计。启动入口确认：`src/racing/racing_bringup/launch/competition_total.launch.py` Line 83/95/115 分别引入 Stage1/Stage2/Stage3；Stage1/2/3 各自 launch 加载的生产配置见 `src/racing/racing_stage1/launch/competition_stage1.launch.py` Line 189、`src/racing/racing_stage2/launch/competition_stage2.launch.py` Line 162、`src/racing/racing_stage3/launch/competition_stage3.launch.py` Line 111。

## S1 前进两个速度

- `blind_linear_speed`: 0.50 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 97 | 实际选择逻辑 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 4767，正常盲开发布 Line 4901
- `blind_qr_slowdown_linear_speed`: 0.40 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 103 | 触发条件 `odom_x > blind_qr_slowdown_start_x_m`，实际选择逻辑 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 4770

## S1 倒退速度

- `back_linear_speed`: -0.40 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 310 | 记录路径倒退发布 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 5058

## S1 进入通道相关实际线速度

- `recovery_turn_linear_speed`: 0.20 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 273 | 倒退后大角度/中角度滚动对正进入通道 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 5106/5111
- `recovery_linear_speed`: 0.35 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 270 | 倒退后小角度对正进入通道 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 5108
- `corridor_wall_acquire_speed_mps`: 0.20 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 537 | 无合格双墙时滚动取墙 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 2881
- `corridor_wall_follow_correction_speed_mps`: 0.22 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 529 | 墙轴对齐/中心误差较大时降速 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 2936/2972
- `corridor_wall_follow_linear_speed_mps`: 0.36 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 528 | 双墙合格且中心误差不大时巡航 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 2972
- `corridor_terminal_linear_speed`: 0.20 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 423 | 终端墙几何保持/兜底终端逼近 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 3607/3629
- `corridor_terminal_commit_speed_mps`: 0.32 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 514 | 终端提交直线 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 3594
- `corridor_reacquire_reverse_speed`: -0.12 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 386 | Y 门线交权失败后退回 staging `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 3327
- `terminal_reverse_align_linear_speed`: -0.12 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 461 | 终端位置到位但航向未到位时后退回正 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 3536
- `corridor_reverse_avoid_linear_speed_mps`: -0.14 m/s | 文件 `src/racing/racing_stage1/config/stage1_controller.yaml` Line 216 | 通道前障碍倒车避障/回通道入口 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 4045/4101
- `corridor_terminal_offset_linear_speed_mps`: 0.18 m/s | 文件 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 104（代码默认，当前 YAML 未覆盖） | 终端障碍偏移交权分支 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 3997/4433

## S2 所有实际线速度

- `track_max_speed`: 0.66 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 99 | 用于 `entry_medium/top_long/exit_medium/stage3_handoff_line`，段表 `src/racing/racing_stage2/racing_stage2/track_controller.py` Line 350/353/356/361，发布 `src/racing/racing_stage2/racing_stage2/stage2_inertial_navigator.py` Line 1483
- `track_entry_arc_linear`: 0.50 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 107 | 入口 90 度弧，弧速映射 `src/racing/racing_stage2/racing_stage2/track_controller.py` Line 249，入口段创建 Line 379
- `track_left_side_arc_linear`: 0.70 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 110 | 第一侧 180 度弧，段表 `src/racing/racing_stage2/racing_stage2/track_controller.py` Line 351
- `track_right_side_arc_linear`: 0.70 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 113 | 第二侧 180 度弧，段表 `src/racing/racing_stage2/racing_stage2/track_controller.py` Line 354
- `track_exit_turn_90_linear`: 0.50 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 116 | 出口 90 度弧，段表 `src/racing/racing_stage2/racing_stage2/track_controller.py` Line 359
- `stage2_s1_avoid_linear_speed_mps`: 0.30 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 237 | `top_long` S1 风格避障 `src/racing/racing_stage2/racing_stage2/stage2_inertial_navigator.py` Line 1902
- `stage2_s1_counter_linear_speed_mps`: 0.30 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 242 | `top_long` 反舵 `src/racing/racing_stage2/racing_stage2/stage2_inertial_navigator.py` Line 1911
- `stage2_s1_recovery_turn_linear_speed_mps`: 0.20 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 248 | `top_long` 回正时大航向误差 `src/racing/racing_stage2/racing_stage2/stage2_inertial_navigator.py` Line 1944
- `stage2_s1_recovery_linear_speed_mps`: 0.35 m/s | 文件 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 247 | `top_long` 回正接近目标航向 `src/racing/racing_stage2/racing_stage2/stage2_inertial_navigator.py` Line 1946

## S3 所有实际线速度

- `initial_align_linear_speed`: 0.12 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 115 | 初始大航向误差摆弧对正 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 1771/1780
- `pursuit_linear_speed`: 0.60 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 89 | 地图搜索 P 的常规速度 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 2934
- `pursuit_turn_linear_speed`: 0.08 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 101 | 地图搜索 P 大航向误差时最低转弯速度 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 2941
- `p_approach_linear_speed`: 0.35 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 148 | P 视觉接管接近 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 2468
- `p_loss_reverse_speed`: 0.10 m/s，实际发布为 -0.10 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 205 | P 丢失短倒退 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 2417/2429
- `p_approach_slow_linear_speed`: 0.12 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 173 | P 近距离最终里程段 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 2563/2640
- `emergency_reverse_speed`: 0.10 m/s，实际发布为 -0.10 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 309 | 紧急近障倒退 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 1731
- `avoid_linear_speed`: 0.10 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 279 | 普通激光避障 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 3188
- `counter_steer_linear_speed`: 0.10 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 315 | 普通避障反舵 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 3195
- `recovery_turn_linear_speed`: 0.08 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 334 | 普通避障回正，大航向误差 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 3208
- `recovery_linear_speed`: 0.12 m/s | 文件 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 331 | 普通避障回正，小航向误差/定时回正 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 3208/3211

## 未列入的非实际使用速度

- S1 `corridor_waypoints_json` 内的 `speed` 字段：当前被解析但未用于命令发布，且 `corridor_wall_follow_enabled=true` 时直接锁最后通道路点，见 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 1552。
- S1 `corridor_linear_speed`、`turn_linear_speed`、`corridor_creep_speed`、`corridor_min_cruise_speed`、`corridor_max_turn_linear_speed`：属于地图 Pure Pursuit 旧通道路径跟踪；当前生产双墙跟随接管，正常不会走该分支。
- S1 `corridor_wall_acquire_turn_linear_speed_mps`：当前代码读取但未用于发布，实际墙获取发布使用 `corridor_wall_acquire_speed_mps`，见 `src/racing/racing_stage1/racing_stage1/competition_controller.py` Line 2881。
- S2 `track_side_arc_vision_trigger_speed_mps`: 当前 `track_entry_boundary_trigger_enabled=false` 且 `track_top_boundary_trigger_enabled=false`，触发限速分支不生效，见 `src/racing/racing_stage2/config/stage2_controller.yaml` Line 135/145 与 `src/racing/racing_stage2/racing_stage2/track_controller.py` Line 657。
- S3 `return_waypoints_json` 内的 `speed=0.45`：当前只用目标点 x/y，地图搜索速度来自 `pursuit_linear_speed`，见 `src/racing/racing_stage3/racing_stage3/stage3_return_navigator.py` Line 852/2820。
- S3 `planner_forbidden_reverse_speed`、`terminal_precommit_linear_speed`、`terminal_corner_approach_speed`、`stage3_channel_linear_speed`：当前生产 `use_global_planner=false`、`terminal_corner_enabled=false`、`stage3_channel_yolo_enabled=false`，见 `src/racing/racing_stage3/config/stage3_controller.yaml` Line 399/218/443。
