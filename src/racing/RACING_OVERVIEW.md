# Racing 三阶段方案总览

> 本文档描述当前 `competition_total.launch.py` 实际启动的生产流程。
> 参数、状态和坐标的唯一事实来源仍是对应节点代码和生产 YAML；本文档随实现变更同步维护。
> `racing_stage2_param_test`、`racing_stage3_param_test` 和 `bak/` 均不属于正式总启动流程。

---

## 1. 当前生产架构

### 1.1 总启动与命令仲裁

总入口：`racing_bringup/launch/competition_total.launch.py`。

总启动同时常驻 Stage1、Stage2、Stage3 节点，Stage1 的 `competition_controller` 是唯一 `/cmd_vel` 发布者：

```
Phase 1: competition_controller ──────────────────────────> /cmd_vel
Phase 2: Stage2 /stage2_cmd_vel ── Stage1 supervisor ────> /cmd_vel
Phase 3: Stage3 /stage3_cmd_vel ── Stage1 supervisor ────> /cmd_vel
                           └─ 首条 S3 命令前可短暂沿用 S2 末段命令
```

- `competition_phase` 由 Stage1 以 transient-local QoS 发布，流程为 `1 -> 2 -> 3`。
- `competition_qr_task` 由 Stage1 从二维码内容解析后发布，值为 `clockwise` 或 `counterclockwise`。
- Stage2 完成发布 `stage2_state=complete`，Stage1 才切换 Phase 3。
- Stage2 交权时发布 `stage3_entry_anchor`；Stage3 生产模式必须得到新鲜锚点才允许出车。
- Stage2 在最后交接直线开始时发布 `stage3_preplan_pose`。该话题仅供 Stage3 的可选后台 A* 预规划；当前生产 YAML 已关闭 A*，因此不影响实际行驶。

### 1.2 视觉资源与 HTTP 端口

视觉服务按 `competition_phase` 互斥：

| 阶段 | 服务端口 | 行为 |
|---|---:|---|
| Stage1 | 8081 | 通道 YOLO 只在 Phase 1 启用，离开后释放模型和端口。 |
| Stage2 | 8082 | SEG 预览只在 Phase 2 提供；二维码任务到达后可预热模型。 |
| Stage3 | 8083 | P 视觉只在 Phase 3 武装；离开阶段或完成后释放。 |

`/vision_debug` 不受 HTTP 生命周期限制。`racing_vision_ai` 只在 Phase 2 缓存相机帧，并在进入 Phase 2 时启动和预热本地 SmolVLM 服务；进入 Phase 3 后停止取帧和新触发，但不会中断已提交的图生文。收到 `stage2_ai_capture` 后豆包 Ark 官方 SDK 的 Responses API、已配置的 Qwen 与就绪的本地 VLM 并发图生文，首个有效流式输出按短句发布到 `ai_description` 并成为胜者；Ark 流式调用只发布最终文本事件，不播报模型的推理摘要。仅在该决胜时请求取消其余两个请求，绝不由阶段切换取消。本地服务在本地模型落败时随决胜释放；本地模型胜出时仅在 `stage3_state=complete` 且胜出播报结束后关闭。该过程不阻塞底盘控制。

### 1.3 关键话题

| Topic | 发布者 | 用途 |
|---|---|---|
| `/cmd_vel` | Stage1 主控 | 唯一底盘指令输出 |
| `/stage2_cmd_vel` | Stage2 | Phase 2 候选控制指令 |
| `/stage3_cmd_vel` | Stage3 | Phase 3 候选控制指令 |
| `/odom_combined` | EKF | 三阶段的 xy 位移/位置源 |
| `/imu/data` | BNO055 | 唯一航向和转角来源 |
| `/scan` | 雷达 | Stage1/2/3 局部避障 |
| `/map` | map_overlay | Stage1 通道 A* 与 Stage3 可选 A* 静态地图 |
| `competition_phase` | Stage1 | 阶段状态（1/2/3） |
| `competition_qr_task` | Stage1 | QR 解析后的方向 |
| `qr_scan_result` | qr_scanner | 原始二维码结果 |
| `stage2_state` / `stage3_state` | Stage2 / Stage3 | 阶段内部状态 |
| `stage3_entry_anchor` | Stage2 | Stage3 锚定 map 位置 |

### 1.4 坐标与位姿强制规则

- `/odom` 仅用于启动诊断和轮速预热，禁止用其 orientation/yaw 导航。
- `/odom_combined` 只使用 xy。Stage2 对相邻 xy 累计欧氏距离；Stage3 从 S2 锚点起以 xy 增量更新 map 位置。
- `/imu/data` 是所有航向、转角、激光角度基准的唯一来源，禁止使用 `/odom` 或 `/odom_combined` 的 orientation。Stage1 在首帧将原始 IMU 映射到 `imu_initial_map_yaw_deg`，并以 transient-local `imu_map_yaw_offset` 发布该固定零点；Stage2 必须继承此映射，不能把原始 IMU yaw 直接当作 map yaw。
- Stage1 通道导航目标是 map 坐标，位置由 `map <- base_footprint` TF 获取；其 IMU 首帧映射到 `stage1_controller.yaml` 的 `imu_initial_map_yaw_deg`，以后仅累计原始 IMU 相对转角。
- map 到 `odom_combined` 的静态变换由 Stage1 YAML 作为总启动默认值注入：平移读取 `map_to_odom_x/y`，yaw 从 `imu_initial_map_yaw_deg` 派生。两者必须同源，防止 IMU 航向闭环和 map 轨迹坐标系出现固定偏角。

---

## 2. Stage1：二维码、倒退与通道交接

**生产包/配置**：`racing_stage1`，`config/stage1_controller.yaml`。

### 2.1 状态与流程

```
Phase 1 blind drive
  -> QR result
  -> recorded-path backing
  -> side-wall corridor follow
  -> terminal handoff alignment
  -> Phase 2
```

1. Stage1 初始盲驱使用 YAML `blind_linear_speed`（当前 0.50 m/s）和 `blind_angular_speed`；`/odom_combined` X 超过 `blind_qr_slowdown_start_x_m`（当前 1.5m）后，普通盲驱直行改用 `blind_qr_slowdown_linear_speed`（当前 0.4 m/s）以减少扫码运动模糊。二维码解码从 `blind_scan_capture_start_odom_x_m`（当前 1.0m）武装，但不产生转向；未识别时仅到 `blind_scan_guidance_start_odom_x_m`（当前 3.0m）才开始向 `blind_scan_centerline_json` 的 map 折线平滑切入，切入角速度在 `blind_scan_guidance_ramp_m`（当前 0.8m）内逐步增加。扫描带两侧 `blind_scan_corridor_half_width_m` 始终用于限制避障候选。
2. `qr_scanner` 使用 WeChat CV 发布 `qr_scan_result`；Stage1 解析并锁存方向后发布 `competition_qr_task`，进入记录路径倒退。
3. 倒退使用 `/odom_combined` 的历史 xy 回放，负线速度追踪倒序路径；角速度以 IMU 和几何目标闭环，不能直接复用历史 yaw。
4. 倒退完成后，Stage1 直接以 `/scan` 中两侧围墙的拟合中线跟随通道，不再让 `corridor_reference_path_json`、A*、Pure Pursuit 或 map X 强制恢复参与方向控制。每侧激光点先按扫描顺序聚类，只有相邻点距离不超过 `0.30m` 的连续簇才会单独拟合；墙簇还必须满足点数、前向跨度和直线残差，零散障碍物不得并入墙线。每条候选墙还必须以 IMU 映射到全局，并与通道出口轴 `corridor_goal_yaw=90°` 平行（正反方向等价）；这会拒绝入口下方的横墙，避免将其误认作侧墙。只有两侧都通过轴向、平行度和宽度检验，才形成完整赛道边界并接管中线控制。已锁定的同侧墙簇在短暂遮挡时不切换到不相干结构；日志会记录 `wall_cluster_source_lock` 的点数、跨度、残差、距离和切线角，可直接验证是否形成完整赛道边界。5m 场地中央的外墙可距车约 2.5m，生产筛选范围覆盖至 4.0m、可接受墙距至 5.6m。单侧墙不再决定前进方向；双墙未形成时，车辆仅低速以 IMU 对准通道轴、继续观察雷达，禁止单墙横向结构把车带偏。双墙仍是最终交权的唯一横向依据。雷达障碍检测仍只使用车前窄窗口的独立短小聚类，通道障碍必须达到独立的最小宽度，侧墙点不会触发避障；避障四状态完成后重新进入同一围墙中心线控制。
5. 围墙直跟仍只用激光相对几何；绝对 yaw 唯一来自 IMU，里程计角度绝不参与。最后 Y 门线前，双墙中心和墙轴质量只作为物理位置/边界判据；终端控制的目标航向始终固定为 `corridor_goal_yaw=90°`，禁止将局部墙切线或 map X 横向误差叠加到最终 IMU yaw。若入口边缘导致双墙拟合短暂丢失，终端可在 YAML 时限内复用最近一次合格的双墙中心/轴质量，不能因单帧失锁退出提交流程。双墙中心、IMU yaw、Y 门线满足门限时锁存 `terminal_commit_straight`，以固定出口 IMU 航向直行，禁止根据新横向误差反向打舵。交权要求 Y 门线、IMU yaw、两墙局部几何持续满足 YAML 的 hold 时间；围墙中心是物理横向依据，map X 仅记录诊断，默认不再阻塞交权。终端避障或提交失败导致 Y 超过窗口时，必须低速倒回 staging 后重新捕获，禁止高位交权；通道超时会停车等待，不应绕过终端判据直接放行。

### 2.2 Stage1 避障

扫码阶段保留原有四状态：

```
forward -> avoiding -> countersteering -> recovering -> forward
```

雷达在 `phase1_window` 中聚类，过滤点数、宽度和距离异常；扫码解码武装后，前向预检范围扩至生产 YAML 的 `blind_scan_avoid_detection_max_x_m`，为横移留出距离。扫码中心线激活后，控制器对左绕、右绕各自积分预测避障、反舵和回归段：候选必须同时满足障碍净距和二维码扫描走廊约束，优先选择最终横向误差更小的一侧。两侧都不可行时先检查车后近区；车后无障碍才低速后退并朝远离前障的一侧转向一次，随后允许有限的临时扫描带横移余量并重新预测。后方受阻或重试后仍无安全候选才停车等待，禁止以硬编码左右转越出识别范围。通道状态不再使用这套正向绕行。前方有效障碍簇触发 `corridor_reverse_avoid`：右前障碍倒车左摆、左前障碍倒车右摆；即使前方暂时清障，车辆仍以反向围墙平行/居中控制持续退回本次通道起点，进入 `corridor_reverse_avoid_entry_tolerance_m` 后才恢复正向已锁墙簇。没有倒车时长上限，也没有通道避障停车分支。日志记录障碍的距离、宽度、点数、倒车转向、起点和当时墙宽/中心误差。扫码避障恢复以扫描中心线航向为目标，通道恢复以当前通道航向为目标。

### 2.3 Phase2/3 指令转发

- Phase 2 只转发新鲜 `/stage2_cmd_vel`，`stage2_cmd_timeout` 当前为 0.5 秒，超时发零速度。
- Phase 3 优先转发新鲜 `/stage3_cmd_vel`；若 S3 尚未给出首条新鲜命令，可短暂转发 S2 最后一条命令，保证交权连续。二者都失效则停车。

---

## 3. Stage2：生产弧线赛道惯性导航

**生产包/配置**：`racing_stage2`，`config/stage2_controller.yaml`。

> 当前生产只有一个统一 YAML。旧 `inertial_stage2.yaml`、矩形 `rect_*` 路点和 `field_track_*.yaml` 已不被生产 launch 使用。

### 3.1 固定段序

```
entry_arc -> entry_medium -> left_side_arc -> top_long -> right_side_arc
-> exit_medium -> exit_turn_90 -> stage3_handoff_line -> complete
```

- `entry_arc` 是入口 90 度弧；随后两个 `*_side_arc` 是 180 度弧。
- 方向由 QR 决定：入口及出口 90 度和两次 180 度的转向符号按 `clockwise/counterclockwise` 镜像。
- `track_side_arc_vision_enabled=true` 时，`entry_medium -> left_side_arc` 与 `top_long -> right_side_arc` 仅在名义终点前 `track_side_arc_vision_trigger_lead_m`（当前 0.20m）的窗口内由 SEG 前方横边确认；从进入该窗口起线速度降为 `track_side_arc_vision_trigger_speed_mps`（当前 0.45m/s）。窗口外 SEG 不得切段；未确认时仍由 `map <- base_footprint` 的 `track_turn_force_min/max_map_x` 强制切段。设为 `false` 时，两个 180 度弯忽略 SEG，只按 TF x 切段；TF 缺失才回退里程保护窗口。
- 直线完成距离用 `/odom_combined` xy 累计。四个弯道均由 IMU 相对转角驱动：入口和出口弯目标为 `90°`，两个侧弯从入弯时锁存的实际 IMU yaw 累积 `180°`。入口 `entry_arc`、第一侧弯 `left_side_arc`、第二侧弯 `right_side_arc` 和出口 `exit_turn_90` 分别具有独立的提前角、收角比例和完成角度容差。每段先按 `v / omega = R` 的固定曲率巡航，再按其自身 `track_<segment>_*` 参数切为同方向低角速度收角；收角只改变角速度，线速度始终保持该弯独立配置，达到该段容差且已走过最小弧长比例即切下一段。绝不反向打舵，也不把低 IMU yaw-rate 作为切段硬门槛，防止残余惯性让任一弯卡死并阻塞 Stage3 交接。两个 180 度弯直径约 `0.50m`，即 `track_corner_radius=0.25m`；其巡航 `0.35m/s` 与 `1.40rad/s` 指令满足该半径的曲率关系。
- 进入 `stage3_handoff_line` 后，S2 以实时 map TF 发布预测起点；当 map Y 小于 `track_stage3_handoff_map_y`（当前 2.40）时发布交接锚点和 `stage2_state=complete`。

### 3.2 控制与安全

- 生产速度、弧线控制、视觉修正和交权门限全部在 `stage2_controller.yaml` 的 `track_*` 参数中。四个弯各自拥有独立的 `track_<segment>_linear` 与 `track_<segment>_angular`，入口和出口 90 度弯使用 `0.40m` 半径，两个 180 度弯使用 `0.25m` 半径；每一组线速度和角速度应以 `v / omega = R` 配对，当前直线最大速度为 0.66 m/s。
- 直线段由 IMU 航向保持为主，可信 SEG 中线只作小幅横向修正；SEG 不可单独结束转弯。
- `StraightAvoidanceController` 只在直线段接管。雷达聚类先经过正前方 +/-15 度及最小横向尺寸过滤，并须在连续扫描中保持位置、横向和尺寸关联达到 `stage2_straight_avoid_confirm_frames`（当前 3 帧），才成为可规划障碍；弧线段和两个 180 度弯前预检区会清空候选，不能跨段触发。它从同一帧雷达估计两侧围栏内缘，并以车体半宽、障碍宽度和安全边界求可行的最小横移；只有完整 S 形横移可在障碍前完成且不扫到任一围栏时才接管。避障全程保持当前直线速度，偏航量由所需横移反算并受最大角度限制，结束后进入短冷却再接受新候选。弧线段关闭前方硬停车，避免扫到赛道边界误停。
- 两个 180 度弯前的 `stage2_turn_precheck_*` 当前只写诊断日志，不停车、不减速、不改变切段。
- 控制循环超过 `control_gap_stop_sec`（当前 1.0 秒）未刷新时，命令心跳发布零速度；控制循环恢复后从当前段继续。
- `top_long` 距离末端 `stage2_ai_capture_lead_m`（当前 0.50 m）时只发一次异步图像分析触发，不等待云端或语音；图生文三模型并发竞速，语音从胜出模型的第一条完整短句开始播报。

### 3.3 启动模式

`competition_stage2.launch.py` 单独启动时默认可启动底盘/雷达支持栈、隔离测试 phase 及 `/stage2_cmd_vel -> /cmd_vel` relay。总启动显式关闭这些支持栈、测试发布者和 relay，由 Stage1 统一仲裁指令。

---

## 4. Stage3：单目标搜索 + P 视觉终停

**生产包/配置**：`racing_stage3`，`config/stage3_controller.yaml`。

### 4.1 当前生产路径

当前生产配置为 `use_global_planner: false`。A* 与静态禁区恢复代码仍作为可选能力保留，但不会参与正式当前路线，也不会因为 `stage3_preplan_pose` 改变实际控制。

```
Phase 3
  -> wait for fresh Stage2 anchor
  -> low-speed initial_align (only if heading error is large)
  -> drive to P visual-search goal (0.50, 0.10)
  -> visual_search_wait
  -> P visual approach
  -> P-bbox depth threshold -> complete
```

- S3 锁存 `stage3_entry_anchor` 与当时 `/odom_combined` xy；后续 map 位置为锚点加上旋转后的 odom xy 增量，运行中不使用实时 map TF 覆写控制位置。
- 交权瞬间把原始 IMU yaw 映射至 map -Y（`stage3_entry_map_yaw_deg=-90`），后续仅累计 IMU 相对转角。
- 初始目标方位误差达到 `initial_align_trigger_deg` 才进入 `initial_align`；阿克曼底盘以非零低速摆弧对准，不能原地转向。
- 未识别 P 时，向 `return_waypoints_json` 中当前单一视觉搜索目标行驶；到 `waypoint_tolerance`（当前 0.25 m）范围后停车等待 P，不能继续盲走。
- P 连续识别满足门限后立即接管。首次检测框偏差换算为锁定的 IMU 目标航向，车辆以该航向直线接近；只有框偏差越过 `p_heading_reacquire_offset` 且满足重捕获间隔时才更新目标航向，避免远距离逐帧追框走弧线。P 框中心 ROI 的有效深度中位数小于 `p_depth_stop_distance_m`（当前 0.37 m）立即发布 `stage3_state=complete`。
- P 接近中有效深度首次不大于 `p_approach_disable_avoidance_distance_m`（当前 0.75 m）后，本次任务跳过常规雷达避障直到完成；近距离雷达聚类仍会触发限时倒车加侧转，随后重新进入常规避障，禁止原地等待。

### 4.2 避障与丢失恢复

Stage3 粗导航复用 Stage1 的 `forward -> avoiding -> countersteering -> recovering` 聚类避障。避障对左右候选转向分别评估最小转角后的航向与当前 P 视觉搜索目标的夹角，选择更接近目标的一侧；明确朝近障同侧转入有硬安全惩罚。紧急近障不再由 Stage1 仲裁器硬停车：S3 以紧急前方聚类触发 `emergency_reversing`，在 YAML 限定的时间内低速倒车并向安全侧反向侧转，完成后回到 `forward` 重新判定常规避障。P 视觉接管后，普通避障开始时锁存当前 P 视觉航向；反舵和恢复阶段必须回到该锁定 IMU 航向，不能改用地图搜索点方向。若此后 P 丢失，低速倒车回正也复用同一锁定航向；P 重新识别后才按新框偏差更新航向。超时仍未重获则停车等待，禁止回退到粗导航盲走。

---

## 5. 日志、辅助包与启动

| 项目 | 位置/说明 |
|---|---|
| Stage1 日志 | `~/dev_ws/log/competition_stage1/latest.log` |
| Stage2 日志 | `~/dev_ws/log/competition_stage2/latest.log`，仅在首次进入 Phase 2 时创建并覆盖 |
| Stage3 日志 | `~/dev_ws/log/competition_stage3/latest.log`，仅在首次进入 Phase 3 时创建并覆盖 |
| QR | `qr_scanner`，WeChat CV 解码 |
| 图生文 | `racing_vision_ai`，接收 Stage2 一次性触发，豆包/Qwen/本地 VLM 流式竞速 |
| 语音 | `voice_driver`，异步顺序播报 `ai_description` 流式短句 |
| 通用日志/Marker | `racing_common` |

```bash
source /opt/ros/humble/setup.bash
ros2 launch racing_bringup competition_total.launch.py

# 独立 Stage2：默认隔离测试 phase、支持栈和 cmd relay 均开启
ros2 launch racing_stage2 competition_stage2.launch.py

# 独立 Stage3：需自行提供 phase、S2 交接锚点、里程计、IMU、雷达和深度相机
ros2 launch racing_stage3 competition_stage3.launch.py
```

## 6. 维护注意事项

1. 修改阶段状态机、生产 YAML、launch 话题或交权坐标时，必须同步更新本文档和 `docs/CHANGELOG.md`。
2. 不要把 `bak/`、参数测试包或已删除矩形赛道方案写成生产行为。
3. 生产参数的准确数值优先以 `stage1_controller.yaml`、`stage2_controller.yaml`、`stage3_controller.yaml` 为准；本文只保留对流程有决定意义的当前值。
