# Racing 三阶段方案总览

> 本文档描述当前 `competition_total.launch.py` 实际启动的生产流程。
> 参数、状态和坐标的唯一事实来源仍是对应节点代码和生产 YAML；本文档随实现变更同步维护。
> 历史参数测试包和 `bak/` 内容已从源代码中删除，不属于正式总启动流程。

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

`/vision_debug` 不受 HTTP 生命周期限制。`racing_vision_ai` 在节点启动时加载本地 SmolVLM 服务；Stage1 发布二维码任务后，立即缓存相机帧并用小图完成一次多模态视觉预填充，避免展示牌抓拍时才加载模型或首次执行视觉编码。Phase 2 只控制新抓拍是否接受；进入 Phase 3 后停止新触发，但绝不取消已提交的图生文，节点关闭时也会等待该请求写出结果。收到 `stage2_ai_capture` 后会将送入模型的同一帧覆盖保存为 `~/dev_ws/log/competition_stage2/ai_capture.jpg`，再由豆包 Ark 官方 SDK 的 Responses API、已配置的 Qwen 与就绪的本地 VLM 并发图生文。`vision_ai_config.yaml` 中 `response.streaming_enabled` 默认 `false`，即等待胜出模型生成完整描述后一次性发布到 `ai_description` 并播报；需要抢首句时改为 `true`，按首个有效流式输出短句发布并成为胜者。生产 YAML 对 Ark 和 Qwen 均显式关闭思考模式，Ark 流式调用也只发布最终文本事件。仅在该决胜时请求取消其余两个请求，绝不由阶段切换取消。本地服务在本地模型落败时随决胜释放；本地模型胜出时仅在 `stage3_state=complete` 且胜出播报结束后关闭。`latest.log` 的 `[LOCAL_TIMING]` 记录抓拍至本地请求、首个输出和完整生成的耗时。该过程不阻塞底盘控制。

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
- `/imu/data` 是所有常规航向、转角和激光角度基准的来源，禁止使用 `/odom` 或 `/odom_combined` 的 orientation。Stage1 在首帧将原始 IMU 映射到 `imu_initial_map_yaw_deg`，并以 transient-local `imu_map_yaw_offset` 发布该固定零点；二维码倒退完成后用 IMU 滚动对正到通道 90°，再进入通道。Stage2 不能把原始 IMU yaw 直接当作 map yaw。
- Stage1 通道导航目标是 map 坐标，位置由 `map <- base_footprint` TF 获取；其 IMU 首帧映射到 `stage1_controller.yaml` 的 `imu_initial_map_yaw_deg`，以后仅累计原始 IMU 相对转角。Stage1 -> Stage2 交权优先使用雷达拟合的前方横墙距离：最终路点状态下锁定同一面连续横墙，候选选择参考 `stage1_controller.yaml` 的 `corridor_front_wall_reference_distance_m`，距离进入 `corridor_front_wall_handoff_distance_m` 后冻结触发证据，且必须满足墙修正 X 窗口、侧墙居中/轴向和 IMU yaw 窗口才切 Phase2；固定 map Y 门线 `corridor_release_max_y_m` 仅作兜底触发和越界保护。强制放行计时使用 `corridor_handoff_force_after_reject_sec`，具体值以生产 YAML 为准。墙修正 X 与双墙中心航向修正均读取生产 YAML，交权失败倒车重捕获使用镜像后的 X+中心误差航向，保证倒车运动方向同时退回低 Y 并向 X 窗口靠近。
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

1. Stage1 初始盲驱使用 YAML `blind_linear_speed`（当前 0.50 m/s）和 `blind_angular_speed`；`/odom_combined` X 超过 `blind_qr_slowdown_start_x_m`（当前 1.5m）后，普通盲驱直行改用 `blind_qr_slowdown_linear_speed`（当前 0.4 m/s）以减少扫码运动模糊。二维码解码从 `blind_scan_capture_start_odom_x_m`（当前 1.0m）武装，但不产生转向；未识别时仅到 `blind_scan_guidance_start_odom_x_m`（当前 3.0m）才开始向 `blind_scan_centerline_json` 的 map 多点折线平滑切入，当前中心线从 `(0.25,0.20)` 经 `(2.50,0.50)` 近似弯向 `(3.80,1.80)`；切入角速度在 `blind_scan_guidance_ramp_m`（当前 0.8m）内逐步增加。扫描带两侧 `blind_scan_corridor_half_width_m` 始终用于限制避障候选。
2. `qr_scanner` 使用 WeChat CV 发布 `qr_scan_result`；Stage1 解析并锁存方向后发布 `competition_qr_task`，进入记录路径倒退。
3. 倒退使用 `/odom_combined` 的历史 xy 回放，负线速度 `back_linear_speed`（当前 -0.45 m/s）追踪倒序路径；角速度以 IMU 和几何目标闭环，不能直接复用历史 yaw。
4. 倒退完成后，Stage1 清空扫码避障残留状态，先用 IMU 以非零前进速度滚动对正到 `back_align_yaw_deg=90°`，再进入通道；阿克曼底盘禁止用 `v=0` 原地转向。`backing_align` 期间不响应普通扫码避障，避免入口侧墙或障碍把状态机拉回盲扫恢复。进入通道后从 `/scan` 拟合两侧围墙作居中、map-X 修正和入口 yaw 修正。取墙速度当前为 0.30 m/s（大角度滚动转向 0.22 m/s），双墙居中后以 0.48 m/s 跟随，偏差较大时降到 0.32 m/s，最终预对正速度为 0.28 m/s，提交直线为 0.42 m/s。每侧激光点按扫描顺序聚类，只有相邻点距离不超过 `0.30m` 的连续簇才会单独拟合；墙簇还必须满足点数、前向跨度、直线残差、车体系轴向、平行度和宽度检验，零散障碍物或入口横墙不得参与。候选锁墙轴向门槛为 `35°`，只用于入口偏航时先形成双墙质量参考；墙轴不再单独抢占转向，行驶航向固定围绕 IMU `90°` 并叠加墙修正 X 并线偏置。IMU->map 重标仍必须进入 `10°` 内。单墙仍不得决定前进方向。雷达障碍检测仍只使用车前窄窗口的独立短小聚类，侧墙点不会触发避障。
5. 通道行驶以双墙中心和 IMU 轴向为主；墙轴重标、前墙距离、X 窗口、中心误差、yaw 门限和强制放行计时均读取 `stage1_controller.yaml`。前墙按连续点簇拟合为车体系横线，锁定同一物理墙源并持续更新距离，进入 YAML 的 `corridor_front_wall_handoff_distance_m` 后冻结 `FRONT_WALL_HANDOFF_LATCH` 证据并检查交权硬门。任一不满足则记录 `handoff gate rejected` 并倒退到 staging 重捕获；重捕获倒退、重新前进和最终提交直线段统一使用受限的 X+双墙中心航向闭环，避免重复沿同一直线失败。若前方横墙不可用，则 `corridor_release_max_y_m` 仍作为兜底触发。合格时 Stage1 发布 `stage2_entry_pose`，日志为 `STAGE2_ENTRY_POSE` 和对应 handoff 原因。

### 2.2 Stage1 避障

扫码阶段保留原有四状态：

```
forward -> avoiding -> countersteering -> recovering -> forward
```

雷达在 `phase1_window` 中聚类，过滤点数、宽度和距离异常；扫码解码武装后，前向预检范围扩至生产 YAML 的 `blind_scan_avoid_detection_max_x_m`，为横移留出距离。扫码中心线激活后，控制器对左绕、右绕各自积分预测避障、反舵和回归段：候选必须满足障碍净距；扫描走廊仍是默认硬约束，但障碍中心角超过 `blind_scan_avoid_obstacle_side_min_angle_deg`（当前 5°）时，远离障碍的一侧可使用 `blind_scan_avoid_obstacle_side_lane_extra_m`（当前 0.12m）扫描带放宽，并优先于单纯贴近中心线的同侧绕行。预测期内已从障碍前方完整通过的候选优先，但固定预测窗口没有走完低速动作时，仍允许满足安全约束的候选正向执行，不能错误停车。两侧都不满足安全约束时先检查车后近区；车后无障碍才低速后退并朝远离前障的一侧转向一次，随后允许有限的临时扫描带横移余量并重新预测。后方受阻或重试后仍无安全候选才停车等待，禁止以硬编码左右转越出识别范围。通道状态不再使用这套正向绕行。前方有效障碍簇触发 `corridor_reverse_avoid`：右前障碍倒车左摆、左前障碍倒车右摆；即使前方暂时清障，车辆仍以反向围墙平行/居中控制持续退回本次通道起点，进入 `corridor_reverse_avoid_entry_tolerance_m` 后才恢复正向已锁墙簇。通道倒车避障和扫码脱困倒车的命令由各自状态独占，不能被普通正向绕障命令覆盖。没有倒车时长上限，也没有通道避障停车分支。日志记录障碍的距离、宽度、点数、倒车转向、起点和当时墙宽/中心误差。扫码避障恢复以扫描中心线航向为目标，通道恢复以当前通道航向为目标。

### 2.3 Phase2/3 指令转发

- Phase 2 只转发新鲜 `/stage2_cmd_vel`，`stage2_cmd_timeout` 当前为 0.5 秒，超时发零速度。
- Phase 3 优先转发新鲜 `/stage3_cmd_vel`；若 S3 尚未给出首条新鲜命令，可短暂转发 S2 最后一条命令，保证交权连续。二者都失效则停车。

---

## 3. Stage2：生产弧线赛道惯性导航

**生产包/配置**：`racing_stage2`，`config/stage2_controller.yaml`。

> 当前生产仍只加载 `stage2_controller.yaml`，但弯道速度、提前收角、两个 180° 的 map-X 强制触发阈值和 `exit_medium` 出口短直线距离已在该 YAML 内按 `clockwise/counterclockwise` 拆成方向 profile。旧 `inertial_stage2.yaml`、矩形 `rect_*` 路点和 `field_track_*.yaml` 已不被生产 launch 使用。

### 3.1 固定段序

```
entry_arc -> entry_medium -> left_side_arc -> top_long -> right_side_arc
-> exit_medium -> exit_turn_90 -> stage3_handoff_line -> complete
```

- `entry_arc` 是入口 90 度弧；随后两个 `*_side_arc` 是 180 度弧。
- 方向由 QR 决定：入口及出口 90 度和两次 180 度的转向符号按 `clockwise/counterclockwise` 镜像。
- S2 启动时优先读取 Stage1 发布的 `stage2_entry_pose` 墙修正入口 X，并以 `wall_offset=entry_pose.x - TF_entry_x` 修正实时 TF；随后 `track_x=当前 TF map_x + wall_offset`，即交接瞬间的 `track_x` 等于 Stage1 交接修正 X。没有新鲜入口位姿或 TF 不可用时退回真实 TF X。该修正不改写 TF，也不影响 Stage3 的真实 map 锚点。
- `entry_medium -> left_side_arc` 与 `top_long -> right_side_arc` 在各自名义终点前的视觉窗口内由 SEG 横边确认；视觉未确认时，仍由重置后的 `track_x` 与当前方向 profile 的 `track_<direction>_turn_force_min/max_map_x` 阈值强制切段。直线仅由 IMU 航向、可信 SEG 中线小修正和 yaw-rate 阻尼控制；雷达避障仅在 `top_long` 直线段接管。
- 直线完成距离用 `/odom_combined` xy 累计。四个弯道均由 IMU 相对转角驱动：入口和出口弯名义目标为 `90°`，两个侧弯名义目标为从入弯时锁存实际 IMU yaw 起累计 `180°`。每弯只使用当前方向 profile 的线速度、角速度和提前收角；`track_counterclockwise_*` 保持当前逆时针基准，`track_clockwise_*` 单独用于顺时针，不再强行共用同一提前角。一旦达到对应 `exit_lead_deg` 收角点，立即切入下一直线，由直线的 IMU 航向闭环和 yaw-rate 阻尼吸收余摆。绝不等待残余惯性补足名义角度、绝不反向打舵，也不把低 IMU yaw-rate 作为切段硬门槛，防止任一弯占用后续直线。
- `exit_medium` 使用当前方向 profile 的 `track_<direction>_exit_medium_distance_m`，用于控制第二个 180° 后、出口 90° 前的短直线长度。进入 `stage3_handoff_line` 后，S2 以实时 map TF 发布预测起点；当 map Y 小于 `track_stage3_handoff_map_y` 时发布交接锚点和 `stage2_state=complete`。

### 3.2 控制与安全

- 生产速度、弧线控制、视觉修正、真实 map-X 阈值和交权门限全部在 `stage2_controller.yaml` 的 `track_*` 参数中。四个弯各自拥有独立的 `track_<segment>_linear` 与 `track_<segment>_angular`，入口和出口 90 度弯使用 `0.40m` 半径，两个 180 度弯使用 `0.25m` 半径；每一组线速度和角速度应以 `v / omega = R` 配对，当前直线最大速度为 0.66 m/s。
- `TRACK_MAP_X_RESET` 日志记录交接瞬间锁存的修正入口 X、实时 TF X、`wall_offset` 与实际 `track_x` 来源；`TURN_TRIGGER` 同时记录真实 `map_xy` 和修正后的 `track_map_x`，可直接核验两个 `track_turn_force_*` 阈值是否在预期位置切弯。
- S2 移除 MPPI、围栏推断、连续帧门控、局部横移规划、制动安全门、弯道预检和旧的插入式 S 形绕行。仅在 `top_long` 使用与 S1 同构的前向聚类状态机：`forward -> avoiding -> countersteering -> recovering -> forward`。聚类按 `stage2_s1_avoid_*` 的前向矩形窗口、点数和宽度过滤；障碍在左前则右转、障碍在右前则左转。避障命令期间冻结本段进度，反舵后以该段锁存的 IMU 航向回正。`entry_medium`、所有弧段、`exit_medium` 与 `stage3_handoff_line` 不执行雷达避障，避免赛道边墙误触发。
- 控制循环超过 `control_gap_stop_sec`（当前 1.0 秒）未刷新时，命令心跳发布零速度；控制循环恢复后从当前段继续。
- 默认模式下，`top_long` 切入 `right_side_arc` 的入弯瞬间开始计时，满 `stage2_ai_capture_delay_after_turn_sec`（当前 0.20 s）后只发一次异步图像分析触发，确保相机已转向展示牌；图生文三模型并发竞速，`vision_ai_config.yaml` 的 `response.streaming_enabled` 默认关闭，语音等待胜出模型完整生成后一次性播报。若改为 `true`，则从胜出模型的第一条完整短句开始流式播报。本地 VLM 的 `timeout_sec: 0` 表示本地 HTTP 推理和包含本地候选的竞速等待不超时，云端不可用时持续等待本地结果。每次本地生成在 S2 `latest.log` 中输出 `[LOCAL_TIMING] request_started`、`first_delta`、`request_complete`，分别给出从抓拍至请求、首字和完整响应的秒数。非流式模式下不会产生 `first_delta`，以 `request_complete` 核验完整生成耗时。若 YAML 的 `stage2_ai_preset_enabled` 开启，则不触发图生文；从同一入弯点等待 `stage2_ai_preset_delay_after_turn_sec`（当前 5.0 s）后，按二维码方向直接向 `ai_description` 发布 `stage2_ai_preset_clockwise_text` 或 `stage2_ai_preset_counterclockwise_text`。

### 3.3 启动模式

`competition_stage2.launch.py` 单独启动时默认可启动底盘/雷达支持栈、隔离测试 phase 及 `/stage2_cmd_vel -> /cmd_vel` relay。总启动显式关闭这些支持栈、测试发布者和 relay，由 Stage1 统一仲裁指令。

---

## 4. Stage3：单目标搜索 + P 视觉接管终停

**生产包/配置**：`racing_stage3`，`config/stage3_controller.yaml`。

### 4.1 当前生产路径

当前生产配置为 `use_global_planner: false`。A* 与静态禁区恢复代码仍作为可选能力保留，但不会参与正式当前路线，也不会因为 `stage3_preplan_pose` 改变实际控制。

```
Phase 3
  -> wait for fresh Stage2 anchor
  -> low-speed initial_align (only if heading error is large)
  -> anchored map navigation to P visual-search point (0.50, 0.10)
  -> consecutive P detections take over heading and forward speed
  -> P lost: short reverse + return to map visual-search route
  -> P depth <= 0.50m: low-speed IMU/odom final run for 0.50m
  -> complete
```

- S3 锁存 `stage3_entry_anchor` 与当时 `/odom_combined` xy；后续 map 位置为锚点加上旋转后的 odom xy 增量，运行中不使用实时 map TF 覆写控制位置。
- 交权瞬间把原始 IMU yaw 映射至 map -Y（`stage3_entry_map_yaw_deg=-90`），后续仅累计 IMU 相对转角。
- 初始目标方位误差达到 `initial_align_trigger_deg` 才进入 `initial_align`；阿克曼底盘以非零低速摆弧对准，不能原地转向。
- 粗导航只向 `return_waypoints_json` 的单一视觉搜索点 `(0.50, 0.10)` 行驶，当前 `pursuit_linear_speed=0.72m/s`；它只用于将车带回可看见 P 的区域，不能发布完成。连续 P 检测在任何返程位置均可接管控制，按 P 框偏差锁定 IMU 目标航向后保持 `p_approach_linear_speed=0.48m/s` 前进。
- P 临时丢失时，车辆按最后锁定的视觉航向低速倒退 `p_loss_reverse_duration_sec`，随后恢复锚点地图导航并再次寻找 P；重获 P 后重新接管。正常雷达避障在视觉接近期间仍有效，直至 P 框深度进入最终阈值。
- P 框内有效深度不大于 `p_depth_stop_distance_m=0.50m` 时，锁定该时刻 IMU 航向，改用 `p_approach_slow_linear_speed=0.16m/s` 低速前进。若 Aurora 近距离深度在 P 框放大后失效，则以 `p_final_visual_fill_trigger_ratio=0.45` 的 P 框占比和最近 `p_final_visual_depth_assist_m=0.75m` 内有效深度作为备用证据，同样切入最终里程段，避免把近处 P/终点墙当作雷达障碍倒退。最终段只以 `/odom_combined` xy 计算欧氏位移；根据当前最终段速度，以 `v * p_final_brake_response_sec + v^2 / (2 * p_final_brake_decel_mps2) + p_final_brake_margin_m` 预留制动距离，在距 `p_final_odom_travel_m=0.50m` 的预测制动点先发布零速并在受控容差内发布 `stage3_state=complete`，不能等到跨过 0.50m 才停车。若终段已行驶不小于 `p_final_stall_completion_min_m=0.30m` 后，前进命令下 `/odom_combined` 超过 `p_final_progress_timeout_sec=0.75s` 无有效 xy 增量，则判定已顶到终点墙，立即停车并发布 `complete`；终段航向修正使用独立小角速度上限和 `10°` 死区，避免顶墙后继续绕圈。P 在这段中自然丢失不影响完成。角度始终只使用 IMU，禁止使用 odom orientation。墙角 TLS 代码保留用于离线调试，生产 YAML 已关闭且不参与完成门槛。

### 4.2 避障与丢失恢复

Stage3 在地图搜索和 P 视觉接近期间复用 Stage1 的 `forward -> avoiding -> countersteering -> recovering` 聚类避障。避障对左右候选转向分别评估最小转角后的航向与当前搜索目标的夹角，选择更接近目标的一侧；明确朝近障同侧转入有硬安全惩罚。紧急近障触发 `emergency_reversing`，完成后回到 `forward` 重新判定常规避障。普通避障、反舵、恢复和紧急倒车线速度当前分别为 0.16、0.16、0.18 和 0.14 m/s。P 接管且深度或视觉填充备用证据进入最终段的同一控制周期就会取消已有避障状态，并跳过普通和紧急雷达避障；此后只以锁定 IMU 航向和受限 `0.50m` 里程完成终停。

P 视觉接近本身不等于关闭避障：在最终 `P final odometry run armed` 之前，雷达普通避障和紧急倒车仍优先于 `P_APPROACH` 前进命令。若 P 接近阶段持续下发前进速度但 `/odom_combined` xy 在 `p_approach_progress_timeout_sec` 内没有达到 `p_approach_progress_min_delta_m` 推进量，则判定车体已顶住真实障碍或卡滞，进入同一套 `emergency_reversing` 后退带转向脱困；倒车完成后重新检查雷达并恢复 P/地图导航。只有最终近距离里程段屏蔽雷达，避免终点墙或 P 本体误触发绕行。

---

## 5. 日志、辅助包与启动

| 项目 | 位置/说明 |
|---|---|
| Stage1 日志 | `~/dev_ws/log/competition_stage1/latest.log` |
| Stage2 日志 | `~/dev_ws/log/competition_stage2/latest.log`，仅在首次进入 Phase 2 时创建并覆盖；S2 主控与图生文节点均以同一文件锁写入，图生文追加抓拍、模型竞速、结果与失败诊断。 |
| Stage3 日志 | `~/dev_ws/log/competition_stage3/latest.log`，仅在首次进入 Phase 3 时创建并覆盖 |
| QR | `qr_scanner`，WeChat CV 解码 |
| 图生文 | `racing_vision_ai`，接收 Stage2 一次性触发，豆包/Qwen/本地 VLM 并发竞速；默认完整结果一次发布，可用 `vision_ai_config.yaml` 的 `response.streaming_enabled: true` 打开流式短句发布 |
| 语音 | `voice_driver`，异步顺序播报 `ai_description` 文本 |
| 通用日志/Marker | `racing_common` |

```bash
source /opt/ros/humble/setup.bash
ros2 launch racing_bringup competition_total.launch.py

# 独立 Stage2：默认隔离测试 phase、支持栈和 cmd relay 均开启
ros2 launch racing_stage2 competition_stage2.launch.py

# 独立 Stage3：需自行提供 phase、S2 交接锚点、里程计、IMU、雷达和深度相机
ros2 launch racing_stage3 competition_stage3.launch.py
```

正式总启动会为所有子进程设置 `RMW_FASTRTPS_USE_SHM=0` 与 `RMW_FASTRTPS_TRANSPORT=UDPv4`，避免 RDKX5 多次启停后残留的 `/dev/shm/fastrtps_port*` 锁导致 Fast DDS shared-memory transport 初始化报错。若需要连 `ros2 launch` 父进程本身也不打印该类错误，可在执行启动命令前手动导出同名环境变量。

## 6. 维护注意事项

1. 修改阶段状态机、生产 YAML、launch 话题或交权坐标时，必须同步更新本文档和 `docs/CHANGELOG.md`。
2. 不要把 `bak/`、参数测试包或已删除矩形赛道方案写成生产行为。
3. 生产参数的准确数值优先以 `stage1_controller.yaml`、`stage2_controller.yaml`、`stage3_controller.yaml` 为准；本文只保留对流程有决定意义的当前值。
4. 正式总启动的 Stage1 shutdown 清理会先发布 `/cmd_vel` 零速，再对 `aurora930_node` 执行 `CONT/TERM/KILL` 释放 Aurora 930 USB 句柄；单独启动 `qr_scanner/start_competition.launch.py` 时也执行同样的相机释放。
