# Racing 三阶段生产总览

本文件是赛事生产运行方式的总览。唯一生产入口为：

```bash
ros2 launch racing_bringup competition_total.launch.py
```

不再存在第二条开赛命令。公共基础层稳定后，S1 取得运动权，S2/S3 可并行驻留待命但不得发布运动命令。

## 1. 公共基础层

顶层 launch 只常驻以下公共节点：

- `origincar_base`，发布 `/odom`、`/imu/data_raw`，订阅 `/cmd_vel`；
- `imu_filter_madgwick_node`，将原始 IMU 处理为 `/imu/data`；
- `ekf_filter_node`，发布 `/odom_combined`；
- 雷达驱动，发布 `/scan`；
- Aurora RGB/Depth 相机，发布 `/aurora/rgb/image_raw` 和深度图；
- 机器人和传感器 TF；
- map server、lifecycle manager、启动雷达定位节点发布的 `map -> odom_combined` TF；
- 语音节点；
- `competition_supervisor`。
- `pose_chain_audit`，被动记录原始轮速/IMU、EKF、控制指令、雷达、地图、`/tf` 和
  `map->base_footprint` 查询；每条记录带 publisher 节点、GID、消息时间和接收年龄，写入
  `log/pose_chain_audit/<session>/pose_chain_audit.jsonl`。

生产环境没有 BNO055。`/imu/data` 的角速度来自底盘 IMU 经启动零偏校准后的链路；Madgwick 的 `orientation.yaw` 没有磁力计绝对参考，不能作为生产绝对航向。

Supervisor 只有在下列条件连续稳定后才开始 S1：`/cmd_vel` 有底盘订阅者、IMU、EKF 里程计、雷达、相机和 `/map` 都持续有新消息，且启动雷达定位已经锁定并发布 `map -> base_footprint`。`/odom_combined` 必须恰好只有一个 publisher；否则记录完整 publisher 身份并拒绝取得运动权。发现上一轮残留的阶段节点会拒绝本轮会话。

Supervisor 也是 `competition_phase` 的唯一 transient-local 发布者：待命/结束为 `0`，取得运动权的 S1/S2/S3 分别为 `1/2/3`。阶段专属 AI 只能以该话题判断是否允许执行任务，不能从其他阶段状态推断运动权。

基础层在整场比赛中只启动一次，不随阶段交接重启。底盘具有 `cmd_vel_watchdog_timeout_sec`（默认 `0.35s`）：最后一条非零指令超时未刷新时，驱动直接向底盘发送零速。

## 2. Supervisor

`competition_supervisor` 是唯一阶段编排者。它持续发布 transient-local 的 JSON 话题 `competition_supervisor_state`：

```text
session_id, base_state, active_stage, prewarming_stage, lifecycle_state, reason
```

`active_stage` 和 `prewarming_stage` 始终分开。例如 S1 运行期间是 `active_stage=S1`、`prewarming_stage=S2`，不能把 S1 运行状态覆盖成“Stage2 prewarming”。

阶段服务只允许 Supervisor 调用：

```text
/competition/stage1/activate  /competition/stage1/release
/competition/stage2/activate  /competition/stage2/release
/competition/stage3/activate  /competition/stage3/release
```

`release` 的正常语义是停止发布命令、释放本阶段资源并由阶段进程自己 `rclpy.shutdown()`；它不是正常交接中的停车指令。

## 3. 阶段生命周期

```text
基础 ready
  -> S1 standby -> ready -> activate -> running
  -> S2 standby prewarming（基础 ready 后即启动）
  -> QR 锁存方向
  -> S1 handoff_ready -> S2 first command -> S1 release/self-exit
  -> S2 running -> S3 prewarming
  -> S2 complete -> S3 first command -> S2 release/self-exit
  -> S3 running -> P point zero speed + complete
  -> Supervisor exits immediately -> top-level launch closes base layer
```

### S1

公共基础层启动 `start_corner_pose_diagnostic` 作为启动定位器；S1 启动 `competition_controller` 与 `qr_scanner`。相机属于公共基础层；S1 不加载视觉 AI，也不加载 BNO055。

`start_corner_pose_diagnostic` 在车辆静止时从后方 `(0,0)` 墙角的两条正交雷达墙线推算 Map 起点和 Map 航向。通过稳定门限后锁存起点，持续发布由雷达结果计算出的 `map -> odom_combined`，并以 transient-local JSON 发布 `map_x/map_y/map_yaw_deg` 与里程计锚点。生产摆放航向由参数限制在 `0~35°`，用来消除正交墙把 Map X/Y 互换时产生的互补角歧义；锁定后不再用行进扫描在线改写地图航向，避免遮挡、动态点或错误墙对造成跳变。行进期间保持锁定航向并只积分经零偏校准的 IMU 陀螺角速度。它不发布运动命令，也不再依赖旧的 `map_to_odom_*` 或 `initial_map_heading_deg` 参数。S1 只有在收到有效雷达起点并完成 IMU 陀螺角速度时间积分锚定后才发布 `ready`；二维码锁存后定位器继续运行，保证 S2/S3 交接期间地图 TF 不丢失。

控制器初始为 `standby/ready`，只有 `/competition/stage1/activate` 后才会启动 Nav2 任务。S1 不再实现自己的全局搜索、局部采样、雷达匹配或 TTC 监视器；这些职责统一交给 Nav2：`/scan` 进入局部/全局 `ObstacleLayer`，`InflationLayer` 提供足迹安全边界，`SmacPlannerHybrid` 负责带 Ackermann 最小转弯半径的全局路线，`nav2_mppi_controller::MPPIController` 负责短时域避障和 `/cmd_vel`，`velocity_smoother` 只做最终速度变化率限制。地图外、未知格和黑色区域仍由代价地图拒绝；扫描障碍不写回永久地图。`competition_controller.py` 现在只是 `NavigateToPose` action 适配器，因而整个 S1 只有 Nav2 一条正常运动发布链。

启动定位器仍由雷达墙角提供唯一的 `map -> odom_combined`，Nav2 只启动 `navigation_launch.py`，不启动 AMCL 或第二个 map TF 源。`map -> odom_combined -> base_footprint -> base_link -> laser` 必须在 `/scan` 时间戳下完整可查；其中 `base_link -> laser` 由公共 bringup 静态发布。`/odom_combined` 的短时位置和 `/imu/data` 的航向仍遵守全局位姿规则，Nav2 通过 TF 使用这条统一链。

S1 Smac Hybrid-A* 的启发式查表限制为覆盖地图对角线的 8 m；这只缩短冷启动查表，不改变地图内搜索约束。

扫码完成时适配器取消二维码 `NavigateToPose`，发布 `competition_qr_task`，再从实时 TF 位姿向通道入口 `(2.50,2.50,90°)` 发送第二个 action。Nav2 成功且入口位姿在窗口内稳定后，适配器发布 `stage1_state=handoff_ready` 和 `stage2_entry_pose`，进入 `handoff_wait`。生产路径不再使用盲开中线、固定倒车、墙体 map-X 自动校正或并行安全命令发布者。

### S2

基础层稳定后 Supervisor 即启动 S2 和 S2 专属视觉 AI 的 standby 进程。二维码只负责传递并锁存方向，S2 在 `prewarming/ready` 时加载参数、模型，但绝不发布运动命令。S1 的 `stage2_entry_pose` 是地图锚点，不是激活时当前位置；S2 记录收到锚点时的原始 TF，Supervisor 的 activate 到达后重新读取实时 TF/IMU，再输出第一条连续 `/cmd_vel` 并发布 `stage2_state=handoff_command_ready`。锚点延迟只记录诊断，不因年龄单独拒绝。随后 S1 收到 release，自行退出。

S2 完成时发布 `stage3_entry_anchor` 与 `stage2_state=complete`，保持最后有效命令直到 Supervisor 在收到 S3 的 `handoff_command_ready` 后调用 release。S2 本地不再用独立计时判断交接失败；S3 接管超时只由 Supervisor 判定。S2 后段的 `stage3_prewarm` 事件只预热 S3，不改变 S2 的运动权。

### S3

S3 standby 时加载 P 点视觉资源并等待合法的 `stage3_entry_anchor`，不发布运动命令。锚点允许在 transient-local 交付中延迟到达；S3 以锚点配合 `/odom_combined` 的 XY 位移增量建立当前位姿。activate 后输出第一条连续 `/cmd_vel` 并发布 `stage3_state=handoff_command_ready`，然后 S2 收到 release 并自行退出。

S3 生产控制只允许一条终端链路：`map_search -> visual_approach -> terminal_commit -> complete`。`map_search` 在 S2 交接位姿上只生成一次受最小转弯半径约束的入弧和其切线直线，使用路径前视点跟踪；角速度同时受 `v/R` 曲率上限和变化率限制，因此不会对远处地图目标反复左右折返。P 连续识别后才切换到画面横向偏差连续修正；满足宽松的 P 填充率和偏差窗口后进入 `terminal_commit`，固定 `angular.z=0` 直行，P 消失经短暂确认后停车完成。终端阶段不使用里程盲行、墙角校正、倒车恢复或雷达绕行；未知障碍安全保持只在地图搜索阶段生效。

到达 P 点是唯一正常终停：S3 锁定零速、输出最终 `[POSE_REAL]`、发布 `stage3_state=complete`、释放视觉资源并开始自行退出。Supervisor 收到 `complete` 后立即退出，不等待阶段 launch wrapper 或固定 grace 时间；顶层 launch 随后回收公共基础层，因此不需要额外 Ctrl+C。

Supervisor 进程退出只通过其顶层 `Node(on_exit=...)` 触发一次 `Shutdown` 事件；公共基础层由 launch 的标准信号收尾流程关闭，终端必须自动返回 shell。正常完成路径不并行发送额外信号或启动独立回收器；操作员不应以 Ctrl+C 完成正常收尾。

## 4. 自然速度交接

S1 -> S2 和 S2 -> S3 的正常路径不会由 Supervisor 主动发送零速：

```text
旧阶段 handoff_wait（保持最后有效命令）
  -> 新阶段 ready
  -> 新阶段 armed 后发布第一条连续命令和 handoff_command_ready
  -> Supervisor 调用旧阶段 release
  -> 旧阶段停止发布并自行退出
```

短暂的双 publisher 确认窗口是直接 `/cmd_vel` 架构下的有界交接窗口；它不会长期并行。新阶段必须用入口位姿、IMU 航向和旧阶段末速度建立第一条指令。若接管超时，Supervisor 按异常路径停车并锁定失败，不把超时当作正常切换。

## 5. 异常和退出

以下情况走安全失败路径：基础设备掉线、运动阶段进程崩溃、交接超时、complete 后未退出、Ctrl+C。顺序为：独立 `/cmd_vel` 零速保护、release/等待、SIGINT、SIGTERM、最后 SIGKILL。只有尚未取得运动权的 `starting/ready/prewarming` 阶段可以有限次重试；运动后崩溃不得自动重新行驶。

## 6. 坐标和日志

- 运动过程的唯一连续解算链为：底盘串口轮速 `vx/vy` → `/odom` 速度 → `robot_localization` EKF；IMU 陀螺 `angular_velocity.z` 同时进入 EKF 提供航向变化 → `/odom_combined` → 与初始 `map -> odom_combined` 锚点组成 `map -> base_footprint`。运动过程中不再由控制器或网页重新积分、二次旋转或覆盖这三个量。
- `/odom_combined` 只提供 XY、距离和位移；绝不使用其 orientation/yaw 导航。
- `/imu/data` 是所有阶段唯一的航向和转角来源。
- S1 终端 `[POSE_REAL]` 与 Web 的 `pose.x/y/yaw` 均读取同一个 `map -> base_footprint` TF；S1 交接记录 `stage2_entry_pose`；S2 交接记录 `stage3_entry_anchor`。
- 三阶段持续从 `map -> base_footprint` 输出真实 `[POSE_REAL]`。
- Web 监视器只读取 `map -> base_footprint` 的完整 TF（X/Y/yaw）并做像素映射；不再积分 IMU、读取启动位姿覆盖坐标或维护第二套 Map 位姿。
- 启动定位器将原始扫描、外参 TF、里程计、IMU、墙角解和最终 Map 位姿写入 `log/coordinate_trace/<session>/start_corner_trace.jsonl`。
- `pose_chain_audit` 另外记录每个关键话题的完整消息字段、`/tf`/`/tf_static` 每条变换、地图栅格以及周期性 publisher/ EKF 参数快照，用于区分“谁发了转弯指令”和“哪条位姿边先跳变”。

每次比赛使用统一 `COMPETITION_SESSION_ID`，日志写入：

```text
log/competition_runs/<session_id>/
├── ros/runtime/            # ROS 系统日志（顶层 launch）
├── ros/stage1/  ros/stage2/  ros/stage3/   # 各阶段 ROS 日志
├── stage1/latest.log       # 会话日志 + qr_latest.jpg（qr_scanner）
├── stage2/latest.log       # 会话日志 + ai_capture.jpg + local-smolvlm.log（vision_ai）
├── stage3/latest.log       # 会话日志 + latest_vision.jpg（Stage3）
├── pose_chain_audit/       # 位姿链审计 JSONL
└── coordinate_trace/       # 坐标追踪 JSONL
tools/                       # 开发工具日志（video / manual_trajectories / telemetry_web_monitor）
```

各阶段目录下的 `latest.log` 只是指向最新会话的软链接，历史事故日志不会被覆盖。终端只输出 `[STARTUP]`、`[TASK]`、`[POSE_REAL]` 和 `[ERROR]`。

## 7. 独立测试

三个 `competition_stage*.launch.py` 可以用于隔离测试，但测试 phase/task 必须是隔离话题，不能启动生产阶段 relay，也不能影响 `competition_total.launch.py`。

## 8.运动链路
```txt
STM32 原始轮速
  → origincar_base 解码 vx / vy
  → /odom.twist
  → robot_localization EKF

STM32 原始陀螺 z
  → /imu/data_raw
  → /imu/data.angular_velocity.z
  → EKF 航向变化

/odom vx/vy + IMU gyro_z
  → /odom_combined
  → odom_combined→base_footprint TF

初始 T₀(map→odom_combined)
  + 动态 /odom_combined
  → map→base_footprint TF

map→base_footprint TF
  ├→ S1 控制器
  ├→ 终端 [POSE_REAL]
  └→ 网页 Map X/Y/yaw
```
