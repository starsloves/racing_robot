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

生产环境没有 BNO055。`/imu/data` 的角速度来自底盘 IMU 经启动零偏校准后的链路；Madgwick 的 `orientation.yaw` 没有磁力计绝对参考，不能作为生产绝对航向。

Supervisor 只有在下列条件连续稳定后才开始 S1：`/cmd_vel` 有底盘订阅者、IMU、EKF 里程计、雷达、相机和 `/map` 都持续有新消息，且启动雷达定位已经锁定并发布 `map -> base_footprint`。发现上一轮残留的阶段节点会拒绝本轮会话。

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

控制器初始为 `standby/ready`，只有 `/competition/stage1/activate` 后才会发布 `/cmd_vel`。S1 的生产运动链采用三层约束：`map_restricted` footprint 膨胀后的航向感知全局搜索（允许前进/倒车，带最小转弯半径、倒车和换挡惩罚）、基于 `/scan` 临时障碍的短时域 MPPI 风格轨迹采样，以及独立 TTC/footprint 硬安全层。地图外、未知格和黑色区域都是硬障碍；扫描障碍不写回永久地图；没有安全轨迹时才安全保持零速。S1 起点直接采用雷达锁定的 Map 位姿，后续将 `/odom_combined` 的 XY 相对起点旋转到该 Map 起始航向；IMU 仅以带消息时间戳的 `/imu/data.angular_velocity.z` 积分航向。标量 ODOM 距离仅用于起步直行距离和诊断，不能替代二维位置增量。`map_heading_lidar` 只传递启动锁定的绝对方向，不参与行进中的再校正。行进扫描按自身时间戳取历史 Map 位姿，以 IMU 航向固定角度后与静态地图障碍边缘做 XY 一致性匹配：连续稳定的小偏差只补偿 Map XY，超过自动补偿上限的可信偏差会记录具体 `dx/dy` 并触发定位质量门停车；证据不足时不校正也不猜测。绝不使用 Madgwick 的 orientation yaw，也绝不使用里程计 orientation。每个任务段只保留一条已经通过静态足迹验证的全局路线，普通位移不触发重规划；仅在任务目标、地图或局部安全轨迹持续失效时重新规划。

扫码完成时立即清空当前全局路径并发布 `competition_qr_task`；S1 先从实时位姿规划到二维码目标，二维码到位后再次从实时位姿规划到通道入口 `(2.50,2.50)`，对齐名义 `90°` 后发布 `stage1_state=handoff_ready` 和 `stage2_entry_pose`，进入 `handoff_wait` 并保持最后的有效非零命令。生产路径不再使用盲开中线、固定 `back_target_x` 倒车或墙体 map-X 自动校正。

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

- `/odom_combined` 只提供 XY、距离和位移；绝不使用其 orientation/yaw 导航。
- `/imu/data` 是所有阶段唯一的航向和转角来源。
- S1 交接记录 `stage2_entry_pose`；S2 交接记录 `stage3_entry_anchor`。
- 三阶段持续从 `map -> base_footprint` 输出真实 `[POSE_REAL]`。

每次比赛使用统一 `COMPETITION_SESSION_ID`，日志写入：

```text
log/competition_runtime/<session_id>/
log/competition_stage1/<session_id>/latest.log
log/competition_stage2/<session_id>/latest.log
log/competition_stage3/<session_id>/latest.log
```

各阶段目录下的 `latest.log` 只是指向最新会话的软链接，历史事故日志不会被覆盖。终端只输出 `[STARTUP]`、`[TASK]`、`[POSE_REAL]` 和 `[ERROR]`。

## 7. 独立测试

三个 `competition_stage*.launch.py` 可以用于隔离测试，但测试 phase/task 必须是隔离话题，不能启动生产阶段 relay，也不能影响 `competition_total.launch.py`。
