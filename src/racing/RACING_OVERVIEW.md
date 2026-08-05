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
- map server、lifecycle manager、`map -> odom_combined` TF；
- 语音节点；
- `competition_supervisor`。

生产环境没有 BNO055。`/imu/data` 的来源固定为底盘 IMU 经 Madgwick 过滤后的链路。

Supervisor 只有在下列条件连续稳定后才开始 S1：`/cmd_vel` 有底盘订阅者、IMU、EKF 里程计、雷达、相机和 `/map` 都持续有新消息，且 `map -> base_footprint` 可以查询。发现上一轮残留的阶段节点会拒绝本轮会话。

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
  -> S3 running -> P point zero speed -> S3 self-exit
  -> Supervisor exits -> top-level launch closes base layer
```

### S1

S1 只包含 `competition_controller` 与 `qr_scanner`。相机属于公共基础层；S1 不加载视觉 AI，也不加载 BNO055。

控制器初始为 `standby/ready`，只有 `/competition/stage1/activate` 后才会发布 `/cmd_vel`。扫码完成时发布 `competition_qr_task`；到通报口时发布 `stage1_state=handoff_ready` 和 `stage2_entry_pose`，并进入 `handoff_wait`，保持最后的有效非零命令。

### S2

基础层稳定后 Supervisor 即启动 S2 和 S2 专属视觉 AI 的 standby 进程。二维码只负责传递并锁存方向，S2 在 `prewarming/ready` 时加载参数、模型，但绝不发布运动命令。S1 的 `stage2_entry_pose` 是地图锚点，不是激活时当前位置；S2 记录收到锚点时的原始 TF，Supervisor 的 activate 到达后重新读取实时 TF/IMU，再输出第一条连续 `/cmd_vel` 并发布 `stage2_state=handoff_command_ready`。锚点延迟只记录诊断，不因年龄单独拒绝。随后 S1 收到 release，自行退出。

S2 完成时发布 `stage3_entry_anchor` 与 `stage2_state=complete`，保持最后有效命令直到 S3 接管。S2 后段的 `stage3_prewarm` 事件只预热 S3，不改变 S2 的运动权。

### S3

S3 standby 时加载 P 点视觉资源并等待合法的 `stage3_entry_anchor`，不发布运动命令。锚点允许在 transient-local 交付中延迟到达；S3 以锚点配合 `/odom_combined` 的 XY 位移增量建立当前位姿。activate 后输出第一条连续 `/cmd_vel` 并发布 `stage3_state=handoff_command_ready`，然后 S2 收到 release 并自行退出。

到达 P 点是唯一正常终停：S3 锁定零速、输出最终 `[POSE_REAL]`、发布 `stage3_state=complete`、释放视觉资源并自行退出。Supervisor 只确认它已消失，随后退出；顶层 launch 因此自然结束，不需要额外 Ctrl+C。

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
