# 修复历程总览（chronological）

> 统一记录：2026-06-06 ~ 2026-06-26  
> 范围：`racing_stage2_param_test` + 相关配置

---

## 2026-06-06 — Fast DDS SHM 端口初始化报错

启动时打印 `Failed init_port fastrtps_port7417`（非致命，SHM 端口初始化失败）。

**修复**：在 launch 头部加 `SetEnvironmentVariable` 禁用 SHM，强制走 UDPv4。

**文件**：`launch/direct_inertial_test.launch.py`

---

## 2026-06-06 — 紧急停车改为硬杀所有相关进程

`OnShutdown` 循环发零速度不可靠。

**修复**：改为 `pkill -9` 杀所有 racing/ROS/视觉/雷达/底盘/定位进程，补发一次零速。新增 `~/dev_ws/panic_stop.sh`。

**文件**：`launch/direct_inertial_test.launch.py`、`panic_stop.sh`

---

## 2026-06-06 — 90° 转角分析（数据驱动）

从 `latest.log` 发现首段 `rect_enter_align` 目标 +90°，执行 13 秒超时，实际 yaw 仅从 0.1° → 35.2°。

**根因**：`cmd_angular=0.75 rad/s`，`wheel_odom_angular=0.08 rad/s`，执行效率仅 10.7%。底盘 STM32 固件内部限幅导致差速不足，两个轮子几乎同速。

**建议**：降低 `turn_angular_speed`、增加 `segment_timeout`、`turn_linear_speed` 归零。

**文件**：`docs/STAGE2_90DEG_TURN_ANALYSIS.md`

---

## 2026-06-25 — 阶段二测试包综合修复（BUG-1~4）

### BUG-1：避障阻塞段完成（严重）
`_try_avoid_step()` 在段完成检查之前 return，运动段永远无法结束。
- **修复**：段完成检查移到 `_try_avoid_step()` 之前，段完成为最高优先级。

### BUG-2：vision_inertial_tester 参数名不匹配 + 缺 YAML（严重）
参数名 `vision_lost_timeout` vs launch 传入 `vision_lost_timeout_sec` 不匹配，转弯参数无 YAML 覆盖回退到低值。
- **修复**：统一参数名 + 补充 YAML 加载。

### BUG-3：field_track_*.yaml 缺失
两个 YAML 文件从未创建，代码仍用启动参数 `rectangle_*_leg_m`。
- **修复**：创建完整 9 段定义（含补偿角）。

### BUG-4：bpu_direct_test.py 硬编码路径
模型路径 + 日志目录硬编码。
- **修复**：改用 `ament_index_python` 运行时定位。

---

## 2026-06-25 — 第二批修复（BUG-5~10）

### BUG-5：DataRecorder CSV 空列
`seg_type`/`seg_desc` 始终为空。
- **修复**：移除这三列。

### BUG-6：vision_inertial_tester 零除
`turn_angular_speed=0` 时崩溃。
- **修复**：加 `max(0.01)` guard。

### BUG-7：矩形参数默认值冲突
代码默认 `1.20/0.60/2.80` vs launch 默认 `1.10/0.50/3.00`。
- **修复**：统一为 `1.10/0.50/3.00`。

### BUG-8：避障参数隐藏 clamp
`detour_front_angle_deg` 被 clamp 到 35°。
- **修复**：移除隐藏 clamp。

### BUG-9：camera_video_recorder 输出路径
默认写在工作空间根目录。
- **修复**：改为 `log/video/`。

### BUG-10：死代码标记
`s1_executor.py` + `maybe_inject_detour()` 未被调用。
- **修复**：加 DEPRECATED 标记。

---

## 2026-06-25 — build_ring_plan() YAML 迁移

### 架构变更
- 删除 9 个旧参数（`rectangle_first_leg_m`、`rect_*_deg` 等）
- 新增 `field_track_yaml` 单参数
- 新建 `field_track.py` 模块：YAML 加载 + 段构建
- `build_ring_plan()` 从 65 行硬编码 → 6 行调用 `field_track.load_plan()`
- 启动文件移除旧参数声明

### 补全
第一次迁移未命中 `__init__` 中旧参数声明，`self._field_track_yaml` 未赋值导致 `AttributeError`。
- **修复**：精确索引替换，删除全部旧参数，注入正确声明。

---

## 2026-06-26 — counterclockwise YAML 加载修复

**问题**：`test_direction:=counterclockwise` 仍加载 `field_track_clockwise.yaml`，`ValueError`。

**根因**：`self.direction` 来自父类 `task_raw`（测试模式为 None），回退到 `clockwise`。

**修复**：改为使用 `self.test_direction`（来自 launch 参数）。

**文件**：`direct_inertial_tester.py`

---

## 2026-06-26 — finish_mission() 双重 shutdown 修复

**问题**：`finish_mission()` 内调用 `rclpy.shutdown()`，与 `spin_until_stop` 的 `Ctrl+C` 处理冲突。

**修复**：移除 `finish_mission()` 中的 `rclpy.shutdown()`，由 `main()` 统一处理。

**文件**：`direct_inertial_tester.py`

---

## 2026-06-26 — corner_2 角度 +90° & shutdown 补全

**修正**：`field_track_*.yaml` 中 `rect_corner_2` 角度修正为 +90°。`main()` 中增加 `node._request_stop = request_stop`

**文件**：`field_track_clockwise.yaml`、`field_track_counterclockwise.yaml`、`direct_inertial_tester.py`

---

## 2026-06-26 — 数据记录器 data_recorder

新增独立 ROS 2 节点，20Hz（后改为 1Hz）CSV 记录，每次运行覆盖旧文件。

**记录内容**：wheel/ekf/imu 三路位姿 + 三段 cmd_vel 指令 + 激光雷达 + 状态反馈。

**集成**：已嵌入 `direct_inertial_test.launch.py`、`vision_inertial_test.launch.py`、`lane_follow_test.launch.py`。

**文件**：`racing_stage2_param_test/data_recorder.py`

---

