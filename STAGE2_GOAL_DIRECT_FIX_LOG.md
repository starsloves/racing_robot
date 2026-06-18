# Stage2 避障（goal_direct）方案与测试记录

> 工作区：`dev_ws` · 包：`racing_stage2_param_test`  
> **用途：** 记录每条修复/调参是否 **已试、结果、是否保留**；新 agent **必须先查本文**，避免重复踩坑。  
> **§1 goal_direct**：历史 offline 分支（`world_segment` / bypass-rejoin-exit）；**当前仓库实机代码**见 **§1b**（`direct_inertial_tester.py` Stage1 式避障）。  
> **明确不做**：单步 DWA 换 (v,ω) 作主控制；恢复 `next_leg → rect_return_origin` 斜切；长段静态身后 rejoin。  
> **实机最新（2026-06-05 下午）：** Stage1 避障已在父类（§1b）；**轮速 `/odom` 位姿/航向/计程/控制完全统一**（§1b.8，Run H）；**leg2=0.40 m**；全量日志 **`log/direct_inertial_test/latest.log`**（每次 launch 覆盖）。**接续：场测 H1/H2**（§1b.8 表）；offline goal_direct（§1）与实机 wheel 源尚未统一（H4）。

---

## 0. 使用说明（给 Agent）

1. 提出新改法前，在本文 **§2 已试**、**§3 未试**、**§1b**（参数测试实机/S1）、**§1b.8**（wheel 位姿源）中检索关键词（如 `next_leg`、`rejoin`、`avoid_leg`、`navigation_pose_source`、`latest.log`）。
2. 若 **已试且失败**，不要原样重试；除非写明与上次差异。
3. 每次 offline full 或专项跑完后，在 **§4 测试记录** 追加一行/一节，并更新 **§1 当前 baseline**。
4. 汇总文件：`log/stage2_param_test/汇总/test_summary.txt`  
5. 算法用户向说明：`src/racing/racing_stage2_param_test/docs/AVOIDANCE.md`
6. **实机绕圈/直行：** 先读 `log/direct_inertial_test/latest.log`；位姿统一规则见 **§1b.8**；场测清单 **H1～H4** 同节。

**验收门槛（offline）：** `min_clearance_m > -0.02` 且 `mission_finished=True`（`scenario_passes`）

**验证命令**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racing_stage2_param_test
python3 -m racing_stage2_param_test.auto_offline_test --group full
python3 -m racing_stage2_param_test.auto_offline_test --scenarios rect_side_2_50 rect_return_75
```

---

## 1. 当前 baseline（2026-05-25 · Run D）

| 项 | 说明 |
|----|------|
| 坐标 | 世界系 P0+ψ（`world_segment.py`）；名义折线 `simulate_plan_world_poses` |
| FSM | 长段：`bypass → pass → rejoin`；短边/段末 cut：`bypass → pass → exit` |
| rejoin | **动态** `along = max(progress+ahead, s_obs+span)`，每拍刷新 |
| direct_cut | **不用** `next_leg`；`exit` 控制 = **贴段 PD** + `current_exit_lateral_m()` 衰减 |
| 内绕 | `select_bypass_side`：**仅短边**（`seg_len < 0.68m`）clearance 够时优先 `preferred -1` |
| handoff | exit 须 `\|lat\| ≤ handoff_limit`；末段 **`finish_proximity ≤ 0.40m`（有副作用，见 §2.4、§8）**；竖边 cut → turn |
| trim | `segment_end_trim` 航向优先（>12° / >45°）；末段距 finish ≤0.42m 跳过 trim |

**最新 full 组（`--group full`）**

| 指标 | 值 |
|------|-----|
| PASS / FAIL | **11 / 9**（Run C：9 / 11） |
| 顶边 25/50/75 | PASS；**90 FAIL**（dist_finish=0.576m，回归） |
| 回程 | **25/50/75/90 全 PASS** |
| 竖边 | 全 FAIL；`side_2_50` clr **-0.081m**；handoff `\|lat\|≈0.16m`（曾 0.32m） |

<details>
<summary>展开：20 场景明细（Run D）</summary>

```
full_ring_no_obstacle: PASS  clr=0.000
rect_first_leg_25:       FAIL clr=-0.107
rect_first_leg_50:       PASS clr=-0.001
rect_first_leg_75:       PASS clr=+0.061
rect_first_leg_90:       FAIL clr=+0.057  dist_finish=0.413
rect_return_25:          PASS clr=+0.047
rect_return_50:          PASS clr=+0.044
rect_return_75:          PASS clr=+0.072
rect_return_90:          PASS clr=+0.030
rect_side_1_25:          FAIL clr=-0.289
rect_side_1_50:          FAIL clr=-0.180
rect_side_1_75:          FAIL clr=-0.056
rect_side_2_25:          FAIL clr=-0.064
rect_side_2_50:          FAIL clr=-0.081
rect_side_2_75:          FAIL clr=-0.093
rect_top_25:             PASS clr=+0.055
rect_top_50:             PASS clr=+0.084
rect_top_75:             PASS clr=+0.114
rect_top_90:             FAIL clr=+0.147  dist_finish=0.576
```

</details>

**仍存根因**

| 现象 | 根因（Run D 后） |
|------|------------------|
| 竖边 clearance 负 | handoff `\|lat\|≈0.16m` 仍偏大；0.50m 竖边通道窄 |
| 底边 25/90 | 擦锥或 dist_finish 超 0.40m |
| 顶边 90 | **回归**：finish 定位偏差（非 clearance） |

---

## 1b. 参数测试避障 — Stage1 边转边避（2026-06 合并完成）

> **现状：** 边转边避（S1 风格）状态机已实现在 **父类** `DirectInertialTester`，两个测试共享：  
> - **绕圈** `direct_inertial_test.launch.py`（矩形 1.10×0.50×2.80m）  
> - **直行台架** `avoidance_straight_test.launch.py`（直线 3.00m）  
> **子类** `AvoidanceStraightTester` 继承父类 `_try_avoid_step()`，仅覆盖参数默认值（leg1=0.30, leg2=**0.40**）。
>
> **新做：** Phase 8 里程超限 + 转角障碍（§1b.7）；**Phase 9** 轮速位姿统一 + 会话日志（§1b.8）。

### 1b.1 当前实机 baseline（合并后）

| 节点 | 避障 | 说明 |
|------|------|------|
| **父类** [`direct_inertial_tester.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/direct_inertial_tester.py) | **Stage1 边转边避**（`_try_avoid_step()`） | 6 态：IDLE→TURN_AWAY→LEG1→TURN_BACK→LEG2→TURN_RECOVER→FINE_ALIGN→IDLE |
| **绕圈启动** `direct_inertial_test.launch.py` | 同上（继承父类） | 默认 leg2=0.40；YAML 可覆盖 |
| **直行台架** [`avoidance_straight_tester.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/avoidance_straight_tester.py) | 继承父类 `_try_avoid_step()`；覆盖参数默认值 | 默认 leg1=0.30, leg2=**0.40**；直行距离 3.00m @ 0.20m/s |

**Stage1 边转边避统一判据（合并后）：**

| 项 | 说明 |
|----|------|
| 触发 | `move` + `allow_detour` + `detour_enabled`；`front_obstacle_distance ≤ detour_obstacle_distance`（默认 **0.48 m**） |
| FSM | `idle → turn_away → leg1 → turn_back → leg2 → turn_recover → fine_align → idle` |
| 转弯控制 | **开环转角积分**（**`current_yaw`=/odom** 差值累加，>15° 单帧跳变过滤；§1b.8 后不再用裸 IMU）；`avoid_turn_angular_speed=0.40` |
| 直行控制 | **ω=0 不纠航**；欧几里得距离 `math.hypot(dx, dy)` 从 leg 起点计数 |
| FINE_ALIGN | **闭环 P** 修正残余航向误差（`heading_kp=1.6`） |
| 冷却 | 避障结束后 **`detour_cooldown_sec=2s`** 内不重复触发（`_avoid_cooldown_until`） |
| 段完成 | **里程投影** `projected_distance()` vs `distance_m`（**2026-06-05 起位姿源统一为 `/odom`，见 §1b.8**） |
| 未接入 | `maybe_inject_detour()` 在 `run_move_segment` **未调用** |

### 1b.2 S1 几何两脚（合并后 · 6 态 + FINE_ALIGN）

**原则：** 避障子过程 **不用时间** 切换阶段；用 **开环转角积分** + **平面位移** 判定每步结束。

**符号：** ψ₀ = 本段 `segment_heading`，ψ₁ = ψ₀ ± offset（转离障碍），ψ₂ = ψ₀ ∓ offset（越过 ψ₀ 到另一侧）。

**选边（与激光一致）：**

| 障碍方位 | 第一脚目标 | 第二脚目标 |
|----------|------------|------------|
| 偏左（`obstacle_is_left`） | ψ₀ **− 30°** | ψ₀ **+ 30°** |
| 偏右 | ψ₀ **+ 30°** | ψ₀ **− 30°** |

**参数表（合并后）：**

| 参数 | 父类默认值 | 直行台架覆盖 |
|------|:---------:|:----------:|
| `avoid_leg_heading_offset_deg` | 30.0 | 30.0 |
| `avoid_leg1_distance_m` | 0.30 | 0.30 |
| `avoid_leg2_distance_m` | **0.40** | **0.40** |
| `avoid_leg_linear_speed` | 0.10 | 0.10 |
| `avoid_turn_linear_speed` | 0.08 | 0.08 |
| `avoid_leg_distance_tol_m` | 0.04 | 0.04 |
| `avoid_turn_angular_speed` | 0.40 | 0.40 |

**示例（ψ₀=86°，障碍在右）：** 86° → **116°**（+30°）走 **0.30m** → **56°**（-30°）走 **0.40m** → FINE_ALIGN 回 86°。

**状态机（6 态 + FINE_ALIGN）：**

```
idle ──(触发)──► turn_away    开环转 ±30°，vt=0.08m/s
turn_away ──(累计转角≥|offset|−2°)──► leg1    ω=0，vl=0.10m/s，走 leg1 距离
leg1 ──(位移≥leg1−tol)──► turn_back   开环反向转 ∓60°，vt=0.08m/s
turn_back ──(累计转角≥|2×offset|−2°)──► leg2    ω=0，vl=0.10m/s，走 leg2 距离
leg2 ──(位移≥leg2−tol)──► turn_recover   开环转回 ±30°，vt=0.08m/s
turn_recover ──(累计转角≥|offset|−2°)──► fine_align   闭环 P 修正残余航向
fine_align ──(|yaw−ψ₀|≤4°)──► idle（冷却 2s，避障结束）
```

**转角积分：** `_avoid_accumulate_turn(yaw)` — 当前帧 yaw − 上一帧 yaw，归一化；单帧 >15° 视为 IMU 跳变跳过不累加；只累加与 `_avoid_turn_sign` 同方向的步长。

**FINE_ALIGN：** 开环转角完成后残余误差 1-3°，闭环 P（`heading_kp=1.6`）修正到 `heading_tolerance`（4°）。

### 1b.3 实现落点（合并完成）

| 阶段 | 改哪里 | 状态 |
|------|--------|------|
| **① 合并入父类** | `DirectInertialTester` 内置 6 态状态机（`_try_avoid_step()`） | **已完成** |
| **② 直行台架子类** | `AvoidanceStraightTester` 继承父类，覆盖参数默认值 | **已完成** |
| **③ 参数覆盖** | 父类默认 leg1=0.30, leg2=**0.40**；直行台架同 leg2=0.40（YAML 可覆盖） | **已完成** |
| **④ 冷却机制** | `_avoid_cooldown_until` 防避障结束后立即重复触发 | **已完成** |
| **⑤ 里程超限保护** | `_estimate_avoid_projection_m()` 预计算 + 运行时超限直接终止 | **已完成** |
| **⑥ 转角障碍检测** | `run_turn_segment()` 障碍<0.24m 时原地转向 | **已完成** |

**① 验证命令（直行台架）：**

```bash
ros2 launch racing_stage2_param_test avoidance_straight_test.launch.py
```

**② 验证命令（绕圈）：**

```bash
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py
```

### 1b.4 本阶段明确不做

| 项 | 说明 |
|----|------|
| 段剩余严重不足（<0.30m） | 当前预计算安全网兜底；极端不足不另做策略 |
| 转角主动避障（非仅停止前移） | 目前仅 `linear=0` 原地转向；转角障碍由下一段 move 避障处理 |
| DWA 主控制 | 同 §5 |

### 1b.5 待上线前再定的边界（备忘）

| # | 问题 | 候选 |
|---|------|------|
| B1 | leg1/leg2 行进中前方仍 ≤ 0.48 m | 忽略继续几何步 / 或停线速度 |
| B2 | 段初航向门限 | 保留 `\|yaw−ψ₀\| > 12°` 不触发 |
| B3 | odom 打滑导致距离不准 | 日后可加激光「前方持续清空」作辅助，**不作时间兜底** |
| B4 | FINE_ALIGN 时障碍再次逼近 | 选：放弃修正立即回 IDLE / 完成修正但取消冷却 |

### 1b.6 合并后 · 绕圈/比赛

- **已做：** S1 边转边避在 `direct_inertial_tester`；绕圈与直行台架共用；wheel 位姿源 §1b.8。
- **Phase 8：** 避障 zigzag 投影 ~1.03–1.13 m 超段剩余 → 触发前跳过 / 运行中终止避障、段自然过渡下段（**不用** `emergency_recover`）；`run_turn_segment` 前方 <0.24 m → `linear=0` 原地转。
- **未做：** `field_track_clockwise.yaml` 世界路点替代 `rectangle_*_leg_m` 链式计程；段末剩余不足两脚时直接切 plan 下一段，不硬凑 50+50 cm。

### 1b.7 Phase 8 新做：里程超限预计算 + 转角障碍检测

#### 背景

避障 zigzag 全程沿段航向的投影 ~1.03~1.13m（取决于 leg2 距离和 offset）。如果段长只有 1.10m（绕圈底边），触发避障时剩余距离可能不够走完整套，导致 TURN_RECOVER/FINE_ALIGN 时 `projected_distance()` 已超过段目标。

实车日志证实：1.10m 段上避障在 LEG2 末尾里程达 1.06/1.10m，段已完成但避障未结束。

#### 预计算 `_estimate_avoid_projection_m()`

```python
# 转向阶段精确投影 = (vt/w) * ∫cos(θ)dθ
ta_proj = (vt/w) * sin(offset)                 # TURN_AWAY: 0→+offset
leg1_proj = leg1 * cos(offset)                 # LEG1
tb_proj = (vt/w) * (sin(offset)-sin(-offset))  # TURN_BACK: +offset→-offset
leg2_proj = leg2 * cos(offset)                 # LEG2
tr_proj = (vt/w) * (0 - sin(-offset))          # TURN_RECOVER: -offset→0
fine_proj = 0.02                               # FINE_ALIGN 小修正
```

父类/台架默认（leg1=0.30, leg2=**0.40**, offset=30°）→ **~1.03m**（leg2 曾 0.50 时约 ~1.13m，已改回 0.40）。

#### 触发前跳过

`_should_trigger_avoid()` 新增：`remaining < estimated_projection - 0.05m` → 跳过触发，日志。

#### 运行中超限应急

`_try_avoid_step()` 中每帧检查（排除 `fine_align` / `idle` 状态）：
- `projected_distance() >= target_m - distance_tolerance`
- 若超限：设 `pending_segment_start_yaw = segment_heading`（下段转弯从正确的 ψ₀ 算目标）→ `_reset_avoid()` → `return False`
- **不原地修正航向**（用户明确否决 `emergency_recover` 方案）
- 下段转角段的 PID 从当前偏航角自动转到目标，无需事先回正

#### 转角障碍检测

`run_turn_segment()` 中：
- `front_obstacle_distance < detour_obstacle_distance * 0.5`（~0.24m）→ `linear_speed = 0.0`
- 机器人原地转向，不撞入障碍物。下一段 move 自然触发避障处理该障碍。

#### 用户明确否决的方案

| 否决方案 | 原因 |
|----------|------|
| **`emergency_recover`**（超限后停着不动 + 原地回正到 ψ₀） | 用户预期：超限应直接过渡到下一段，不应停车修正 |
| **FINE_ALIGN 中线性速度强制归零** | 修正量小（<0.02m 投影），没必要掐 |

### 1b.8 统一位姿源 `/odom`（2026-06-05 · Agent 对话合入）

> **现象（实车 `latest.log`）：** 启动后有时「里程不计」；或直行时角度乱跳、`cross` 涨到 −35 cm。  
> **根因：** 位置来自 `/odom`，但 **转弯/纠航/段航向曾用 IMU**；Madgwick 与编码器零点差 **~157°**，`projected_distance` 投影轴与真实运动不一致；EKF `/odom_combined` 偶发先于轮速就绪、位置停在 (0,0)。

#### 统一规则（`navigation_pose_source: wheel`）

| 用途 | 数据源 |
|------|--------|
| `current_position` (x,y) | `/odom`（origincar_base 编码器积分） |
| `current_yaw`（控制/转弯/计程轴） | `/odom` 四元数，**每条轮速消息 `_sync_unified_pose_from_wheel()`** |
| `segment_heading` / 转弯目标 | 段切换时 `_unify_segment_pose()`，与 `current_yaw` 一致 |
| `projected_distance()` | 轮速 xy + `segment_heading`（`max(0, along)`） |
| IMU `/imu/data` | 仅 `imu_yaw` / `imu_off` 诊断，**不写 `current_yaw`** |
| EKF `/odom_combined` | 仅 `ekf_xy` / `yaw_ekf` 诊断，**不写位姿**（wheel ready 后） |

#### 启动门槛

- `wheel_odom_warmup_min_msgs=5`，`wheel_odom_warmup_sec=0.40` 后才 `mission` 开跑。  
- **不再**要求 IMU 就绪才能启动（wheel 模式下）。

#### 配置文件

| 文件 | 作用 |
|------|------|
| [`config/direct_inertial_test.yaml`](src/racing/racing_stage2_param_test/config/direct_inertial_test.yaml) | 绕圈测试：`navigation_pose_source`、warmup、leg2=0.40、日志目录 |
| [`config/avoidance_straight_test.yaml`](src/racing/racing_stage2_param_test/config/avoidance_straight_test.yaml) | 直行台架：同上 wheel 源 |

`direct_inertial_test.launch.py` 已加载 `direct_inertial_test.yaml`（叠在 `inertial_stage2.yaml` 之上）。

#### 会话日志（实车分析用）

- 路径：**`log/direct_inertial_test/latest.log`**（每次 launch **覆盖**）。  
- 终端启动时打印：`会话日志: .../latest.log`。  
- 关键标签：`CONFIG` `STARTUP` `ODOM_WHEEL` `ODOM_ANCHOR` `SEGMENT` `TELEM` `PROGRESS` `DETOUR` `FEEDBACK`。  
- `TELEM` 字段：`yaw`（=控制航向）、`yaw_wheel`、`yaw_imu`、`yaw_leg`、`along`/`raw_along`、`cross`、`wheel_v`、`cmd_v`、`front/left/right` 障碍距。

**验收（日志）：**

- `ODOM_ANCHOR`：`yaw` ≈ `yaw_wheel` ≈ `yaw_leg`；顺时针首转目标 **~+90°**（不是 IMU 的 **−113°**）。  
- 直行 `TELEM`：`yaw` 与 `yaw_wheel` 同步变化；`head_err` 小；`along` 随动车增长。  
- `yaw_imu` 可与 `yaw` 差很多，**只要不再参与控制即可**。

#### 实车日志摘录（修复前 · 说明根因）

```
ψ_wheel=0.1°   ψ_imu=157°     # 启动：两套零点差 ~157°
target_yaw=-113°               # 转弯用 IMU：157+90
ψ_leg=89.5°  ψ_imu=-116°      # 计程用轮速、纠航用 IMU → 直行 cross -35cm，ψ_wheel 90°→2°
```

#### 接续任务 / 待验证

| # | 项 | 状态 |
|---|-----|------|
| H1 | 重跑 `direct_inertial_test`，确认 `yaw` 统一后首段 1.10m 走直、`along` 满程 | **待场测** |
| H2 | 重跑 `avoidance_straight_test`，确认 leg2=0.40m 与 wheel 源 | **待场测** |
| H3 | 若 `imu_off` 长期 >45° 且需 IMU 融合，单独做 **IMU↔odom 标定**（非本阶段） | 未做 |
| H4 | offline goal_direct（§1）与实机 wheel 源 **尚未统一**；offline 仍用仿真 odom | 已知 |

```bash
colcon build --packages-select racing_stage2_param_test
source install/setup.bash
ros2 launch racing_stage2_param_test direct_inertial_test.launch.py
# 日志
less log/direct_inertial_test/latest.log
grep -E 'ANCHOR|TELEM|PROGRESS|imu_off' log/direct_inertial_test/latest.log
```

---

## 2. 已尝试方案

### 2.1 架构 / 大改（2026-05 世界坐标重构）

| ID | 方案 | 结果 | 保留 | 备注 |
|----|------|------|------|------|
| R0 | **DRIVEN 弦线 + corridor_track** | 弃用 | 否 | 短边 S 弯、side_2 clearance≈-0.27m |
| R1 | **goal_direct + 世界路点** | 采用 | 是 | `world_segment.py`、`ring_track` 链式折线 |
| R2 | 删 `OfflineRingHarness` / chord / anchor | 完成 | 是 | `offline_runner.py` + `hardware_sim.py` |
| R3 | 短边 **`next_leg = upcoming_move_entry_world()`** | **失败** | 否 | 右边 50% 斜插 `rect_return_origin`；**已改为 exit** |
| R4 | 长段 **静态 rejoin**（`s_obs+half_span` 固定） | **失败** | 否 | 顶边 50% rejoin ~58s 绕圈；**已改动态 rejoin** |
| R5 | **`bypass → pass → exit`**（direct_cut） | 部分有效 | 是 | 消除斜插；竖边仍擦锥 |
| R6 | **动态 rejoin**（ahead + progress） | 有效 | 是 | 顶边 4 场景全 PASS |
| R7 | **内绕优先（全局）** | **失败** | 否 | 顶边 50/75 从 PASS 变 FAIL；**改为仅短边** |
| R8 | **内绕优先（仅短边 seg<0.68m）** | 部分有效 | 是 | 右边 50% `side=-1`；clearance -0.21→-0.08 |
| R9 | handoff 取消 88% **仅 next_leg** 捷径 | 部分有效 | 是 | 改为 `segment_end_progress_threshold` |
| R10 | `segment_along_past_locked_obstacle` + exit handoff | 部分有效 | 是 | 回程仍 stuck（yaw 问题未解） |
| R11 | direct_cut 后 **`start_segment(+1)` 进 turn** | 有效 | 是 | 竖边已验证（`右边_50%` → `rect_corner_4`）；**回程无 turn 不适用** |
| R12 | 短边 rolling bypass **加大 along**（direct_cut） | 略好 | 是 | bypass 沿程略快 |
| **T1** | **`exit` 贴段 PD**（`goal_direct_exit_segment_cmd`） | 有效 | 是 | 回程 75/90 stuck→PASS |
| **T2** | **`current_exit_lateral_m()` 横向衰减** | 部分 | 是 | handoff lat 0.32→0.16；clr 仍负 |
| **T3** | **handoff 收紧** + 6s 超时须 `\|lat\|` | 部分 | 是 | 禁 dist 捷径；延长 exit 收敛 |
| **T4** | **`segment_end_trim` 航向护栏** + finish_proximity | 部分 | **trim 保留；finish_proximity 待改** | 回程不 stuck；但图未回底边、顶边 90 回归 |
| **P6** | **Stage1 边转边避合并入 `DirectInertialTester`** | 有效 | 是 | 父类内置 6 态 + FINE_ALIGN；子类继承，2026-06 |
| **P7** | **避障冷却机制 `_avoid_cooldown_until`** | 有效 | 是 | FINE_ALIGN 完成设冷却 2s；防避障刚结束立即重复触发 |
| **P7b** | **`return True` → `False` 当 `segment_heading` 为 None** | 有效 | 是 | 防 `_try_avoid_step` 返回 True 但避障未启动 → `run_move_segment` 卡死 |
| **P8a** | **里程超限预计算 + 运行中终止** | 有效 | 是 | `_estimate_avoid_projection_m()`；超限时终止避障 + 设 `pending_segment_start_yaw` 保下段航向正确 |
| **P8b** | **转角障碍检测（`run_turn_segment`）** | 有效 | 是 | 障碍<0.24m 时 `linear=0` 原地转向 |
| **P9a** | **轮速 `/odom` 计程**（EKF 启动常卡 (0,0)） | 有效 | 是 | `wheel_odom_topic=/odom`；EKF 不覆盖 `current_position` |
| **P9b** | **`navigation_pose_source: wheel`**（xy+yaw+计程+控制同源） | 有效 | 是 | `current_yaw` 由 `/odom` 同步；IMU/EKF 仅日志 |
| **P9c** | **`_unify_segment_pose()`** 段锚点与 `current_yaw` 对齐 | 有效 | 是 | 转弯目标用轮速航向，不再 IMU+90° |
| **P9d** | **会话日志** `log/direct_inertial_test/latest.log` | 有效 | 是 | 每次 launch 覆盖；`TELEM` 含 yaw/v/cmd/雷达 |
| — | **`emergency_recover`（先停再回正）** | **否决** | 否 | 用户要求超限应直接过渡到下一段，不应停车 |
| — | **IMU 与轮速混用控制** | **否决** | 否 | 实车 ψ_imu≈157° vs ψ_wheel≈0°，直行拧成弧线（§1b.8） |

### 2.4 Run D 调试迭代 — 试过且有副作用 / 未保留的改法

> T1～T4 合入过程中在同一 session 里反复试过的 **中间态**；勿原样再加回。

| ID | 改法 | 结果 | 保留 | 备注 |
|----|------|------|------|------|
| D1 | **`finish_proximity` 欧氏距离 ≤ 0.40 m** 即 `finish_mission` | **副作用** | **待改/收紧** | 回程 75 在 **(0.53, 0.47)** exit 中途 finish；y 比名义底边高约 **0.34 m**；测试 PASS、**图未回到目标** |
| D2 | **`segment_end_trim` 段末 forward crawl**（v=0.04～0.12） | **失败** | 否 | 回程 overshoot：progress 1.55→1.86 m，终点 dist_finish **0.688 m** |
| D3 | **exit 阶段 `progress ≥ end_thresh` 时 linear=0** | **失败** | 否 | exit 冻结，lat≈0.31 不收，**OFFLINE_STUCK** |
| D4 | **exit 阶段 `progress ≥ target - tol` 时 linear=0** | **失败** | 否 | 同上，段末无法 creep 回线 |
| D5 | **6 s 超时 handoff 无条件 return True**（T3 前） | **失败** | 否 | lat≈0.26 m 即 handoff → trim 空转 stuck |
| D6 | **trim 跳过 + finish_proximity ≤ 0.42 m**（末段 move） | **部分** | 否（0.42 门闩） | 与 D1 叠加，加剧「数字 PASS、轨迹不对」 |

**Run D 调试结论（回程 75% debug）：**

```
AVOID_EXIT reason=finish_proximity  progress=1.09m  lat=-0.30m
终点 (0.534, 0.466)  nominal_finish (0.477, 0.123)  dist_finish=0.348m → PASS
dist_origin=0.709m（用户期望回 (0,0) 是误解，见 §7）
```

### 2.2 历史调参（goal_direct 早期，git ~5b11d66）

| 方案 | 结果 | 备注 |
|------|------|------|
| rolling bypass（短边） | 部分有效 | 与 R12 同类 |
| lat_build 强掐 linear | **失败** | clearance ~-0.25m 或超时 |
| dynamic bypass 大步 anchor | **失败** | 刮桶加重 |
| corridor_sidestep ω 混合 | **失败** | 与 PP 打架 |
| stuck_handoff 在 pass 提前退出 | **失败** | 已限 exit/rejoin |
| need_direct_cut + exit 场内缩边（旧弦线） | 曾采用 | 符号 fix；现用 `segment_end_goal_world` |

### 2.3 明确否决（勿回退）

| 方案 | 原因 |
|------|------|
| `next_leg` 指 **隔 turn 的下一段 move**（尤其回程入口） | 右边「直插场地」 |
| 长段 **固定身后 rejoin** | 顶边绕圈 |
| **全局**内绕覆盖 clearance 逻辑 | 顶边 clearance 恶化 |
| 圆心距离圈 / 无约束世界 chase | 历史失控 |
| `rect_side_2` 硬编码分支 | 用户要求通用 |
| **DWA 主控制** | 改动面大；见 §5 |

| **DWA 主控制** | 改动面大；见 §5 |
| **`finish_proximity` 阈值 0.40～0.42 m** | 末段 **x 接近、y 仍偏高** 即 finish；图与验收不一致（§8） |
| **segment_end_trim 给 forward 爬行** | 段末 overshoot、dist_finish 恶化（D2） |
| **exit 在 progress≥end_thresh 时掐 linear=0** | exit 无法收 lateral，易 stuck（D3/D4） |
| **6 s 超时 handoff 不检查 lat** | 大横偏 handoff → trim 空转（D5） |
| **放宽 finish / scenario 阈值到 0.50 m** | 掩盖未回到名义终点 |
| **末段 finish 堆 progress + lateral + Δy 三门闩** | 冗余；末段应用 **世界坐标单距离**（§8） |

---

## 3. 未尝试 / 排队验证（下一步 · Run E）

> T1～T4 主体已合入；**finish_proximity 待替换**；竖边 clearance、顶边 90 仍待做。

| ID | 方案 | 针对 | 状态 |
|----|------|------|------|
| **E1** | **末段 finish：世界坐标 `dist(finish) ≤ 0.12～0.15 m`**（替换 0.40 m） | 回程/底边图与 PASS 一致 | **未试 · 推荐优先** |
| **E2** | 可选门闩：**回程 along ≥ 0.88×段长**（防 exit 绕障中途误 finish，仍用世界几何） | 与 E1 配套 | 未试 |
| **E3** | **竖边 exit lat 优先** + handoff `\|lat\| ≤ 0.10 m` | 左右超出 plan、擦锥 | 未试 |
| **E4** | 短边 `avoid_target_offset_m` 0.35→0.32（仅 `seg_len < 0.68 m`） | side_2 clearance | 未试 |
| **E5** | **plot**：蓝线含 enter 弯弧、(0,0) 与 nominal_finish 标注 | 判读/沟通 | 未试 |
| **E6** | turn 残差 / 无障整圈 drift 压减 | top_90、整圈 skew | 未试 |
| **S1** | **几何两脚避障**：ψ₀±30°，leg1=0.30 m、leg2=**0.40 m**、**v=0.10**、**不用时间**；详见 **§1b.2** | 已合并入 `direct_inertial_tester`；直行台架子类继承 | **已实现 · 待场测 H2** |

### 3.0 曾讨论、不建议的执行顺序（备忘）

| 顺序 | 内容 |
|------|------|
| 推荐（goal_direct offline） | **E1+E2**（finish）→ **E3**（竖边 lat）→ full → 不够再 E4/E6 |
| **当前实机** | S1 已在父类（§1b.3）；**优先场测** `direct_inertial_test`（H1）与 `avoidance_straight_test`（H2），对照 `latest.log` |
| 低优先 | 仅改 plot（E5）不改善控制，但减少「公式错了」误判 |

### 3.1 旧文档 P0/P1（尚未在 2026-05-25 后系统复测）

| # | 方案 | 状态 |
|---|------|------|
| P0-1 | 入障前 approach 限速加强 | 未单独 A/B |
| P0-2 | 真·先横后前（lat_build<0.75 时 v≈0） | 未试（曾类似 lat_build 掐 v **失败**） |
| P0-3 | 段初预转向 | 未试 |
| P1-4 | 短边 bypass max lateral 0.36 | 未试 |
| P1-5 | 放宽 need_direct_cut 判定（75% 段末） | 部分被 R5 exit 覆盖 |
| P1-6 | exit/pass margin 再向内 | 未试 |

---

## 4. 测试记录（按时间）

### Run A — 世界坐标重构后首跑 full（2026-05-25 早）

| 项 | 值 |
|----|-----|
| 改动 | P0–P4 重构（world_segment、offline_runner、next_leg 弯角、场景锥） |
| 命令 | `auto_offline_test --group full` |
| PASS/FAIL | **6 / 14** |
| 要点 | 无障 full ring PASS；顶边 50/75 PASS；右边 50 clearance **-0.210**；多竖边/回程 FAIL |

### Run B — 第一轮 goal_direct 修复（exit/rejoin/内绕，未限短边内绕）

| 项 | 值 |
|----|-----|
| 改动 | R3→R5 exit 序列、R4 动态 rejoin、R7 全局内绕、handoff 收紧 |
| PASS/FAIL | full 约 **6 PASS**；顶边 50/75 **退化 FAIL** |
| 结论 | **R7 全局内绕不可保留** |

### Run C — 第一轮修复 + 短边内绕 + handoff 修补（当前代码）

| 项 | 值 |
|----|-----|
| 改动 | R8 短边内绕、R9/R10 段末阈值与 along_pass、R11 确认 |
| 命令 | `colcon build` + `auto_offline_test --group full` |
| PASS/FAIL | **9 / 11** |
| 对比 Run A | 顶边 **4/4 PASS**；side_2_50 clr **-0.210→-0.076**；first_leg_90 不再 stuck |
| 日志 | `log/stage2_param_test/汇总/test_summary.txt` |

**Run C 关键日志结论**

| 场景 | 验证点 |
|------|--------|
| `右边_50%` | `side=-1`，`next_leg=none`，`phase=exit`，handoff → `rect_corner_4` |
| `顶边_50%` | `rejoin` 动态前移，~38s exit，clr +0.084 |
| `回程_75%` | AVOID_EXIT 后 yaw≈83°，`segment_end_trim` → **OFFLINE_STUCK_ABORT** |

### Run D — T1～T4 逐项 + full（2026-05-25）

| Step | 状态 | 结果 |
|------|------|------|
| T1 exit 贴段 PD | 完成 | return_75/90 **stuck→PASS**；first_leg_90 clr 改善仍 dist FAIL |
| T2 exit 横向衰减 | 完成 | side_2_50 clr -0.076→-0.081；AVOID_EXIT `\|lat\|` 0.32→0.16 |
| T3 handoff lat | 完成 | 禁 dist 捷径；6s/14s 超时须 lat 达标 |
| T4 trim 护栏 | 完成 | 航向优先 trim；末段 finish_proximity / trim 跳过 |
| **full** | 完成 | **11 PASS / 9 FAIL**（Run C：9/11）；未达目标 13 PASS |

**Run D 关键日志**

| 场景 | 验证点 |
|------|--------|
| `回程_75%` | `finish_proximity` handoff → mission_finished；clr +0.072 |
| `右边_50%` | AVOID_EXIT `\|lat\|=0.16m`；仍擦锥 clr -0.081 |
| `顶边_90%` | clr +0.147 但 dist_finish=0.576 → **回归 FAIL** |

*逐项明细：`log/stage2_param_test/汇总/incremental_results.txt`*

### Run D 后 — 轨迹图审阅结论（2026-05-25）

| 场景 | 轨迹 vs plan（1.10/0.50/2.80 m） | 说明 |
|------|----------------------------------|------|
| `回程_75%` | plan y∈[0.12,0.87]；traj y∈[0.00,**1.09**]；终点 **(0.53,0.47)** | finish_proximity 提前收工 |
| `右边_50%` | traj y 最低 **-0.32**（plan 底边 y≈0.12） | 绕障横偏 + 段间 drift 累积 |
| `底边_50%` | 终点 **(0.61,0.46)**，dist_finish=0.357 m | 同上，非 plan 公式错误 |

---

## 5. 不做：DWA 主控制

与旧版相同：**不做**单步 DWA 选 (v,ω) 替代 goal_direct。`motion_safe` / `dwa_clearance_along_motion` 仅碰撞检查。

---

## 6. 相关文件

| 文件 | 作用 |
|------|------|
| [`direct_inertial_tester_avoidance.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/direct_inertial_tester_avoidance.py) | goal_direct FSM（**offline 分支，本树可能缺失**） |
| [`direct_inertial_tester.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/direct_inertial_tester.py) | 绕圈 + 直行台架共用；**Stage1 边转边避已合并入父类** |
| [`avoidance_straight_tester.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/avoidance_straight_tester.py) | 直行台架子类；继承父类；leg2=0.40 |
| [`config/avoidance_straight_test.yaml`](src/racing/racing_stage2_param_test/config/avoidance_straight_test.yaml) | 直行 launch；`navigation_pose_source: wheel` |
| [`config/direct_inertial_test.yaml`](src/racing/racing_stage2_param_test/config/direct_inertial_test.yaml) | 绕圈 launch；wheel 源 + 日志 + leg2=0.40 |
| [`session_file_log.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/session_file_log.py) | 实机会话日志（`log/<subdir>/latest.log`，覆盖写） |
| **`log/direct_inertial_test/latest.log`** | 绕圈实车全量遥测（Agent 分析入口） |
| [`ring_track.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/ring_track.py) | 折线、exit_goal、场景（**offline 分支**） |
| [`docs/AVOIDANCE.md`](src/racing/racing_stage2_param_test/docs/AVOIDANCE.md) | 算法说明（**可能缺失**） |
| [`AGENTS.md`](AGENTS.md) | Agent 约束（含本文引用） |

---

## 7. 轨迹图（ring_plot）vs plan — 避免误判

**蓝线 plan（`full_ring_plan_polyline`）**

- 起点是 **enter_align 弯弧之后** 约 **(-0.12, 0.12)**，**不是 (0,0)**。
- 绿线 trajectory 从 **(0,0)** 起，含 `corridor_arrive_settle` + `rect_enter_align`。
- 图上一开头「错开」≠ 控制与 plan 用了两套公式；边长 offline 与 plot 均为 **1.10 / 0.50 / 2.80 m**（`launch_param_loader`）。

**名义终点 ≠ 入口 (0,0)**

- 整圈 **nominal_finish ≈ (0.48, 0.12)**（`rect_return_origin` 段末，底边 y≈0.12）。
- **(0,0)** 是通道/惯导 **进环入口**；跑完一圈不要求回到 (0,0)。

**绿线相对蓝线「整圈变大、变歪」— 真实现象**

| 原因 | 说明 |
|------|------|
| 开环 odom 积分 | 仿真无 map 闭环，turn/直行误差 **段间累积** |
| 避障绕开 | 竖边 handoff `\|lat\|` 仍 ~0.16～0.30 m，通道窄 → 图上看「超出 plan」 |
| 末段提前 finish | `finish_proximity` 0.40 m → 未贴底边即结束（§2.4 D1） |

**plan 几何本身**：`segment_endpoints_world` / `simulate_plan_world_poses` 与 `build_ring_plan` 一致；问题在 **控制 + 验收阈值**，不是 ring_track 算错。

---

## 8. 末段 finish 设计结论（世界坐标）

**不要用**：`progress` + `segment_lateral` + `|y - finish_y|` 三个门槛叠在一起（冗余、难调）。

**末段「到了没有」应直接用世界坐标**（`nominal_mission_finish_pose()`）：

```python
fx, fy = nominal_mission_finish_pose()
dist = hypot(x - fx, y - fy)
if dist <= 0.12:   # 或 0.15；勿再用 0.40
    finish_mission()
```

**Run D 教训**：`finish_proximity` 用世界坐标但 **阈值 0.40 m 过松** → x 接近时 dist≈0.35 m 仍 PASS，**y 差 0.34 m 也判到达**。

**可选一条门闩**（仍基于世界几何，非第三套 segment 变量）：

- 回程段世界端点 S→F，`along(S→F)` ≥ 段长×0.88 **且** `dist(F) ≤ 0.12 m`，防止 exit 绕障中途误 finish。

**与 offline 验收对齐**：`scenario_passes` 的 `dist_finish` 建议与 finish 逻辑同阈值（**≤0.15 m**），避免「数字 PASS、图不对」。

**保留**：段内 PD / 绕障仍用 P0+ψ；**仅末段 finish 判定**用世界点距离。

### Run F — 删除 `pre_corner_scenario_static_match` 捷径（2026-05-30）

| 项 | 值 |
|----|-----|
| 改动 | `avoidance_should_enter` 去掉 `pre_corner_scenario_static_match()` 段名即进；删除该函数 |
| 命令 | `auto_offline_test --group full` |
| PASS/FAIL | **4 / 15** |
| 对比 Run D baseline | `rect_return_50` **-0.280→-0.020 PASS**（回程段正常 AVOID_ENTER/EXIT，不再 side_2 误记 passed）；`side_2` 全程无 AVOID |
| 仍 FAIL | 多为 clearance 略负（-0.02～-0.18 m），无 stuck |
| PASS | `full_ring_no_obstacle`, `rect_return_50`, `rect_top_25`, `rect_top_90` |
| 日志 | `log/stage2_param_test/汇总/test_summary.txt` |

### Run G — Phase 6-8 直行台架 + 绕圈避障合并（2026-06-05 实车 · 上午）

| 项 | 值 |
|----|-----|
| 改动 | **P6**：Stage1 边转边避合并入 `DirectInertialTester`（父类）；**P7**：冷却机制 + cooldown；**P8a**：里程超限预计算 + 运行时终止；**P8b**：转角障碍检测 |
| 命令 | `ros2 launch racing_stage2_param_test direct_inertial_test.launch.py`（绕圈） |
| 结果 | 底边 1.10m 避障正常启动、完成 zigzag（TURN_AWAY→LEG1→TURN_BACK→LEG2→TURN_RECOVER→FINE_ALIGN）；冷却后避障不重复触发；段完成后正常过渡到转角。 |
| 日志 | 里程超限 `1.06/1.10m → 终止避障` 确认 P8a 生效 |
| 遗留 | 转角障碍检测待遇障验证；**下午发现里程/航向混源问题（→ Run H）** |

### Run H — 轮速位姿统一 + 会话日志（2026-06-05 实车 · 下午 · Agent 对话）

| 项 | 值 |
|----|-----|
| 现象 | ① 启动后有时 `along` 不涨；② 直行时 `ψ_imu` 相对 `ψ_wheel` 乱跳，车拧弯、`cross` 达 −35 cm；③ 转弯目标曾出现 **−113°**（IMU）而非 **~90°**（轮速） |
| 根因 | **IMU 与 `/odom` 航向零点差 ~157°**；计程/段锚用轮速、转弯/纠航用 IMU；EKF 偶发早到、(0,0) 占位 |
| 改动 | **P9a～P9d**（§2.1）：`navigation_pose_source: wheel`、`_sync_unified_pose_from_wheel`、`_unify_segment_pose`、`direct_inertial_test.yaml`、全量 `TELEM` 写 `log/direct_inertial_test/latest.log`；**leg2 统一 0.40 m** |
| 命令 | `colcon build --packages-select racing_stage2_param_test` + `direct_inertial_test.launch.py` |
| 结果 | **代码已合入，待重新场测**（修复前日志见 §1b.8） |
| 接续 | §1b.8 表 H1～H4；新 agent 先读 `latest.log` 中 `ODOM_ANCHOR` 与 `yaw`/`imu_off` |

---

*最后更新：2026-06-05 · Run H：轮速 `/odom` 位姿/航向/计程/控制完全统一 + 实机会话日志；leg2=0.40 m；详见 §1b.8、§2.1 P9*
