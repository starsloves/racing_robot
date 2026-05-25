# goal_direct 段内避障（目标点斜切）

**2～4 个世界坐标路点 + Pure Pursuit 式朝点走**，不用 `lat_target(s)` 镜像曲线。

## 目标点序列

入障时 `plan_avoidance_goals()` 一次性锁定：

| 目标 | 含义 |
|------|------|
| `bypass_goal` | 锥桶侧面，沿弦线法向 `bypass_side × offset` |
| `pass_goal` | 沿弦线越过 `s_obs + r + pass_margin`，仍带部分横偏 |
| `rejoin_goal` | 段内够回正时，弦线上横偏 0 的回正点 |
| `exit_goal` | 斜切：段末向场内缩 `exit_inward_margin` |

**斜切判定**（`need_direct_cut`）：

```
need_direct_cut = (s_obs + half_span > s_end - tol) or (seg_len < cut_segment_len_m)
```

为真 → 序列 `bypass → pass → exit`（跳过 rejoin）；否则 `bypass → pass → rejoin`。

## FSM

```
bypass → pass → rejoin | exit → handoff
```

- 到点容差：`avoid_goal_reach_tol_m`（默认 ~0.07 m）
- 过桶：`robot_passed_locked_obstacle()`（沿程 + 激光身后）
- handoff：到达末目标且过桶；斜切时注入 `_corner_shortcut_turn_target` 并 `start_segment(turn)`

## 控制（`goal_direct_cmd`）

```
heading_cmd = atan2(gy - y, gx - x)
ω = kp * angle_error(heading_cmd, yaw)
v = v_max * dist_scale * curve_scale * clearance_gate
```

近目标 / 大航向误差自动限速；`motion_safe` / DWA 点仿真作安全钳制。

## 绕边（一次锁定，不 flip）

1. 锥桶相对弦线横偏明显 → 走对侧
2. 正中 → 比较左右 `effective_side_clearance_m`
3. `preferred`（`CLOCKWISE_INWARD_BYPASS_SIDE`）仅 tie-break

## 参数（launch / yaml）

| 参数 | 默认 | 说明 |
|------|------|------|
| `avoid_goal_bypass_offset_m` | 0 → 用 `avoid_bypass_max_lateral_m` | 绕开横距 |
| `avoid_goal_pass_margin_m` | 0.12 | 过桶沿程余量 |
| `avoid_goal_cut_segment_len_m` | 0.68 | 短段强制斜切 |
| `avoid_goal_reach_tol_m` | 0.07 | 到点容差 |
| `avoid_goal_heading_kp` | 0 → 用 `heading_kp` | 朝点角增益 |
| `avoid_goal_exit_inward_margin_m` | 0.17 | 段末向内缩边 |

日志标签：`detour_strategy=goal_direct`，`AVOID_ENTER` 打印 `need_direct_cut` 与各路点。

## 离线场景矩阵

命名 `{段}_{比例%}`，障碍落在 `DRIVEN_CW_SEGMENT_ENDPOINTS` 实测直行线上。

批量组：`smoke` / `standard` / `per_segment_50` / `corner_stress` / `full`。

验收：整圈 `mission_finished`，`min_clearance > -0.02`，终点距原点 ≤ 0.40 m（有障）/ 0.25 m（无障）。

```bash
python3 -m racing_stage2_param_test.offline_ring_test --scenario rect_side_2_50
python3 -m racing_stage2_param_test.auto_offline_test --group standard
```

## 比赛移植清单（param_test 通过后）

| 项 | 说明 |
|----|------|
| 代码落点 | 移植 `plan_avoidance_goals` + FSM + `goal_direct_cmd` 到 official `stage2_inertial_navigator.py` |
| 实车 launch | `ros2 launch racing_bringup competition_total.launch.py`（非 `direct_inertial_test`） |
| 环尺寸 | 官方 `loop_long_length_m`≈3.42 m、`loop_short_length_m`≈1.08 m；重测弦线端点，勿照搬 param_test 0.50 m 竖边 |
| 参数 | 对齐 `inertial_stage2.yaml`；斜切 `exit_goal` 须缩边防蹭外栏 |
| 状态机 | 尊重 `allow_detour`、`detour_cooldown_sec`、turn 段不进避障 |
| 感知 | 保留 `circle_is_side_fence` / 远墙过滤；顺/逆时针各测一轮 |
| 速度 | 官方 `ring_linear_speed` 0.24；段末斜切须限速 |
| 验收 | 离线 clearance + 实车 `stage2_state` / Stage3 起点 |
