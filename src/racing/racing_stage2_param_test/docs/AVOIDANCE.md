# Stage2 参数测试 — 避障

导航总模型见 **[NAVIGATION.md](NAVIGATION.md)**（世界坐标：当前位姿 + 目标点）。

## 状态（简化）

| 变量 | 含义 |
|------|------|
| `current_position` | 当前 odom `(x,y)` |
| `goal_*_xy` | 当前阶段要去的世界点 |
| `goal_direct_phase` | `bypass` → `pass` → `exit` / `rejoin` / `next_leg` |
| `segment_plan_start_xy` / `segment_plan_end_xy` | 本段名义 **S / E**（蓝线端点） |

**不再**以「进段实测 P0」作为主进度尺；沿程 `projected_distance()` = 在名义 S→ψ 上的投影并 **clamp 到段长**。

## 相位与目标点

| 相位 | 目标 | 控制 |
|------|------|------|
| bypass | 障碍沿程 + 侧向 offset | PP 朝路点；**段首/弯后 + 障碍≤50%** → `ENTRY_DIRECT` 直朝 bypass，极近则 skip 到 pass |
| pass | 障碍沿程 + pass_margin | PP；横向衰减 |
| rejoin | 长段：沿程略前 | PP |
| exit | 段末 **E** 附近（`segment_end_goal_world`） | 短边 **贴 plan PD** |
| handoff | — | 见下 |

**内绕**：`CLOCKWISE_INWARD_BYPASS_SIDE`；短边优先 preferred side。

### exit（direct_cut）

- **距 plan 终点 E > 0.12 m**：Pure Pursuit 朝 **名义段末 E**（`segment_plan_end_xy`），与 bypass/pass 同一套「去一个点」。
- **近 E 后**：贴段 PD 收横偏/航向（`goal_direct_exit_segment_cmd`）。

### handoff（exit / direct_cut）

`goal_direct_handoff_ready()`：

- 已过障
- `projected_distance` 达段末 **或** `distance_to_segment_plan_end_m ≤ 0.12 m`
- `|lat| ≤ exit_handoff_lat_limit_m()`（≤ 0.10 m）
- `clear_streak ≥ 1`

超时 `avoidance_stuck_handoff_ready()` 仍要求 exit 时横偏在 limit 内。

## 末段整圈完成

- `dist(robot, nominal_finish) ≤ 0.15 m`（`FINISH_WORLD_DIST_M`）
- 回程段 `mission_return_finish_along_ok()` 防 exit 中途误 finish
- `finish_approach`：**沿回程段航向**收横偏，不横插名义终点

## 离线仿真

```bash
python3 -m racing_stage2_param_test.offline_ring_test --scenario rect_side_2_50
python3 -m racing_stage2_param_test.auto_offline_test --group abnormal
```

## 场景锥桶

`{段}_{比例%}` → `expected_point_on_world_segment`（名义折线，非实测弦线）。
