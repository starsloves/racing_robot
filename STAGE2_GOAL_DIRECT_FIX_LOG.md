# Stage2 避障（goal_direct）方案与测试记录

> 工作区：`dev_ws` · 包：`racing_stage2_param_test` · 算法：`goal_direct`  
> **用途：** 记录每条修复/调参是否 **已试、结果、是否保留**；新 agent **必须先查本文**，避免重复踩坑。  
> **明确不做**：单步 DWA 换 (v,ω) 作主控制；恢复 `next_leg → rect_return_origin` 斜切；长段静态身后 rejoin。

---

## 0. 使用说明（给 Agent）

1. 提出新改法前，在本文 **§2 已试** 与 **§3 未试** 中检索关键词（如 `next_leg`、`rejoin`、`exit`、`内绕`）。
2. 若 **已试且失败**，不要原样重试；除非写明与上次差异。
3. 每次 offline full 或专项跑完后，在 **§4 测试记录** 追加一行/一节，并更新 **§1 当前 baseline**。
4. 汇总文件：`log/stage2_param_test/汇总/test_summary.txt`  
5. 算法用户向说明：`src/racing/racing_stage2_param_test/docs/AVOIDANCE.md`

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

### 3.0 曾讨论、不建议的执行顺序（备忘）

| 顺序 | 内容 |
|------|------|
| 推荐 | **E1+E2**（finish）→ **E3**（竖边 lat）→ full → 不够再 E4/E6 |
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
| [`direct_inertial_tester_avoidance.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/direct_inertial_tester_avoidance.py) | goal_direct FSM、handoff |
| [`direct_inertial_tester.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/direct_inertial_tester.py) | 段末 trim、corner_shortcut |
| [`ring_track.py`](src/racing/racing_stage2_param_test/racing_stage2_param_test/ring_track.py) | 折线、exit_goal、场景 |
| [`docs/AVOIDANCE.md`](src/racing/racing_stage2_param_test/docs/AVOIDANCE.md) | 算法说明 |
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

---

*最后更新：2026-05-25 · 当前 full baseline：11 PASS / 9 FAIL（Run D）· finish_proximity 待 Run E 替换*
