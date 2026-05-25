# Stage2 避障（goal_direct）方案记录

> 工作区：`dev_ws` · 包：`racing_stage2_param_test` · 算法：`goal_direct`（2026-05）  
> 本文档记录 **已尝试 / 未尝试** 的修复方向，便于后续迭代。  
> **明确不做**：单步 DWA 换 (v,ω)（改动面大，当前不采纳）。

---

## 1. 当前在用什么（baseline）

| 项 | 说明 |
|----|------|
| 策略名 | `goal_direct`（`detour_strategy=goal_direct`） |
| 控制 | 2～4 个世界坐标路点 + 朝当前点走（Pure Pursuit 风格） |
| FSM | `bypass → pass → (rejoin \| exit) → handoff` |
| 几何 | 实测弦线 `DRIVEN_CW_SEGMENT_ENDPOINTS` + `plan_avoidance_goals()` |
| 斜切 | `need_direct_cut` 为真时跳过 rejoin，直接 `exit_goal`（段末向内缩边） |
| 绕边 | 一次锁定，无 `BYPASS_FLIP` |
| 安全 | 保留 `motion_safe` / `dwa_clearance_along_motion` **仅作碰撞检查**，不作主控制律 |

**Git 里程碑**

| Commit | 内容 |
|--------|------|
| `94eb951` | 弦线几何 + corridor 选边（goal_direct 前） |
| `2389e9c` | **corridor_track → goal_direct** 主替换 |
| `5b11d66` | 短边 rolling bypass、略加宽 offset、creep 微调 |

**离线验收门槛**：`min_clearance_m > -0.02` 且 `mission_finished=True`

**最新 standard 组（`5b11d66` 后）**

| 场景 | finished | clearance |
|------|----------|-----------|
| full_ring_no_obstacle | yes | 0.000 m |
| rect_top_50 / rect_return_50 / rect_first_leg_50 | yes | ~ **-0.07 m** |
| rect_side_2_50 | yes | ~ **-0.115 m**（旧 corridor ~-0.27 m） |
| rect_side_1_50 | yes | ~ -0.117 m |
| rect_first_leg_75 | yes | ~ **-0.30 m** |

**根因（仍未达标处）**：bypass 段 **横向未充分让开就沿程靠近锥桶**；`rect_side_2_50` 段入口 nearest≈0.11 m 即进避障。

---

## 2. 已尝试方案

### 2.1 架构级（已完成）

| 方案 | 结果 | 备注 |
|------|------|------|
| **corridor_track**（mirror `lat(s)` + flip + 短边特例） | 弃用 | 短边/段末 S 弯不自然；`rect_side_2_50` clearance≈-0.27 m |
| **goal_direct 路点 + FSM** | 采用 | 见 `2389e9c` |
| **need_direct_cut + exit_goal 场内缩边** | 采用 | 修复 exit 曾指到外栏（y≈-1.93）的符号错误 |
| **preferred 绕边（正中锥桶）** | 采用 | 用静态场景坐标判 cross，不再误选外场侧 |
| **删除 maybe_flip / rect_side_2 硬编码** | 完成 | 通用规则 |
| **driven 弦线几何** | 保留 | `ring_track.py` |

### 2.2 goal_direct 调参 / 小改（已试）

| 方案 | 结果 | 备注 |
|------|------|------|
| **rolling bypass**（短边/斜切：沿程略前 + 全横偏） | 部分有效 | `rect_side_2_50` -0.13→~-0.115 m；仅短边启用 |
| **bypass 锚点在锥桶前**（`driven_s - lead`） | 部分有效 | 与 rolling 配合 |
| **略加宽短边 offset**（0.32→0.33，cap 0.35） | 略好 | 仍不足 -0.02 |
| **`avoid_pass_clearance_m` 0.12**（loader） | 略好 | 与 offset 联动 |
| **before_abreast 限速 + nearest<0.18 creep** | 略好 | 见 `goal_direct_cmd` |
| **lat_build 强掐 linear（<0.8 时 v 大减 / v≈0）** | **失败** | clearance 恶化至 ~-0.25 m 或段超时卡死 |
| **dynamic bypass 大步前移 anchor** | **失败** | 曾刮桶加重 |
| **corridor_sidestep ω=10×lat_err 混合** | **失败** | 与朝点控制打架 |
| **stuck_handoff 在 pass 阶段提前退出** | **失败** | 已改为仅 exit/rejoin 可 stuck handoff |

### 2.3 明确否决（历史，勿回退）

| 方案 | 原因 |
|------|------|
| 圆心距离圈规划 | 与用户方向不符 |
| 世界坐标 chase / 无约束追点 | 曾出现 x≈-170 失控 |
| `rect_side_2` 专用分支 | 用户要求通用能力 |
| 完整 **DWA 主控制**（每拍选 v,ω 替代 goal_direct） | **改动大**；见 §4 |

---

## 3. 未尝试方案（推荐优先级）

### P0 — 改动小，针对 clearance

| # | 方案 | 做法概要 | 预期 |
|---|------|----------|------|
| 1 | **入障前 approach 限速** | 加强 `mission_obstacle_linear_cap_mps`；短边略增 `avoid_watch_distance` | 段入口不贴桶进 bypass |
| 2 | **真·先横后前 bypass 子状态** | `lat_build < 0.75` 时 **v≈0** + rolling 路点 **几乎纯侧向**（非仅把 v×0.4） | 直击根因；需与方案 1 联调 |
| 3 | **段初预转向** | 转弯末/刚进短直段且已见桶 → 名义直行带小横偏或更早 AVOID_ENTER | 减少「一进段就 0.11 m」 |

### P1 — 调参 / 判定

| # | 方案 | 做法概要 | 风险 |
|---|------|----------|------|
| 4 | 短边 `avoid_bypass_max_lateral_m` 0.36 | 仅 cut 段 | 蹭外栏 |
| 5 | **放宽 need_direct_cut**（段末 75%） | `s_obs/seg_len>0.65` 或更大 half_span 阈值 → 强制 exit 不 rejoin | 修 `rect_first_leg_75` |
| 6 | `exit_goal` / `pass_goal` 再向内缩 | 短边 margin +0.02～0.03 | 弯点对准 |

### P2 — 赛后 / 范围外

| # | 方案 | 说明 |
|---|------|------|
| 7 | 移植 official `stage2_inertial_navigator.py` | param_test 通过后；环尺寸 3.42×1.08 m 重测弦线 |
| 8 | `competition_total.launch.py` 实车复测 | 非 `direct_inertial_test` |

---

## 4. 不做：DWA 主控制

plan 中曾提 **「单步 DWA 对 active_goal 打分选 (v,ω)」** 作为可选增强。

**当前决定：不做。**

| 理由 |
|------|
| 需新增 cost 权重、候选 (v,ω) 网格、与 FSM 切换逻辑，**改动面接近换一套局部规划器** |
| 现有 `motion_safe` 已用 `dwa_clearance_along_motion` 做安全钳制，够用 |
| 优先把 **goal_direct + 先横后前 + approach 限速** 跑通，再考虑是否值得上 DWA |

若将来评估 DWA，应单独开分支、单独文档，**不与本节 P0 混做**。

---

## 5. 建议下一步（执行顺序）

1. **P0-1 + P0-2**：只盯 `rect_side_2_50`，目标 clearance > -0.02 m  
2. **P1-5**：修 `rect_first_leg_75`  
3. 跑 `auto_offline_test --group standard` 更新 `log/stage2_param_test/汇总/`  
4. 达标后再 **P2 移植**

**验证命令**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racing_stage2_param_test
python3 -m racing_stage2_param_test.offline_ring_test --scenario rect_side_2_50
python3 -m racing_stage2_param_test.auto_offline_test --group standard
```

**日志**

- 汇总：`log/stage2_param_test/汇总/test_summary.txt`
- 单场景：`log/stage2_param_test/右边_50%/debug.log`（搜 `AVOID_ENTER` / `need_direct_cut`）

---

## 6. 相关文件

| 文件 | 作用 |
|------|------|
| `src/racing/racing_stage2_param_test/.../direct_inertial_tester_avoidance.py` | goal_direct 主逻辑 |
| `src/racing/racing_stage2_param_test/.../ring_track.py` | 弦线、exit_goal |
| `src/racing/racing_stage2_param_test/docs/AVOIDANCE.md` | 算法说明（用户向） |
| `src/racing/racing_stage2_param_test/.../launch_param_loader.py` | 离线/launch 参数 |

---

*最后更新：2026-05-24 · 对应 git：`5b11d66`*
