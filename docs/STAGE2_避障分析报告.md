# Stage2 参数测试 — 避障方案问题分析报告

## 一、已清理的孤儿文件

以下文件是根目录残留的旧版/备份，ROS2 不会执行它们，已删除：

| 文件 | 说明 | 状态 |
|------|------|------|
| `racing_stage2_param_test/direct_inertial_tester.py` | 根版旧代码（1062行，旧结构） | ✅ 已删 |
| `racing_stage2_param_test/avoid_controller.py` | 子包重复 | ✅ 已删 |
| `racing_stage2_param_test/avoid_geometry.py` | 子包重复（内容相同） | ✅ 已删 |
| `racing_stage2_param_test/avoid_controller.yaml` | 旧参数配置 | ✅ 已删 |

当前仅保留子包版（`racing_stage2_param_test/racing_stage2_param_test/`），setup.py entry_point 只会导入子包版。

---

## 二、P0 — Critical Bug：转角避障 obstacle_left 符号反了

### 位置
`racing_stage2_param_test/racing_stage2_param_test/direct_inertial_tester.py` 第 809 行

### 当前代码（错误）
```python
obstacle_left=(turn_angle < 0),
```

### 应改为
```python
obstacle_left=(turn_angle > 0),
```

### 推理

`avoid_geometry.py` 中：
```python
sign = -1.0 if obstacle_left else 1.0
psi1 = normalize_angle(psi0 + sign * offset_away_rad)
```

`obstacle_left=True` → `sign=-1` → `psi1 = psi0 - away` → **右转（ψ减小）**
`obstacle_left=False` → `sign=+1` → `psi1 = psi0 + away` → **左转（ψ增大）**

#### 顺时针场景（拐角 angle_deg = -75°，右转）

车头沿 −X 方向（ψ=180°），下一段向上走。拐角在右侧（场外）。

"从场内绕" = 先**向左偏** → 需要 `psi1 > psi0`（左转）

| 版本 | 公式 | sign | psi1 | 方向 | 判定 |
|------|------|------|------|------|------|
| `(turn_angle < 0)` | `(-75<0)=True` | -1 | psi0 - away | **右转→场外** | ❌ |
| `(turn_angle > 0)` | `(-75>0)=False` | +1 | psi0 + away | **左转→场内** | ✅ |

#### 逆时针场景（拐角 angle_deg = +78°，左转）

车头沿 +X 方向（ψ=0°），下一段向上走。拐角在左侧（场外）。

"从场内绕" = 先**向右偏** → 需要 `psi1 < psi0`（右转）

| 版本 | 公式 | sign | psi1 | 方向 | 判定 |
|------|------|------|------|------|------|
| `(turn_angle < 0)` | `(78<0)=False` | +1 | psi0 + away | **左转→场外** | ❌ |
| `(turn_angle > 0)` | `(78>0)=True` | -1 | psi0 - away | **右转→场内** | ✅ |

**结论：两个方向都反了。不改的话，转角避障会让车朝场外绕行。**

---

## 三、P0 — 两份代码分歧（已通过删孤儿解决）

执行版（子包版 `racing_stage2_param_test/`）和根版（已删）在避障逻辑上完全不同：

| 差异点 | 根版（旧） | 子包版（实际执行） |
|--------|-----------|-------------------|
| 段完成检查 | 避障前先判 | Scene B 内判 |
| 转角触发参数 | 独立 `corner_trigger_m` | 复用 `detour_obstacle_distance` |
| obstacle_left | `(turn_angle > 0)` ✅ | `(turn_angle < 0)` ❌ |
| 场景划分 | 统一流 | Scene A/B 分流 |

---

## 四、P0 — Scene A 逻辑缺陷（"直行靠近"死区 + 触发条件过宽）

### 位置
`direct_inertial_tester.py:run_move_segment()` 第 795-859 行

### 问题 1：obstacle_at_corner 触发过宽
**当前逻辑**（第 796-799 行）：
```python
obstacle_at_corner = (
    math.isfinite(self.front_obstacle_distance)
    and self.front_obstacle_distance < self.detour_obstacle_distance  # 0.55m
)
```

**缺陷**：
- 只要前方 0.55m 内有障碍就认为是"拐角障碍"
- **没有判断障碍是否真的在拐角附近**（如段末剩余距离）
- 导致长直道**中段障碍**也被误判为"拐角障碍"

**场景重现**：
```
rect_top: 2.90m 直道
│
0.0 ──── 0.50m ──────────────────── 2.35m ──── 2.90m
         ↑ 障碍出现                 ↑ 真正的拐角

remaining = 2.40m > 0.55m
→ obstacle_at_corner = True ❌（障碍不在拐角）
→ Scene A 接管
```

---

### 问题 2："直行靠近"分支屏蔽正常避障
**当前逻辑**（第 854-859 行）：
```python
# 还未到触发距离 → 直行靠近，不碰 avoider
angular = self._compute_move_lateral_angular()
linear = float(self.current_segment.get('speed', self.corridor_linear_speed))
self.cmd_pub.publish(self.create_twist(linear, angular))
self._maybe_log_telemetry('move')
return  # ⚠️ 直接返回，跳过 Scene B 正常避障
```

**后果**：
- Scene A 判定 `obstacle_at_corner=True` + `remaining >= 0.55m`
- 进入"直行靠近"分支 → **直接 return**
- **Scene B 的 `_avoider.step()` 被完全跳过**
- 机器人看着障碍物撞上去，正常避障永不触发

---

### 根本原因
**设计意图混乱**：
1. Scene A 想要处理"拐角处的障碍" → 应该只在**段末附近**触发
2. 但 `obstacle_at_corner` 只检查 `front_obstacle_distance < 0.55m`，不管 `remaining`
3. 导致整个直道只要下一段是转弯，前方有障碍就进 Scene A
4. `remaining >= 0.55m` 时走"直行靠近" → 屏蔽正常避障

---

## 五、P1 — 避障模式选择逻辑缺失

### 问题
当前没有明确的"选择用哪种避障"的逻辑：
- **正常避障**（边转边避，三段式）：适用于直道中段，空间充足
- **转角避障**（单腿斜边，跳段）：适用于段末空间不足，需直接绕过拐角

**当前实现**：
- 第 798 行：`front_obstacle_distance < 0.55` → 触发 obstacle_at_corner
- 第 804 行：`remaining < 0.55` → 启动转角避障
- **两个 0.55m 含义完全不同，但用的是同一个参数**

### 期望逻辑
```
前方检测到障碍（< 0.55m）
    ↓
判断：段末剩余路程（remaining）够不够绕？
    ├─ remaining > X → 正常避障（空间充足）
    └─ remaining ≤ X → 转角避障（空间不足，直接跳段）
```

### 缺失参数
**需要新增**：`corner_trigger_remaining_m`（段末剩余距离阈值）
- 含义：绕完正常避障后还需要多少安全余量
- 建议初值：**0.60m**（正常避障投影距离约 0.4-0.5m + 0.1m 余量）
- **实测调整**：根据场测调优

---

## 六、P0 — 转角避障斜边过长导致段越界风险

### 问题
**当前配置**：`corner_leg_distance_m: 0.778`（config/avoid_controller.yaml:38）

**风险场景**：
```
rect_side_1: 0.385m 或 rect_side_2: 0.36m 短边段
│
├─ 0.0m ──────── remaining=0.30m ──── 0.36m (段末)
│                 ↑ 触发转角避障
│                 斜边 0.778m → 走出段边界 ❌
```

**后果**：
- 部分短边段（0.36m / 0.385m）长度不足 0.778m
- 转角避障走斜边 → **越过段末边界** → 走出赛道或撞墙
- 当前代码转角模式**无段边界检查**（里程检查被跳过）

### 修复方案
降低斜边长度至 **0.40m**，确保所有段都能容纳转角避障路径。

---

## 七、P2 — avoid_controller.yaml 参数与代码默认值不同步

### 生效 yaml（config/avoid_controller.yaml）与子包版代码默认值对比

| 参数 | 代码默认值（avoid_controller.py:64-66） | yaml 配置值 |
|------|---------------------------------------|------------|
| corner_turn_away_deg | 60 | 45 |
| corner_turn_back_deg | 30 | 45 |
| corner_leg_distance_m | 0.354 | 0.778 |

运行时 yaml 覆盖代码默认值，所以实际跑的是 yaml 的值（45/45/0.778）。但如果测试时不用 yaml 就会跑错值。

---

## 七、P2 — avoid_controller.yaml 参数与代码默认值不同步

### 生效 yaml（config/avoid_controller.yaml）与子包版代码默认值对比

| 参数 | 代码默认值（avoid_controller.py:64-66） | yaml 配置值 | 差异 |
|------|---------------------------------------|------------|------|
| avoid_turn_away_deg | 30 | 40 | ✅ yaml 覆盖 |
| avoid_turn_back_deg | 40 | 50 | ✅ yaml 覆盖 |
| avoid_recover_deg | 40 | 15 | ✅ yaml 覆盖 |
| avoid_leg1_distance_m | 0.05 | 0.33 | ✅ yaml 覆盖 |
| avoid_leg2_distance_m | 0.10 | 0.44 | ✅ yaml 覆盖 |
| corner_turn_away_deg | 60 | 45 | ⚠️ 相差 15° |
| corner_turn_back_deg | 30 | 45 | ⚠️ 相差 15° |
| corner_leg_distance_m | 0.354 | 0.778 | ⚠️ 相差 2.2 倍 |

**问题**：
- 运行时 yaml 覆盖代码默认值（正常）
- 但如果 yaml 加载失败或参数缺失，会回退到**完全不同的**默认值
- 增加调试混淆风险

### 修复方案
统一代码默认值与 yaml 配置值，保持一致性。

---

## 八、P2 — detour_heading_gate 可能锁死避障

### 位置
`avoid_controller.py:_should_trigger()` 第 319-321 行

```python
if abs(heading_error) > detour_heading_gate_rad:
    return False  # 不触发
```

### 问题
`detour_heading_gate_deg=12°`。若机器人因打滑/地面不平偏航角 >12°，避障**永久被抑制**直到到达段末触发段切换。

### 建议
放宽至 25° 或移除 gate 条件。

---

## 八、P2 — AvoidController 无超时保护

### 问题
`AvoidController` 的 FSM 没有全局超时。若地面打滑导致 `_turn_toward()` 永远达不到 `heading_tolerance`，FSM 永久卡在某个状态，无法退出。

### 建议
给 `AvoidController.step()` 加全局 timeout 检查，超过时限自动 reset。

---

## 九、P2 — AvoidController 无超时保护

### 位置
`avoid_controller.py` 整个 FSM（状态机）

### 问题
`AvoidController` 的状态机没有全局超时检查。若地面打滑导致 `_turn_toward()` 永远达不到 `heading_tolerance`（1.5°），FSM 永久卡在某个状态，无法退出。

**场景**：
```
step() 循环：
  if self._state == 'turn_away':
      if self._turn_toward(plan.psi1, yaw):  # 若打滑永远达不到 1.5°
          # 永远不进这里
  → 机器人原地打转，永不退出避障 ❌
```

### 后果
- 地面湿滑/打滑 → 转不到目标航向 → 无限循环
- 无超时保护 → 任务卡死

### 修复方案
增加全局超时检查，避障超过 **8 秒**强制 reset，防止无限循环。

---

## 十、P3 — 正常避障的里程中断逻辑（已确认保留投影距离）

### 位置
`avoid_controller.py:step()` 第 427-439 行

### 当前逻辑
避障运行时用 `projected_distance >= segment_distance - tol` 判断是否该退出避障。

### 分析
**潜在问题**：
- 避障走蛇形路径，实际行驶距离 > 投影距离
- 理论上可能提前退出 → 立即重新触发 → 振荡

**实际缓解**：
- 当前有 `detour_cooldown_sec=3.0` 冷却时间
- 避障完成后 3 秒内不会重新触发

### 结论
**保留投影距离**判断（符合惯导整体设计），依赖冷却时间防止振荡。如场测发现振荡，可增大冷却时间或增加"避障完成后强制直行"逻辑。

---

## 十一、P3 — _effective_left_m() 投影精度

### 位置
`avoid_controller.py:197-207`

### 问题
侧方测距取 65°±15° 扇形内最近点，用 `d * sin(angle)` 换算横向距离。但 `side_detour_threshold_m=0.18m` 对比的是单一射线点，可能出现扇形内某点很近但整体空间其实够用的情况 → 误触发。

## 修复执行记录

### 执行日期
2026年7月6日

### 已完成修复

| Phase | 任务 | 文件 | 修改内容 | 状态 |
|-------|------|------|---------|------|
| **1.1** | 修复 obstacle_left 符号 | `direct_inertial_tester.py:823` | `(turn_angle < 0)` → `(turn_angle > 0)` | ✅ 完成 |
| **1.2** | 重构 Scene A 逻辑 | `direct_inertial_tester.py:798-869` | 删除"直行靠近"分支，改为 `remaining <= corner_trigger_remaining_m` 判断 | ✅ 完成 |
| **1.2** | 新增参数 | `direct_inertial_tester.py:37`<br>`avoid_controller.yaml` | 声明并配置 `corner_trigger_remaining_m: 0.60` | ✅ 完成 |
| **1.3** | 降低转角斜边长度 | `avoid_controller.yaml:45` | `corner_leg_distance_m: 0.778` → `0.40` | ✅ 完成 |
| **2.1** | 增加超时保护 | `avoid_controller.py:114,172,388,413` | 新增 `max_avoid_duration_sec=8.0` + 四处超时检查 | ✅ 完成 |
| **2.2** | 放宽 heading_gate | `avoid_controller.yaml:38`<br>`direct_inertial_tester.py:36,81` | 新增 `detour_heading_gate_deg: 25.0` 配置与传递 | ✅ 完成 |
| **2.3** | 同步代码默认值 | `avoid_controller.py:41-67` | 所有参数默认值改为与 yaml 一致 | ✅ 完成 |

### 关键修改说明

#### 1. obstacle_left 符号修复
```python
# 修改前（错误）：
obstacle_left=(turn_angle < 0)  # 所有方向都反了

# 修改后（正确）：
obstacle_left=(turn_angle > 0)  # turn_angle > 0 (左转) → 拐角在左 → 从右绕
                                 # turn_angle < 0 (右转) → 拐角在右 → 从左绕
```

#### 2. Scene A 逻辑重构
```python
# 新逻辑：
if obstacle_at_corner:
    if remaining <= self.corner_trigger_remaining_m:  # 0.60m
        # 路不够 → 转角避障
    # else: 路够 → 走 Scene B 正常避障

# 删除了原来的"直行靠近"分支（854-867行），不再屏蔽正常避障
```

#### 3. 新增参数
- **`corner_trigger_remaining_m: 0.60`**（段末剩余距离阈值）
  - 含义：绕完正常避障后需要的安全余量
  - 实测调整：太大→转角避障过早；太小→可能越界

#### 4. 超时保护机制
```python
# 避障超过 8 秒强制 reset，防止打滑导致 FSM 卡死
if elapsed > self.max_avoid_duration_sec:
    self._log_detour('避障超时 → 强制 reset')
    self.reset()
```

---

## 修复方案汇总（更新）

### Phase 1：紧急修复（阻塞测试，必须立即执行）

#### 1.1 修复 obstacle_left 符号（P0-1）
**文件**：`direct_inertial_tester.py:809`

```python
# 修改前
obstacle_left=(turn_angle < 0),

# 修改后
obstacle_left=(turn_angle > 0),
```

**耗时**：5 分钟

---

#### 1.2 重构 Scene A 逻辑（P0-2 + P1）
**文件**：`direct_inertial_tester.py:795-859`

**新增参数**：
```yaml
# config/avoid_controller.yaml 新增
corner_trigger_remaining_m: 0.60  # 段末剩余距离阈值（实测调整）
```

```python
# direct_inertial_tester.py 约 40 行新增声明
self.declare_parameter('corner_trigger_remaining_m', 0.60)
self.corner_trigger_remaining_m = float(
    self.get_parameter('corner_trigger_remaining_m').value
)
```

**逻辑修改**：
```python
# Scene A 新逻辑（795-859 行重构）
if next_is_turn:
    obstacle_at_corner = (
        math.isfinite(self.front_obstacle_distance)
        and self.front_obstacle_distance < self.detour_obstacle_distance  # 0.55m
    )
    
    if obstacle_at_corner:
        # 关键判断：绕完后路够不够
        if remaining <= self.corner_trigger_remaining_m:  # 新参数 0.60m
            # 路不够 → 转角避障
            if not self._avoider.is_active:
                turn_angle = float(self.plan[self.plan_index + 1]['angle_deg'])
                self._avoider.start_corner_avoid(
                    psi0_rad=self.segment_heading,
                    obstacle_left=(turn_angle > 0),  # ✅ 修复符号
                    corner_away_deg=float(self.get_parameter('corner_turn_away_deg').value),
                    corner_back_deg=float(self.get_parameter('corner_turn_back_deg').value),
                    corner_leg_m=float(self.get_parameter('corner_leg_distance_m').value),
                )
                self._log_session('CORNER_AVOID', ...)
            
            if self._avoider.is_active:
                nav = NavState(...)
                if self._avoider.step(nav):
                    if self._avoider.corner_mode_completed:
                        # 跳段逻辑（保持不变）
                        ...
                    return
        # else: remaining > 0.60 → 路够，不拦截，走 Scene B 正常避障

# Scene B：正常避障（保持不变，866-885 行）
if not self._avoider.corner_mode_active:
    if progress >= target_distance - self.distance_tolerance:
        # 段完成
        ...
if self._avoider.step(nav):
    return
```

**删除**：第 854-859 行"直行靠近"分支

**耗时**：30 分钟

---

#### 1.3 降低转角避障斜边长度（P0-3）
**文件**：`config/avoid_controller.yaml:38`

```yaml
# 修改前
corner_leg_distance_m: 0.778

# 修改后
corner_leg_distance_m: 0.40  # 降低以防越界
```

**耗时**：5 分钟

---

### Phase 2：稳定性增强（建议尽快执行）

#### 2.1 增加避障超时保护（P2-3）
**文件**：`avoid_controller.py`

**修改位置**：
1. `__init__`（约 97-102 行）
2. `_start()`（约 339 行）
3. `step()`（约 403 行开头）

**新增代码**：
```python
# 1. __init__ 新增
class AvoidController:
    def __init__(self, ...):
        # ... 原有变量 ...
        self._avoid_start_time = 0.0
        self.max_avoid_duration_sec = 8.0  # 超时阈值

# 2. _start() 记录启动时间
def _start(self, nav: NavState):
    # ... 原逻辑 ...
    self._avoid_start_time = self._now_sec()

# 3. step() 开头增加超时检查
def step(self, nav: NavState) -> bool:
    # 全局超时检查
    if self.is_active:
        elapsed = self._now_sec() - self._avoid_start_time
        if elapsed > self.max_avoid_duration_sec:
            self._log_detour(f'避障超时 {elapsed:.1f}s → 强制 reset')
            self.reset()
            return False
    
    # ... 原逻辑 ...
```

**耗时**：20 分钟

---

#### 2.2 放宽 heading_gate 限制（P2-2）
**文件**：`config/avoid_controller.yaml`

**新增配置**：
```yaml
# avoid_controller.yaml 新增（当前用代码默认值 12.0）
detour_heading_gate_deg: 25.0  # 从 12° 放宽到 25°
```

**代码修改**（如果 yaml 不生效）：
```python
# avoid_controller.py:42-66
class AvoidConfig:
    def __init__(
        self,
        # ...
        detour_heading_gate_deg=25.0,  # 从 12.0 改为 25.0
    ):
```

**耗时**：5 分钟

---

#### 2.3 同步代码默认值与 yaml（P2-1）
**文件**：`avoid_controller.py:42-66`

**修改**：
```python
class AvoidConfig:
    def __init__(
        self,
        # 正常避障参数（改为与 yaml 一致）
        avoid_turn_away_deg=40.0,      # 从 30 改为 40
        avoid_turn_back_deg=50.0,      # 从 40 改为 50
        avoid_recover_deg=15.0,        # 从 40 改为 15
        avoid_leg1_distance_m=0.33,    # 从 0.05 改为 0.33
        avoid_leg2_distance_m=0.44,    # 从 0.10 改为 0.44
        
        # 转角避障参数
        corner_turn_away_deg=45.0,     # 从 60 改为 45
        corner_turn_back_deg=45.0,     # 从 30 改为 45
        corner_leg_distance_m=0.40,    # 从 0.354 改为 0.40
    ):
```

**耗时**：10 分钟

---

### 执行顺序

```
Step 1: Phase 1 修复（必须，约 40 分钟）
  ├─ 1.1 obstacle_left 符号
  ├─ 1.2 Scene A 逻辑重构
  └─ 1.3 corner_leg 降至 0.40

Step 2: 编译验证
  └─ scp + ssh 远端编译

Step 3: Phase 2 修复（建议，约 35 分钟）
  ├─ 2.1 超时保护
  ├─ 2.2 heading_gate 放宽
  └─ 2.3 同步默认值
```

---

## 需要实测调整的参数

| 参数 | 初值 | 说明 | 调整方向 |
|------|------|------|---------|
| **corner_trigger_remaining_m** | **0.60** | 段末剩余多少米时认为"路不够" | 太大→转角避障触发过早<br>太小→容易越界 |
| corner_leg_distance_m | 0.40 | 转角避障斜边长度 | 太大→越界<br>太小→绕不过 |
| detour_heading_gate_deg | 25.0 | 偏航超过此值不触发避障 | 太小→打滑锁死<br>太大→歪着也避障 |

**重点调试**：`corner_trigger_remaining_m` 决定避障模式选择逻辑的关键阈值。

---

## 优先级汇总（更新）

| 优先级 | 问题编号 | 问题 | 影响 | 修复状态 |
|--------|---------|------|------|---------|
| **P0** | P0-1 | obstacle_left 符号反 | 转角避障必撞墙 | 待修复（Phase 1.1） |
| **P0** | P0-2 | Scene A "直行靠近"屏蔽正常避障 | 长直道障碍不避，撞车 | 待修复（Phase 1.2） |
| **P0** | P0-3 | 转角避障斜边过长（0.778m） | 短边段越界 | 待修复（Phase 1.3） |
| **P0** | - | 两份代码分歧 | - | ✅ 已解决（删孤儿）|
| **P1** | P1-1 | 避障模式选择逻辑缺失 | 无法区分正常/转角避障 | 待修复（Phase 1.2） |
| **P2** | P2-1 | yaml 与代码默认值不同步 | 参数回退风险 | 待修复（Phase 2.3） |
| **P2** | P2-2 | heading_gate 过严（12°） | 偏航时锁死避障 | 待修复（Phase 2.2） |
| **P2** | P2-3 | 无超时保护 | FSM 打滑卡死 | 待修复（Phase 2.1） |
| **P3** | P3-1 | 里程判定用投影距离 | 潜在振荡风险 | ✅ 保留设计（冷却缓解）|
| **P3** | P3-2 | 侧方投影精度 | 误触发 | 暂不修改 |