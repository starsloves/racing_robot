# Stage2 参数测试 — 导航模型（世界坐标）

## 原则（2026-05 重构）

全程 **一套 map / odom 世界坐标** `(x, y, yaw)`，不另建坐标系。

控制器只维护两类量：

| 量 | 含义 |
|----|------|
| **当前位姿** | `/odom_combined`（离线为仿真积分） |
| **当前目标点** | 避障阶段：bypass / pass / exit 等世界 `(x,y)`；无障直行：本段名义终点 **E** |

直行段的几何来自 **`config/field_track_*.yaml`**（每段 S、E、ψ，**map 世界坐标**；入口 **(2.50, 3.20)** 与官方 `corridor_goal` 一致）：

- **S** = 段起点，**E** = 段终点，**ψ** = 段航向  
- **沿程** = 当前位姿在 S→ψ 上的投影，**夹在 [0, 段长]**  
- **横偏** = 到 plan 弦线的法向距离  

实现：`world_plan_nav.py`（`DirectInertialTesterWorldPlanMixin`）。

## 第二阶段完整段序（顺时针）

**前提**：Stage1 通道导航已结束，车在 **map (2.50, 3.20)，ψ=90°**。  
参数测试**不执行** `pre_loop_plan`（无 `scan_leave` / `corridor_staging`）。

```
Stage1 结束 @ (2.50, 3.20), ψ=90°
        │
        ▼
 0  corridor_arrive_settle     pause 0.20 s
 2  rect_enter_align           turn +90°  → ψ=180°（Stage1 末 90° + 入口转）
 3  rect_first_leg             move 1.10 m   S(2.38,3.32) → E(1.28,3.32)  ψ=180°（−X）
 3  rect_corner_1              turn −90°
 4  rect_side_1               move 0.80 m   → E(3.70,4.67)  ψ=0°
 5  rect_corner_2              turn −90°
 6  rect_top                   move 2.59 m   → E(3.70,2.08)  ψ=−90°
 7  rect_corner_3              turn −90°
 8  rect_side_2                move 0.80 m   → E(2.90,2.08)  ψ=180°
 9  rect_corner_4              turn −90°
10  rect_return_origin         move 1.49 m   → E(2.90,3.57)  ψ=90°  [整圈终点 A]
```

**整圈终点 (2.38, 3.32)** = 入口转后点，≠ corridor_goal **(2.50, 3.20)**。

逆时针：QR 选 CCW → 加载 `field_track_counterclockwise.yaml`，入口 **−90°**，拐角 **+90°×4**。

## 与旧模型的区别

| 旧 | 新 |
|----|-----|
| 每段 move 用 **实测入口** 当 P0 量 `progress` | 用 **名义 S** 量沿程，与图上一致 |
| `progress` / `lat` / `world沿程` / `dist_finish` 多套完成条件 | 段末：**沿程够 + 近 E + 横偏小**；整圈：**距 nominal_finish ≤ 0.15 m** |
| 调试里 P0=进段 odom | 调试里 **plan S→E** + **target=(x,y)** |

转弯段仍用 IMU 转固定角度；**不**对 turn 加载 world plan。

## 避障

仍为目标点序列（`direct_inertial_tester_avoidance.py`）：

1. **bypass** → **pass** → **exit**（短边 direct_cut）或 **rejoin** / **next_leg**  
2. 每拍 Pure Pursuit / 贴段 PD 朝 `active_avoidance_goal_xy()`  
3. **handoff**：已过障 + 近段末 **E**（`distance_to_segment_plan_end_m`）+ `|lat| ≤ 0.10 m`

详见 [AVOIDANCE.md](AVOIDANCE.md)。

## 关键文件

| 文件 | 作用 |
|------|------|
| `config/field_track_*.yaml` | 场测路点 S/E/ψ（真值） |
| `field_track.py` / `ring_track.py` | 加载 YAML、转弯 plan、避障辅助 |
| `world_segment.py` | along / lateral / point_on_segment 工具 |
| `world_plan_nav.py` | 段 plan 缓存、`projected_distance`、目标点 |
| `direct_inertial_tester.py` | 任务段切换、末段 finish |
| `direct_inertial_tester_avoidance.py` | 避障路点与 handoff |

## 离线验证

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racing_stage2_param_test
python3 -m racing_stage2_param_test.offline_ring_test --scenario rect_side_2_50
```
