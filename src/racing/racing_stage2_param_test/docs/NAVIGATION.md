# Stage2 参数测试 — 导航模型（世界坐标）

## 原则（2026-05 重构）

全程 **一套 odom 世界坐标** `(x, y, yaw)`，不另建坐标系。

控制器只维护两类量：

| 量 | 含义 |
|----|------|
| **当前位姿** | `/odom_combined`（离线为仿真积分） |
| **当前目标点** | 避障阶段：bypass / pass / exit 等世界 `(x,y)`；无障直行：本段名义终点 **E** |

直行段的几何来自 `ring_track` 链式名义折线（与蓝线、落锥一致）：

- **S** = 段起点，**E** = 段终点，**ψ** = 段航向  
- **沿程** = 当前位姿在 S→ψ 上的投影，**夹在 [0, 段长]**（外鼓不会变成 1.02/0.50）  
- **横偏** = 到 plan 弦线的法向距离  

实现：`world_plan_nav.py`（`DirectInertialTesterWorldPlanMixin`）。

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
| `ring_track.py` | 名义 S/E、障碍插值、整圈终点 |
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
