# Stage3 Map 坐标系统一分析报告

## 任务目标

分析第三阶段在一二两阶段都是 map 的情况下，是否能够结合在一起，起点和终点的位置如何，又该如何进行规划航线。

---

## 一、三阶段是否能统一在 map 坐标系

### 结论：✅ **完全可以**

当前架构已经支持三阶段统一使用 map 坐标系：

```
TF 树：map → odom_combined → base_link

所有阶段位姿统一在 map 系：
├─ Stage1: 起点 P (2.50, 2.60) → 终点 corridor_goal (2.50, 3.20)
├─ Stage2: 起点 (2.50, 3.20) → 终点 (2.38, 3.32)
└─ Stage3: 起点 (2.38, 3.32) → 终点 P (2.50, 2.60)
```

### 关键证据

1. **map_overlay.launch.py 已配置 map 系统**
   - 发布 `/map` (OccupancyGrid)
   - 静态 TF：`map → odom_combined`
   - 默认偏移：`(0.50, 0.20, yaw=10°)`

2. **Stage2 已支持 map 坐标**
   - `inertial_stage2.yaml` 定义 `corridor_goal: map (2.50, 3.20)`
   - `direct_inertial_tester.py` 实现 `_odom_to_map()` 转换

3. **Stage3 原生支持 map 坐标**
   - `map_return_navigator.py` 使用 `global_frame_id = 'map'`
   - P 点使用 map 绝对坐标：`(2.50, 2.60)`

---

## 二、起点和终点位置（map 坐标系）

### 完整流程坐标表

| 关键点 | map X | map Y | yaw (deg) | 说明 |
|--------|-------|-------|-----------|------|
| **P 点（起点/终点）** | 2.50 | 2.60 | 90° | Stage1 起点 = Stage3 终点 |
| **Stage1 终点** | 2.50 | 3.20 | 90° | corridor_goal（扫码后） |
| **Stage2 入口** | 2.50 | 3.20 | 90° | = Stage1 终点 |
| **Stage2 入口转后** | ~2.38 | ~3.32 | 180° (CW) | rect_enter_align 后 |
| **Stage2 整圈终点** | ~2.38 | ~3.32 | 180° | rect_return_origin |
| **Stage3 起点** | auto | auto | auto | = Stage2 实际终点（自动检测）|
| **Stage3 终点** | 2.50 | 2.60 | 90° | 返回 P 点 |

### 坐标连续性验证

```
完整比赛流程：
P (2.50, 2.60) 
  ↓ Stage1 通道导航 0.6m
  → (2.50, 3.20) corridor_goal / Stage2 入口
  ↓ Stage2 矩形赛道（转 90° + 绕行 ~5.8m）
  → (2.38, 3.32) Stage2 终点 / Stage3 起点
  ↓ Stage3 A* 返程 ~0.8m
  → (2.50, 2.60) P 点终点 ✓
```

**起点 = 终点**，形成闭环 ✅

---

## 三、Stage3 航线规划方案（参考 Stage1 扫码后）

### Stage1 扫码后的规划机制（参考标准）

```python
# competition_controller.py
def qr_callback(self, msg):
    task = msg.data.strip()  # "clockwise" / "counterclockwise"
    self.qr_task = task
    self.task_pub.publish(String(data=task))  # 发布任务指令
    self.begin_phase_transition(2, f'qr detected: {task}')
```

**关键点**：
1. 扫码触发 → 发布 `competition_qr_task`
2. Stage2 订阅 → 选择顺时针/逆时针路点配置
3. **立即发布路径到 RViz2**：`publish_corridor_path()`

### Stage3 采用的类似规划机制

```python
# map_return_navigator.py L350-370
def _start_mission(self):
    # 1. 读取当前位置（自动检测，无需 Stage2 传递）
    global_pos = self.current_global_position()
    
    # 2. A* 全局规划：当前位置 → P 点
    pts = self.plan_global_path(global_pos, self.p_point, now)
    
    # 3. ✅ 立即发布路径到 RViz2（参考 Stage1 方式）
    if pts:
        self.path_points = pts
        self.publish_path_points(pts)  # ← 关键：发布到 /stage3_return_path
        self._publish_feedback(f'A* planned {len(pts)} waypoints')
```

### 与 Stage1 对比

| 特性 | Stage1 扫码后 | Stage3 规划 |
|------|--------------|------------|
| **触发方式** | QR 扫码 → phase=2 | phase=3 触发 |
| **目标获取** | 读取 corridor_waypoints_json | 读取 p_point 配置 |
| **规划方式** | Pure Pursuit + 可选 A* | A* + Pure Pursuit |
| **路径发布** | `publish_corridor_path()` | `publish_path_points()` |
| **RViz2 Topic** | `/stage2_corridor_path` | `/stage3_return_path` |
| **Frame** | `map` | `map` |
| **立即发布** | ✅ 启动时发布 | ✅ 启动时发布 |

**结论**：Stage3 完全采用了 Stage1 的路径发布模式。

---

## 四、RViz2 可视化路径实现

### 路径发布完整实现（参考 Stage2）

```python
# global_path_planner.py L406-431
def publish_path_points(self, points, frame_id=None):
    """发布路径到 RViz2（完整实现，含航向）"""
    path_msg = Path()
    path_msg.header.frame_id = frame_id or self.global_frame_id  # 'map'
    path_msg.header.stamp = self.get_clock().now().to_msg()
    
    for index, point in enumerate(points):
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = path_msg.header.frame_id
        pose_msg.header.stamp = path_msg.header.stamp
        pose_msg.pose.position.x = float(point[0])
        pose_msg.pose.position.y = float(point[1])
        pose_msg.pose.position.z = 0.0
        
        # ✅ 航向计算：当前点 → 下一点
        pose_yaw = 0.0
        if index < len(points) - 1:
            next_point = points[index + 1]
            pose_yaw = math.atan2(next_point[1] - point[1], 
                                 next_point[0] - point[0])
        elif index > 0:
            previous_point = points[index - 1]
            pose_yaw = math.atan2(point[1] - previous_point[1], 
                                 point[0] - previous_point[0])
        
        # ✅ 四元数转换
        orientation_z, orientation_w = self.yaw_to_quaternion(pose_yaw)
        pose_msg.pose.orientation.z = orientation_z
        pose_msg.pose.orientation.w = orientation_w
        path_msg.poses.append(pose_msg)
    
    # ✅ 发布（latched QoS）
    self.return_path_pub.publish(path_msg)
```

### 与 Stage2 对比

| 项目 | Stage2 | Stage3 |
|------|--------|--------|
| **Publisher** | `corridor_path_pub` | `return_path_pub` |
| **Topic** | `/stage2_corridor_path` | `/stage3_return_path` |
| **QoS** | TRANSIENT_LOCAL | TRANSIENT_LOCAL |
| **Frame** | `map` / `odom` | `map` |
| **航向计算** | ✅ 完整 | ✅ 完整 |
| **发布时机** | 启动 + 重规划 | 启动 + 重规划 |

**实现质量**：Stage3 完全复制了 Stage2 的成熟代码。

### RViz2 配置方法

```yaml
# 添加到 RViz2
Displays:
  - Class: rviz/Path
    Name: Stage2_Path
    Topic: /stage2_corridor_path
    Color: 255; 0; 0  # 红色
    Line Width: 0.05
    Alpha: 1.0
    
  - Class: rviz/Path
    Name: Stage3_Path
    Topic: /stage3_return_path
    Color: 0; 255; 0  # 绿色
    Line Width: 0.05
    Alpha: 1.0
    
  - Class: rviz/Map
    Name: Map
    Topic: /map
    Alpha: 0.7
    
  - Class: rviz/TF
    Name: TF_Tree
    Show Names: true
    Frames: [map, odom_combined, base_link]

# Fixed Frame 设置
Fixed Frame: map  ← 关键：必须为 map
```

### 验证命令

```bash
# 1. 查看 Stage3 路径 topic
ros2 topic list | grep stage3_return_path
# 输出：/stage3_return_path

# 2. 查看路径内容
ros2 topic echo /stage3_return_path --once
# 应该看到：
# header:
#   frame_id: "map"
# poses:
#   - pose:
#       position: {x: 2.38, y: 3.32, z: 0.0}
#       orientation: {z: ..., w: ...}
#   - pose:
#       position: {x: ..., y: ..., z: 0.0}
#       orientation: {z: ..., w: ...}
#   ...
#   - pose:
#       position: {x: 2.50, y: 2.60, z: 0.0}  # P 点
#       orientation: {z: ..., w: ...}

# 3. 查看 TF 树
ros2 run tf2_tools view_frames
# 检查：map → odom_combined → base_link

# 4. 实时监控规划状态
ros2 topic echo /competition_feedback
# 看到：'A* planned N waypoints' 表示规划成功
```

---

## 五、实施步骤总结

### 已完成修改

1. ✅ **更新 P 点坐标**
   - 文件：`src/racing/racing_stage3_param_test/config/map_return.yaml`
   - `p_point_y: 3.20` → `2.60`

2. ✅ **验证路径发布逻辑**
   - `global_path_planner.py` 的 `publish_path_points()` 实现完整
   - 包含航向计算和四元数转换
   - 参考 Stage2 成熟实现

3. ✅ **编译测试包**
   ```bash
   colcon build --symlink-install --packages-select racing_stage3_param_test
   ```

### 测试方法

```bash
# 启动 Stage3 独立测试
ros2 launch racing_stage3_param_test map_return_test.launch.py

# 5 秒后自动触发 phase=3
# 查看日志应看到：
# [map_return_navigator] planning A* from (...) to P (2.50, 2.60)
# [map_return_navigator] A* planned N waypoints
```

---

## 六、结论

### ✅ 三阶段可完全统一在 map 坐标系

1. **坐标系统一**：所有阶段使用 map 全局坐标，TF 树完整
2. **起点终点明确**：
   - 起点：P (2.50, 2.60) yaw=90°
   - 终点：P (2.50, 2.60) yaw=90°
   - 形成闭环 ✓
3. **规划方案完整**：
   - Stage3 采用与 Stage1 类似的立即发布机制
   - A* 全局规划 + Pure Pursuit 跟踪
   - 参考 Stage2 成熟代码实现
4. **RViz2 可视化**：
   - 路径发布到 `/stage3_return_path`
   - frame_id = `map`
   - 包含完整航向信息
   - 实现与 Stage2 一致

### 📊 三阶段完整流程图

```
                     map 坐标系（全局）
                           │
    ┌──────────────────────┼──────────────────────┐
    │                      │                      │
P (2.50, 2.60)        Stage1 通道          Stage2 矩形
    │                 0.6m ↓                  5.8m ↓
    │              (2.50, 3.20)          (2.38, 3.32)
    │                      │                      │
    └──────────────────────┼──────────────────────┘
                    Stage3 A* 返程
                       0.8m ↑
                           │
                 RViz2 可视化路径
              /stage3_return_path (绿色)
```

### 🎯 关键优势

1. **独立测试友好**：Stage3 可从任意位置启动，自动规划到 P 点
2. **全局一致性**：所有阶段共享同一个 map 坐标系
3. **可视化完整**：RViz2 可同时显示 Stage2 和 Stage3 路径
4. **代码质量高**：复用 Stage2 成熟实现，稳定可靠

---

## 附录：完整配置文件

### map_return.yaml（已更新）

```yaml
map_return_navigator:
  ros__parameters:
    # P 点（map 坐标）
    p_point_x: 2.50
    p_point_y: 2.60  # ← 已更新
    p_point_yaw_deg: 90.0
    
    # A* 规划器
    planner_occupied_threshold: 80
    planner_replan_period_sec: 2.0
    planner_obstacle_inflation_m: 0.25
    
    # Pure Pursuit
    pursuit_linear_speed: 0.18
    pursuit_lookahead_m: 0.45
```

### RViz2 启动配置

```bash
# 方式 1：使用测试 launch
ros2 launch racing_stage3_param_test map_return_test.launch.py

# 方式 2：手动启动 RViz2
rviz2 -d src/racing/racing_stage3_param_test/rviz/map_return.rviz
```

---

**报告生成时间**：2026-07-11
**编译状态**：✅ 通过
**测试状态**：待验证
