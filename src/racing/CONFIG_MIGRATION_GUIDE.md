# 参数配置整合说明

## 概述
已将三个阶段的参数配置整合到单独的统一配置文件，每个阶段一个文件，并添加了详细的中文注释。

## 配置文件结构

### Stage1 (racing_stage1)
```
config/
└── stage1_controller.yaml  # 统一配置文件（321行，全中文注释）
```

**功能模块：**
- 盲开前进
- 激光聚类避障（4状态：forward → avoiding → countersteering → recovering）
- 二维码识别触发后退
- 路径记录与后退
- 地图自由空间区域进入（A* 规划 + Pure Pursuit）

**包含参数：**
- 话题配置：8个话题 + 3个坐标系
- 盲开速度：2个参数
- 激光避障：30个参数（窗口检测、聚类过滤、紧急停障、避障动作）
- 避障恢复：15个参数（反舵 + 回正航向）
- 后退功能：9个参数
- 通道导航：50+个参数（路点、控制、Pure Pursuit、A*规划、重定向、左侧恢复）
- Phase切换：6个参数

---

### Stage2 (racing_stage2)
```
config/
├── stage2_controller.yaml              # 统一配置文件（461行，全中文注释）
├── field_track_clockwise.yaml          # 顺时针赛道轨迹
├── field_track_counterclockwise.yaml   # 逆时针赛道轨迹
└── obstacle_circle_markers.yaml        # 障碍物可视化标记
```

**功能模块：**
- 固定舵角弯段 + IMU 航向兜底直行
- 视觉车道居中修正（横向 offset PD + 纵向终点检测）
- 激光避障绕行（detour + side detour + corner detour）
- 地图栅格规划（通道内 A* 避障）
- 扫码方向解析（顺时针/逆时针）

**包含参数：**
- 话题配置：10个话题
- 控制频率：2个参数
- 速度控制：3个参数
- 转向控制：4个参数
- 到位精度：3个参数
- 方向解析：3个参数
- 通道导航：13个参数（Stage1 已完成，此处禁用）
- 纯追踪：3个参数
- 栅格规划器：9个参数
- 赛道几何：3个参数
- 激光避障：19个参数（前方障碍、侧方障碍、绕行轨迹、转角避障）
- 视觉融合：50+个参数（模型配置、横向纠偏、纵向修正、定位源、日志）

**原配置文件整合：**
- ~~inertial_stage2.yaml~~ → stage2_controller.yaml（已删除）
- ~~avoid_controller.yaml~~ → stage2_controller.yaml（已删除）

---

### Stage3 (racing_stage3)
```
config/
└── stage3_controller.yaml  # 统一配置文件（286行，全中文注释）
```

**功能模块：**
- A* 全局路径规划（避开地图禁区）
- Pure Pursuit 路径跟踪
- 激光聚类避障（4状态，同 Stage1）
- P 点视觉检测与伺服接近
- 矩形区域到达判定

**包含参数：**
- 话题配置：7个话题
- 控制频率：2个参数
- 路点与目标区域：8个参数
- Pure Pursuit：6个参数
- P 点视觉：9个参数
- 激光避障：26个参数（4状态避障 + 聚类窗口）
- A* 全局规划：11个参数

**原配置文件整合：**
- ~~return_stage3.yaml~~ → stage3_controller.yaml（已删除）
- ~~enhanced_return.yaml~~ → stage3_controller.yaml（已删除）

---

## Launch 文件更新

### Stage1
**文件：** `racing_stage1/launch/competition_stage1.launch.py`

**变更：**
```python
# 旧代码（无变化，已使用统一配置）
parameters=[
    os.path.join(stage1_config_dir, 'stage1_controller.yaml'),
    ...
]
```

---

### Stage2
**文件：** `racing_stage2/launch/competition_stage2.launch.py`

**变更：**
```python
# 旧代码
inertial_config = os.path.join(stage2_dir, 'config', 'inertial_stage2.yaml')
avoid_controller_config = os.path.join(stage2_dir, 'config', 'avoid_controller.yaml')
...
parameters=[
    inertial_config,
    avoid_controller_config,
    {...}
]

# 新代码
stage2_config = os.path.join(stage2_dir, 'config', 'stage2_controller.yaml')
...
parameters=[
    stage2_config,
    {...}
]
```

---

### Stage3
**文件：** `racing_stage3/launch/competition_stage3.launch.py`

**变更：**
```python
# 旧代码
return_config = os.path.join(stage3_dir, 'config', 'return_stage3.yaml')
...
parameters=[return_config, {...}]

# 新代码
stage3_config = os.path.join(stage3_dir, 'config', 'stage3_controller.yaml')
...
parameters=[stage3_config, {...}]
```

---

## 参数注释说明

所有参数注释包含以下信息：
1. **参数名称**：参数的 ROS 参数名
2. **单位**：(m), (m/s), (rad/s), (°), (s) 等
3. **功能说明**：参数的作用与影响
4. **取值范围/建议**：参数的合理取值范围
5. **依赖关系**：与其他参数的关联

### 注释格式示例
```yaml
# 前方窗口最小 X 距离 (m) — 距车体前端的近界
phase1_window_min_x: 0.18

# 避障线速度 (m/s) — 避障期间前进速度
avoid_linear_speed: 0.1

# 视觉模型路径（FastDeploy .bin 格式）
vision_model_path: /home/sunrise/dev_ws/src/racing/racing_stage2/models/bset.bin
```

---

## 参数调整指南

### Stage1 调整重点
- **避障灵敏度**：`phase1_window_*`, `safe_distance`, `avoid_min_duration_sec`
- **后退控制**：`back_target_x`, `back_linear_speed`, `back_angular_kp`
- **通道导航**：`corridor_waypoints_json`, `corridor_linear_speed`, `corridor_entry_region_radius_m`

### Stage2 调整重点
- **视觉权重**：`vision_primary_control`, `fusion_weight_vision`, `vision_angular_kp`
- **直行速度**：`ring_linear_speed`, `corridor_linear_speed`, `turn_linear_speed`
- **避障参数**：`detour_obstacle_distance`, `avoid_turn_away_deg`, `avoid_leg1_distance_m`

### Stage3 调整重点
- **路径规划**：`return_waypoints_json`, `goal_box_*`, `pursuit_linear_speed`
- **P 点视觉**：`p_approach_conf_threshold`, `p_complete_bbox_fill_ratio`, `p_approach_linear_speed`
- **避障控制**：`avoid_safe_distance`, `recovery_heading_kp`

---

## 验证清单

### 编译验证
```bash
cd ~/dev_ws
colcon build --packages-select racing_stage1 racing_stage2 racing_stage3
```

### 配置加载验证
```bash
# Stage1
ros2 launch racing_stage1 competition_stage1.launch.py

# Stage2 单独测试
ros2 launch racing_stage2 competition_stage2.launch.py enable_test_publisher:=true

# Stage3 单独测试
ros2 launch racing_stage3 competition_stage3.launch.py enable_test_publisher:=true
```

### 参数查询验证
```bash
# 查看 Stage1 实际加载的参数
ros2 param list /competition_controller

# 查看 Stage2 实际加载的参数
ros2 param list /stage2_inertial_navigator

# 查看 Stage3 实际加载的参数
ros2 param list /stage3_return_navigator
```

---

## 迁移注意事项

1. **旧配置文件已删除**：
   - `racing_stage2/config/inertial_stage2.yaml`
   - `racing_stage2/config/avoid_controller.yaml`
   - `racing_stage3/config/return_stage3.yaml`
   - `racing_stage3/config/enhanced_return.yaml`

2. **如果有自定义参数覆盖**：
   - 检查 launch 文件中的 `parameters=[...]` 字典覆盖
   - 将自定义参数迁移到新的 `stage*_controller.yaml`

3. **赛道轨迹文件保留**：
   - `field_track_clockwise.yaml`
   - `field_track_counterclockwise.yaml`
   - 这两个文件仍然独立，由代码根据扫码方向动态加载

4. **参数命名未变化**：
   - 所有参数名称保持不变
   - 代码无需修改，只是配置文件位置变更

5. **代码中配置文件引用说明**：
   - **Launch 文件**：已更新配置文件路径（见上文 Launch 文件更新）
   - **Python 代码**：无需修改
     - Stage1: 通过 ROS 参数系统加载，不涉及硬编码路径
     - Stage2: `field_track_*.yaml` 由 `field_track.py` 运行时动态解析
       - 路径解析逻辑在 `resolve_yaml_path()` 函数中
       - 根据方向自动选择 `field_track_{clockwise|counterclockwise}.yaml`
       - 优先使用 `*_world.yaml` 版本（如存在）
     - Stage3: 通过 ROS 参数系统加载，不涉及硬编码路径
   - **避障控制器**：Stage2 的 `avoid_controller.py` 是类模块，不直接加载配置文件

---

## 常见问题

### Q1: 编译后提示找不到配置文件
**A:** 确保运行了 `colcon build`，配置文件会被安装到 `install/<package>/share/<package>/config/`

### Q2: 节点启动时报参数未声明错误
**A:** 检查代码中 `declare_parameter()` 的参数名是否与配置文件一致

### Q3: 参数修改后不生效
**A:** 需要重新编译并source：
```bash
colcon build --packages-select racing_stage1 racing_stage2 racing_stage3
source install/setup.bash
```

### Q4: 想恢复旧配置文件
**A:** 旧配置文件已备份到 git 历史，使用以下命令恢复：
```bash
git checkout HEAD~1 -- src/racing/racing_stage2/config/
```

### Q5: field_track 文件在哪里加载？
**A:** `field_track_*.yaml` 文件由 `racing_stage2/field_track.py` 的 `resolve_yaml_path()` 函数在运行时动态加载：
```python
# 代码自动根据方向选择文件
if direction.startswith('counter'):
    fn = 'field_track_counterclockwise.yaml'
else:
    fn = 'field_track_clockwise.yaml'
```
不需要在 launch 文件中指定，也不需要修改代码。

---

## 下一步工作建议

1. **参数优化**：根据场地测试结果调整关键参数
2. **文档完善**：为每个阶段添加调参经验文档
3. **参数验证**：编写参数范围检查脚本，防止误配置
4. **动态调参**：考虑通过 `ros2 param set` 实时调整部分参数

---

**整合完成时间：** 2026-07-17  
**版本：** v1.0  
**维护者：** Claude Code
