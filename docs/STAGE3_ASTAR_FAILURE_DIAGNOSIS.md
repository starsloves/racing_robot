# Stage3 A* 路径规划失败诊断报告

**日期**: 2026-07-11  
**问题**: A* 从 (0.00, 0.00) 到 P(2.50, 2.60) 规划失败，回退到直线导航

## 📋 问题现象

```log
[map_return_navigator-11] [INFO] [1783771191.064706218] [map_return_navigator]: planning A* from (0.00,0.00) to P (2.50,2.60)
[map_return_navigator-11] [INFO] [1783771191.071134643] [map_return_navigator]: A* failed, using direct line to P point
```

**关键观察**:
- 起点坐标异常：**(0.00, 0.00)** — 应该是 Stage2 结束位置，不应该在原点
- 规划耗时 **6.4 ms**，极快失败，说明没有真正执行搜索
- 系统回退到直线导航（fallback 机制）

## 🔍 根本原因（已确认）

### **缺少地图数据源 — `carto_slam: false` 导致无 /map 发布**

**问题代码**: `map_return_test.launch.py:29`

```python
DeclareLaunchArgument('include_carto', default_value='false'),  # 修复：EKF 必须启动，carto 禁用
```

**传递给**: `competition_support.launch.py:35` → `origincar_bringup.launch.py`

```python
launch_arguments={
    'carto_slam': LaunchConfiguration('carto_slam'),  # = 'false'
}.items(),
```

### 后果链

1. **无 /map topic** → `self.latest_map is None`
2. **无 map→odom TF** → `lookup_2d_transform('map', 'odom')` 失败
3. **位置回退到原点** → `current_global_position()` 返回 `self.current_position` 初始值 (0.00, 0.00)
4. **A* 无法规划** → `build_static_planner_grid()` 返回 `None`

### 验证

**代码路径**: `global_path_planner.py:151-153`

```python
def build_static_planner_grid(self):
    if self.latest_map is None:
        return None  # ← A* 失败入口
```

**代码路径**: `map_return_navigator.py:358-370`

```python
pts = self.plan_global_path(global_pos, self.p_point, self._now_sec())
if pts:
    # 成功路径
else:
    # ← 走到这里：无 map → 无 A* → fallback 直线
    self._publish_feedback('A* failed, using direct line to P point')
    self.path_points = [global_pos or self.current_position, self.p_point]
```

## ✅ 修复方案

### 方案 A: 启用 Cartographer SLAM（推荐用于实车）

**修改**: `src/racing/racing_stage3_param_test/launch/map_return_test.launch.py:29`

```python
------- SEARCH
DeclareLaunchArgument('include_carto', default_value='false'),  # 修复：EKF 必须启动，carto 禁用
=======
DeclareLaunchArgument('include_carto', default_value='true'),   # Stage3 需要 /map 和 map→odom TF
+++++++ REPLACE
```

**效果**:
- ✅ 启动 Cartographer → 发布 `/map`（实时 SLAM）
- ✅ 发布 `map → odom` TF
- ✅ A* 规划器获得栅格地图
- ⚠️ 需要激光雷达（`include_lidar: true`）
- ⚠️ 计算开销增加（建议 Horizon X3）

---

### 方案 B: 使用预建地图 + AMCL（推荐用于已知场地）

**步骤 1**: 预先建图并保存

```bash
# 运行 Cartographer 建图
ros2 launch origincar_bringup origincar_bringup.launch.py carto_slam:=true

# 保存地图
ros2 run nav2_map_server map_saver_cli -f ~/maps/competition_map
```

**步骤 2**: 创建地图加载启动文件

**新建**: `src/racing/racing_stage3_param_test/launch/map_loader.launch.py`

```python
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

def generate_launch_description():
    # 地图文件路径（需预先建图）
    map_yaml_file = LaunchConfiguration('map')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=os.path.expanduser('~/maps/competition_map.yaml'),
            description='Full path to map yaml file'
        ),
        
        # Map Server（发布 /map）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml_file,
                'use_sim_time': False
            }]
        ),
        
        # Lifecycle Manager（激活 map_server）
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map',
            output='screen',
            parameters=[{
                'autostart': True,
                'node_names': ['map_server']
            }]
        ),
        
        # 静态 TF：map → odom（假设已定位）
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_publisher',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
    ])
```

**步骤 3**: 修改测试启动文件

**修改**: `map_return_test.launch.py`

```python
------- SEARCH
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
=======
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, GroupAction
+++++++ REPLACE
```

```python
------- SEARCH
def generate_launch_description():
    stage3_dir = get_package_share_directory('racing_stage3_param_test')
    config = os.path.join(stage3_dir, 'config', 'map_return.yaml')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')
=======
def generate_launch_description():
    stage3_dir = get_package_share_directory('racing_stage3_param_test')
    config = os.path.join(stage3_dir, 'config', 'map_return.yaml')
    support_launch_path = os.path.join(stage3_dir, 'launch', 'competition_support.launch.py')
    map_loader_launch_path = os.path.join(stage3_dir, 'launch', 'map_loader.launch.py')
+++++++ REPLACE
```

```python
------- SEARCH
        # 硬件驱动（含 SLAM Toolbox → /map）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(support_launch_path),
            launch_arguments={
                'include_bringup': LaunchConfiguration('include_bringup'),
                'include_lidar': LaunchConfiguration('include_lidar'),
                'include_bno055': LaunchConfiguration('include_bno055'),
                'include_camera': 'false',
                'include_depth': 'false',
                'carto_slam': LaunchConfiguration('include_carto'),
            }.items(),
        ),
=======
        # 硬件驱动
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(support_launch_path),
            launch_arguments={
                'include_bringup': LaunchConfiguration('include_bringup'),
                'include_lidar': LaunchConfiguration('include_lidar'),
                'include_bno055': LaunchConfiguration('include_bno055'),
                'include_camera': 'false',
                'include_depth': 'false',
                'carto_slam': 'false',  # 使用预建地图
            }.items(),
        ),
        
        # 预建地图加载器（/map + map→odom TF）
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_loader_launch_path),
        ),
+++++++ REPLACE
```

**效果**:
- ✅ 无实时 SLAM 开销
- ✅ 稳定的 /map 和 map→odom TF
- ⚠️ 需要预建地图
- ⚠️ 静态 TF 假设车辆已定位

---

### 方案 C: 测试环境临时方案（仅验证逻辑）

**适用场景**: 无激光雷达、无预建地图，仅验证 A* 算法逻辑

**修改**: `map_return_test.launch.py`，添加虚拟地图发布器

```python
------- SEARCH
        # 地图导航返程节点
        Node(
            package='racing_stage3_param_test',
            executable='map_return_navigator',
            name='map_return_navigator',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
=======
        # 虚拟地图发布器（测试用）
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': os.path.join(stage3_dir, 'map', 'test_empty.yaml'),
                'use_sim_time': False
            }],
            condition=IfCondition('false')  # 需要创建 test_empty.yaml
        ),
        
        # 静态 TF：map → odom
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_odom_tf',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        ),
        
        # 地图导航返程节点
        Node(
            package='racing_stage3_param_test',
            executable='map_return_navigator',
            name='map_return_navigator',
            parameters=[config],
            output='screen',
            emulate_tty=True,
        ),
+++++++ REPLACE
```

**创建虚拟地图**: `src/racing/racing_stage3_param_test/map/test_empty.yaml`

```yaml
image: test_empty.pgm
resolution: 0.05
origin: [0.0, 0.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.196
```

**创建空白地图**: `test_empty.pgm`（80x80像素，全白 = 空闲空间）

```bash
convert -size 80x80 xc:white src/racing/racing_stage3_param_test/map/test_empty.pgm
```

---

## 🔬 验证步骤

### 修复后验证

```bash
# 启动系统（任选一个方案）
ros2 launch racing_stage3_param_test map_return_test.launch.py

# 另一终端检查地图
ros2 topic echo /map --once | head -20

# 检查 TF
ros2 run tf2_ros tf2_echo map odom

# 检查日志
# 期望：A* planned X waypoints（而非 "A* failed"）
```

### 关键指标

| 检查项 | 期望值 | 当前值 |
|--------|--------|--------|
| `/map` topic | 存在，非空 | ❌ 不存在 |
| `map→odom` TF | 存在 | ❌ 不存在 |
| 起点坐标 | 非 (0,0) | ❌ (0.00, 0.00) |
| A* 规划 | 成功 | ❌ 失败 |

---

## 📊 方案对比

| 方案 | 优点 | 缺点 | 适用场景 |
|------|------|------|----------|
| **A: Cartographer** | 实时建图，无需预建地图 | 计算开销大，需要激光 | 未知场地/实车比赛 |
| **B: 预建地图+AMCL** | 稳定，开销小 | 需要预建图 | 已知场地/重复测试 |
| **C: 虚拟地图** | 快速验证算法 | 非真实环境 | 开发调试 |

---

## 🎯 推荐行动

### 立即修复（最小改动）

**单行修改**: `map_return_test.launch.py:29`

```python
default_value='true'  # 原为 'false'
```

**验证**:
```bash
colcon build --symlink-install --packages-select racing_stage3_param_test
source install/setup.bash
ros2 launch racing_stage3_param_test map_return_test.launch.py
```

**期望日志**:
```
[map_return_navigator] planning A* from (X.XX,Y.YY) to P (2.50,2.60)
[map_return_navigator] A* planned N waypoints
```

### 长期优化

1. 预建比赛场地地图（方案 B）
2. 添加 AMCL 定位替代静态 TF
3. 添加地图有效性检查（`_start_mission` 中）

---

## 🔧 额外调试建议

如启用 Cartographer 后仍失败，添加调试日志：

**修改**: `src/racing/racing_stage3_param_test/racing_stage3_param_test/map_return_navigator.py:358`

```python
------- SEARCH
        global_pos = self.current_global_position()
        if global_pos is None:
            global_pos = self.current_position
        pts = self.plan_global_path(global_pos, self.p_point, self._now_sec())
=======
        global_pos = self.current_global_position()
        self.get_logger().info(
            f'[DEBUG] odom_pos={self.current_position} global_pos={global_pos} '
            f'map_loaded={self.latest_map is not None}'
        )
        if global_pos is None:
            global_pos = self.current_position
        pts = self.plan_global_path(global_pos, self.p_point, self._now_sec())
+++++++ REPLACE
```

这样可以看到：
- Odometry 是否就绪
- TF 是否成功转换
- 地图是否加载

---

## 📝 结论

**根本原因**: `carto_slam: false` → 无 `/map` 发布 → A* 规划器无数据源

**最简修复**: 改 `default_value='true'`（1 行代码）

**生产建议**: 预建地图 + AMCL（方案 B），平衡性能与稳定性

**下一步**: 执行验证步骤，确认 A* 规划成功
