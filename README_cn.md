# 竞速机器人 ROS 2 工作空间

本仓库是运行于 RDK X5 开发板的 ROS 2 Humble colcon 工作空间，用于三阶段竞速机器人比赛。

生产流程分为三个阶段：

1. **第一阶段**：车辆盲驱并扫码，根据二维码任务倒退回放，再完成通道导航。
2. **第二阶段**：车辆沿惯性赛道行驶；二维码决定顺时针或逆时针方向。
3. **第三阶段**：车辆返回视觉搜索点，识别 P 标志，并依据 P 框区域深度停车。

修改赛事相关代码前，必须先阅读 [RACING_OVERVIEW.md](src/racing/RACING_OVERVIEW.md)。该文档是生产流程、状态机、坐标系、话题拓扑和安全策略的总览。

## 目录说明

| 路径 | 用途 |
| --- | --- |
| `src/racing/` | 赛事逻辑、启动文件和赛事专用功能包 |
| `src/origincar/` | 车辆底盘、消息、模型和 bringup 功能包 |
| `src/LSLIDAR_X_ROS2-20240228/` | LSLIDAR ROS 2 驱动 |
| `docs/` | 永久性项目文档和变更日志 |
| `build/`、`install/`、`log/` | 可再生的编译、安装和运行日志产物 |

## 环境要求

- Ubuntu 22.04
- ROS 2 Humble
- 已连接车辆传感器的 RDK X5 板端环境

编译或启动前先加载 ROS 环境：

```bash
source /opt/ros/humble/setup.bash
```

## 编译

开发单个功能包时可使用：

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racing_bringup
source install/setup.bash
```

需要全空间编译时，去掉 `--packages-select` 即可。

## 启动

生产环境的完整比赛入口：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch racing_bringup competition_total.launch.py
```

各阶段可独立启动：

```bash
ros2 launch racing_stage1 competition_stage1.launch.py
ros2 launch racing_stage2 competition_stage2.launch.py
ros2 launch racing_stage3 competition_stage3.launch.py
ros2 launch origincar_base origincar_bringup.launch.py
```

`competition_total.launch.py` 是生产总入口。总启动时，只有第一阶段监管节点发布 `/cmd_vel`；第二、三阶段分别向 `/stage2_cmd_vel` 和 `/stage3_cmd_vel` 发布候选控制指令，再由第一阶段仲裁转发。

## 核心话题

| 话题 | 用途 |
| --- | --- |
| `/cmd_vel` | 最终底盘控制指令，由第一阶段监管节点发布 |
| `/stage2_cmd_vel`、`/stage3_cmd_vel` | 第二、三阶段候选控制指令 |
| `/imu/data` | 全部航向和转角计算的唯一来源 |
| `/odom_combined` | XY 位置和距离来源 |
| `/scan` | 激光雷达局部避障输入 |
| `competition_phase` | 比赛阶段：`1 -> 2 -> 3` |
| `competition_qr_task` | 二维码任务方向：`clockwise` 或 `counterclockwise` |

禁止使用里程计的 `orientation/yaw` 参与导航。所有航向和转角计算必须使用 `/imu/data`。

## 日志与维护

- 三阶段日志位于 `log/competition_stage1/`、`log/competition_stage2/` 和 `log/competition_stage3/`。
- `log/` 可随时清理；永久性文档必须放在 `docs/`。
- 每次修改代码、配置或启动文件后，都要在 `docs/CHANGELOG.md` 记录变更。
- 参数测试功能包和 `bak/` 目录不属于生产比赛流程。

