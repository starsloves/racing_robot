# Racing Robot ROS 2 Workspace

This repository is a ROS 2 Humble colcon workspace for a three-stage racing-robot competition system running on an RDK X5.

The production workflow is:

1. **Stage 1** drives forward, scans the QR task, returns along the recorded path, and navigates the corridor.
2. **Stage 2** follows the inertial race track. The QR result selects the clockwise or counterclockwise direction.
3. **Stage 3** returns to the visual-search goal, detects the P marker, and stops using marker-region depth.

For the authoritative production behavior, state transitions, coordinate conventions, topic topology, and safety rules, read [RACING_OVERVIEW.md](src/racing/RACING_OVERVIEW.md) before changing competition code.

## Repository layout

| Path | Purpose |
| --- | --- |
| `src/racing/` | Competition logic, launch files, and race-specific packages |
| `src/origincar/` | Base vehicle, messages, description, and bringup packages |
| `src/LSLIDAR_X_ROS2-20240228/` | LSLIDAR ROS 2 driver packages |
| `docs/` | Persistent project documentation and change log |
| `build/`, `install/`, `log/` | Generated build, installation, and runtime-log artifacts |

## Requirements

- Ubuntu 22.04
- ROS 2 Humble
- RDK X5 board environment and connected vehicle sensors

Load ROS before building or launching:

```bash
source /opt/ros/humble/setup.bash
```

## Build

Build one package while developing it:

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select racing_bringup
source install/setup.bash
```

To build the entire workspace, omit `--packages-select`.

## Launch

Start the complete competition workflow:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch racing_bringup competition_total.launch.py
```

Stage-specific launch commands:

```bash
ros2 launch racing_stage1 competition_stage1.launch.py
ros2 launch racing_stage2 competition_stage2.launch.py
ros2 launch racing_stage3 competition_stage3.launch.py
ros2 launch origincar_base origincar_bringup.launch.py
```

`competition_total.launch.py` is the production entry point. In that mode, Stage1 is the only publisher to `/cmd_vel`; Stage2 and Stage3 publish candidate commands to `/stage2_cmd_vel` and `/stage3_cmd_vel` respectively.

## Important interfaces

| Topic | Role |
| --- | --- |
| `/cmd_vel` | Final base command, published by the Stage1 supervisor |
| `/stage2_cmd_vel`, `/stage3_cmd_vel` | Candidate commands from later stages |
| `/imu/data` | Required yaw source for all navigation and turn control |
| `/odom_combined` | XY position and distance source |
| `/scan` | Lidar input for local obstacle avoidance |
| `competition_phase` | Competition phase: `1 -> 2 -> 3` |
| `competition_qr_task` | QR-selected direction: `clockwise` or `counterclockwise` |

Do not use odometry orientation for navigation. All heading and turn calculations must use `/imu/data`.

## Logs and maintenance

- Stage logs are written below `log/competition_stage1/`, `log/competition_stage2/`, and `log/competition_stage3/`.
- `log/` is disposable; permanent documentation belongs in `docs/`.
- Record every code, configuration, or launch change in `docs/CHANGELOG.md`.
- Do not treat parameter-test packages or `bak/` content as part of the production competition flow.

