# FIX_PLAN — racing_stage2_param_test 修复计划

## 优先级排序

| # | 严重度 | 文件 | 问题描述 | 修复内容 |
|---|---|---|---|---|
| 1 | **严重** | `direct_inertial_tester.py` | `run_move_segment()` 中 `_try_avoid_step()` 抢在段完成检查前 return，导致运动段永远无法结束 | 将段完成检查（`progress >= target_distance - tolerance`）提到 `_try_avoid_step()` **之前** |
| 2 | **严重** | `vision_inertial_tester.py` + `vision_inertial_test.launch.py` | 参数名不匹配：代码声明 `vision_lost_timeout`，launch 传入 `vision_lost_timeout_sec`，且 vision launch 未加载 `inertial_stage2.yaml` | 统一参数名为 `vision_lost_timeout_sec`；vision launch 补充 YAML 加载 |
| 3 | **中等** | `config/field_track_clockwise.yaml` + `config/field_track_counterclockwise.yaml` | AGENTS.md 声称路点真值仅来源于这两个文件，但它们不存在 | 用当前 `rectangle_*_leg_m` 值创建 YAML，建立世界坐标迁移基础 |
| 4 | **中等** | `bpu_direct_test.py` | 模型路径硬编码为 `/home/sunrise/...`，跨工作空间部署即失效 | 改用 `ament_index_python` 定位包共享目录 |
| 5 | **低** | `direct_inertial_tester.py` | `maybe_inject_detour()` 死代码（父类 + 子类重写均未被 `run_move_segment()` 调用） | 标记废弃/移除 |

## 修复顺序

1. BUG-1: `run_move_segment()` 逻辑重排
2. BUG-2: vision 参数名统一 + YAML 加载
3. BUG-3: 创建 `field_track_*.yaml`
4. BUG-4: `bpu_direct_test.py` 硬编码路径
5. `maybe_inject_detour()` 死代码清理
6. 更新 CHANGELOG
7. 编译验证

## 验证方法

```bash
colcon build --symlink-install --packages-select racing_stage2_param_test
```
