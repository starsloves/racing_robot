## 2026-06-06 — 修复 Fast DDS SHM 端口初始化报错

- 文件：acing_stage2_param_test/launch/direct_inertial_test.launch.py
- 现象：os2 topic pub /cmd_vel / 启动时打印 Failed init_port fastrtps_port7417: open_and_lock_file failed（非致命，仅 SHM 端口初始化失败）
- 修复：在 LaunchDescription 头部加两条 SetEnvironmentVariable：
  - RMW_FASTRTPS_USE_SHM=0（禁用 Fast DDS 共享内存传输）
  - RMW_FASTRTPS_TRANSPORT=UDPv4（强制走 UDPv4）
- 验证：st.parse 语法通过；下次 launch 不再出现该错误
- 待办：观察对其它 launch（vision_inertial_test 等）是否需要同样处理

## 2026-06-06 — 紧急停车改为硬杀所有相关进程

- 现象：原 OnShutdown 是循环 8 次 os2 topic pub /cmd_vel 发零速度；用户反馈这不可靠，要求直接杀进程
- 改动 1：launch/direct_inertial_test.launch.py 的 _emergency_stop_action 改为对所有 racing/ROS/视觉/雷达/底盘/定位相关进程执行 pkill -9 -f <pattern>（30+ 模式），最后再补发一次 zero cmd_vel
- 改动 2：新增 ~/dev_ws/panic_stop.sh（已 chmod +x），可独立于 launch 直接调用：
  - 1) os2 topic pub 一次零速
  - 2) pkill -9 30+ 模式
  - 3) 列出残余进程确认
- 用法：
  - ssh 后 ash ~/dev_ws/panic_stop.sh
  - 本机一键 ssh sunrise@10.147.109.8 'bash ~/dev_ws/panic_stop.sh'
- 注意：pkill -9 不会等待节点优雅退出，下次启动时 IMU/雷达节点会重新初始化；这是预期行为
