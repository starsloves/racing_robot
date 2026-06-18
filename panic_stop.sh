#!/bin/bash
# racing-ssh 一键硬停脚本
# 用法：
#   1) ssh 到车端后执行：   bash ~/dev_ws/panic_stop.sh
#   2) 或者从本机直接执行： ssh sunrise@10.147.109.8 "bash ~/dev_ws/panic_stop.sh"
#
# 行为：
#   - 尝试先发一次零速度（给小车最后一次软停机会）
#   - 然后 pkill -9 所有 racing / ROS / 视觉 / 雷达 / 定位 / 底盘 相关进程
#   - 列出残余进程

set +e

RED=$'\e[31m'; GRN=$'\e[32m'; YEL=$'\e[33m'; NC=$'\e[0m'

echo "${YEL}[1/3] soft stop: send zero cmd_vel${NC}"
timeout 2 ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" 2>/dev/null

echo "${YEL}[2/3] hard kill: pkill -9 racing/ros related processes${NC}"
TARGETS=(
  "direct_inertial_tester"
  "twist_cmd_relay"
  "data_recorder"
  "stage2_inertial_navigator"
  "competition_support"
  "lslidar"
  "origincar"
  "ros2"
  "fastrtps"
  "carthographer"
  "cartographer"
  "robot_state_publisher"
  "static_transform"
  "imu_filter"
  "robot_localization"
  "ekf"
  "realsense"
  "usb_cam"
  "depthimage"
  "vision_inertial"
  "vision_record"
  "simple_avoidance"
  "qr_scanner"
  "voice_driver"
  "racing"
  "stage2_cmd_vel"
  "cmd_vel"
  "pointcloud_to_laserscan"
  "laser_filter"
  "slam"
  "rviz"
  "odom"
  "twist_mux"
  "foxglove"
  "rosbridge"
)
for t in "${TARGETS[@]}"; do
  pkill -9 -f "$t" 2>/dev/null
done
sleep 0.3

echo "${YEL}[3/3] survivors:${NC}"
REMAIN=$(ps -ef | grep -E "ros|racing|fastrtps|cartograph|lidar|origincar|realsense|usb_cam|vision|stage2|simple_avoid|qr_scan|voice|imudrift|ekf" | grep -v grep)
if [ -z "$REMAIN" ]; then
  echo "${GRN}none — all racing/ros processes killed.${NC}"
else
  echo "$REMAIN"
fi
exit 0
