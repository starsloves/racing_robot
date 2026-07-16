#!/bin/bash
# Stage1 通道导航实时监控

echo "=== Stage1 通道导航监控 ==="
echo "按 Ctrl+C 退出"
echo ""

while true; do
    clear
    echo "╔════════════════════════════════════════════════════════════╗"
    echo "║          Stage1 通道导航实时状态监控                       ║"
    echo "╚════════════════════════════════════════════════════════════╝"
    echo ""
    
    # 获取当前位置
    odom_data=$(timeout 0.5 ros2 topic echo /odom_combined --once 2>/dev/null | grep -A 3 "position:" | tail -3)
    if [ -n "$odom_data" ]; then
        pos_x=$(echo "$odom_data" | grep "x:" | awk '{print $2}')
        pos_y=$(echo "$odom_data" | grep "y:" | awk '{print $2}')
        echo "📍 当前位置: ($pos_x, $pos_y)"
    else
        echo "📍 当前位置: (数据获取中...)"
    fi
    
    # 获取目标位置（从配置文件）
    target=$(grep "corridor_waypoints_json" ~/dev_ws/src/racing/racing_stage1/config/stage1_controller.yaml | grep -oP '\[\{"x":[0-9.]+,"y":[0-9.]+\}\]')
    target_x=$(echo "$target" | grep -oP '"x":\K[0-9.]+')
    target_y=$(echo "$target" | grep -oP '"y":\K[0-9.]+')
    echo "🎯 目标位置: ($target_x, $target_y)"
    
    # 计算距离
    if [ -n "$pos_x" ] && [ -n "$target_x" ]; then
        dist=$(echo "scale=3; sqrt(($target_x - $pos_x)^2 + ($target_y - $pos_y)^2)" | bc -l)
        echo "📏 距离目标: ${dist}m"
    fi
    
    echo ""
    echo "────────────────────────────────────────────────────────────"
    
    # 获取速度指令
    cmd_vel=$(timeout 0.5 ros2 topic echo /cmd_vel --once 2>/dev/null)
    if [ -n "$cmd_vel" ]; then
        linear_x=$(echo "$cmd_vel" | grep "x:" | head -1 | awk '{print $2}')
        angular_z=$(echo "$cmd_vel" | grep "z:" | tail -1 | awk '{print $2}')
        echo "🚗 线速度: ${linear_x} m/s"
        echo "🔄 角速度: ${angular_z} rad/s"
    else
        echo "🚗 速度指令: (数据获取中...)"
    fi
    
    echo ""
    echo "────────────────────────────────────────────────────────────"
    
    # 获取最新日志（最后一行）
    latest_log=$(tail -1 ~/dev_ws/log/competition_stage1/latest.log 2>/dev/null)
    if [ -n "$latest_log" ]; then
        echo "📋 最新日志:"
        echo "   $latest_log"
    fi
    
    echo ""
    echo "════════════════════════════════════════════════════════════"
    echo "更新时间: $(date '+%Y-%m-%d %H:%M:%S')"
    
    sleep 2
done
