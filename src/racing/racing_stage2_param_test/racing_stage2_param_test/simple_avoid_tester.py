"""极简直行避障测试节点

功能：
- 启动后从当前位置/航向直行 3 米
- 检测到障碍 → 调用 SpiralAvoider 避障
- 避障完成 → 恢复直行
- 到达目标 → 停止

特点：
- 继承 DirectInertialTester（racing_stage2_param_test 版本）
- 100% 复用避障逻辑（SpiralAvoider）
- 100% 复用避障参数（avoidance_config.yaml）
- 只覆盖赛道规划方法（单段直行）
- 详细日志（0.1 秒遥测间隔）
"""

import rclpy
import threading

from racing_stage2_param_test.direct_inertial_tester import DirectInertialTester
from racing_stage2_param_test.cmd_vel_stop import (
    init_without_ros_signal_handler,
    install_stop_event,
    publish_stop,
    spin_until_stop,
)


class SimpleAvoidTester(DirectInertialTester):
    """极简直行避障测试节点
    
    继承 DirectInertialTester，获得：
    - SpiralAvoider 避障模块
    - ScanProcessor 激光雷达处理
    - RacingLogger 日志系统
    - 所有避障参数（从 avoidance_config.yaml 加载）
    - 控制循环（20 Hz）
    
    只覆盖：
    - build_inertial_plan() → 返回单段直行 3m
    - build_ring_plan() → 返回空列表
    - rectangle_segment_label() → 段标签
    - _telemetry_interval_sec → 0.10 秒（更详细日志）
    """
    
    def __init__(self):
        super().__init__()  # 继承所有功能
        
        # 只覆盖遥测间隔（从默认 0.25s 改为 0.10s）
        self._telemetry_interval_sec = 0.10
        
        self.logger.info(
            'STARTUP',
            '极简直行避障测试：直行 3m（从当前位置/航向启动，速度 0.2m/s）'
        )
    
    def build_inertial_plan(self, nav_succeeded):
        """覆盖父类方法 —— 返回单段直行 3 米
        
        Args:
            nav_succeeded: 导航成功标志（不使用）
        
        Returns:
            list: 单段直行计划
        """
        del nav_succeeded  # 不需要
        return [
            {
                'type': 'move',
                'distance_m': 3.0,
                'speed': self.ring_linear_speed,  # 用环道速度（0.5 m/s）
                'description': 'simple_straight_test',
            }
        ]
    
    def build_ring_plan(self):
        """覆盖父类方法 —— 不需要环形赛道
        
        Returns:
            list: 空列表
        """
        return []
    
    def rectangle_segment_label(self, segment):
        """覆盖段标签（日志显示用）
        
        Args:
            segment: 段信息字典
        
        Returns:
            str: 段标签
        """
        desc = str((segment or {}).get('description', 'unknown'))
        if desc == 'simple_straight_test':
            return '直行测试 3.00m'
        return super().rectangle_segment_label(segment)


def main(args=None):
    """主函数 —— 复用 DirectInertialTester 的启动逻辑"""
    init_without_ros_signal_handler(args)
    node = SimpleAvoidTester()
    stop_event = threading.Event()
    
    def request_stop():
        stop_event.set()
        publish_stop(node.cmd_pub)
    
    node._request_stop = request_stop
    
    install_stop_event(
        stop_event,
        lambda: publish_stop(node.cmd_pub),
        cli_topics=['/cmd_vel', '/stage2_cmd_vel']
    )
    
    try:
        spin_until_stop(node, stop_event)
    except KeyboardInterrupt:
        request_stop()
    finally:
        request_stop()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
