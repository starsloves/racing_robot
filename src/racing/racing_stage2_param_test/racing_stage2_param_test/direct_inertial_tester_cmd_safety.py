"""Command velocity watchdog and emergency stop for direct_inertial_tester."""

from geometry_msgs.msg import Twist


class DirectInertialTesterCmdSafetyMixin:
    """Ensure /stage2_cmd_vel goes to zero when the node hangs or shuts down."""

    def init_cmd_vel_safety(self):
        self.declare_parameter('cmd_vel_watchdog_timeout_sec', 0.30)
        self.declare_parameter('cmd_vel_stop_burst_count', 8)
        self._cmd_vel_watchdog_timeout_sec = max(
            0.10,
            float(self.get_parameter('cmd_vel_watchdog_timeout_sec').value),
        )
        self._cmd_vel_stop_burst_count = max(
            1,
            int(self.get_parameter('cmd_vel_stop_burst_count').value),
        )
        self._last_cmd_vel_publish_mono = self.get_clock().now().nanoseconds / 1e9
        self._cmd_vel_safety_latched = False
        self._cmd_vel_watchdog_fault_logged = False
        self.create_timer(0.05, self._cmd_vel_watchdog_callback)
        self.publish_cmd_vel()

    def publish_cmd_vel(self, linear_x=0.0, angular_z=0.0):
        if self._cmd_vel_safety_latched:
            linear_x = 0.0
            angular_z = 0.0
        self.cmd_pub.publish(self.create_twist(linear_x, angular_z))
        self._last_cmd_vel_publish_mono = self.get_clock().now().nanoseconds / 1e9
        self._cmd_vel_watchdog_fault_logged = False

    def publish_emergency_stop(self, reason='estop'):
        self._cmd_vel_safety_latched = True
        stop = self.create_twist()
        for _ in range(self._cmd_vel_stop_burst_count):
            self.cmd_pub.publish(stop)
        self._last_cmd_vel_publish_mono = self.get_clock().now().nanoseconds / 1e9
        self.get_logger().warning(f'紧急停车: {reason}')

    def publish_watchdog_zero_hold(self, reason='cmd_vel_watchdog_timeout'):
        """Republish stop without latching — recover when control_loop resumes publish_cmd_vel."""
        if not self._cmd_vel_watchdog_fault_logged:
            self._cmd_vel_watchdog_fault_logged = True
            self.get_logger().warning(
                f'速度看门狗: {reason}（未锁死，控制恢复后可继续运动）'
            )
        self.publish_cmd_vel()

    def _cmd_vel_watchdog_callback(self):
        if self._cmd_vel_safety_latched:
            self.cmd_pub.publish(self.create_twist())
            return
        now_mono = self.get_clock().now().nanoseconds / 1e9
        if now_mono - self._last_cmd_vel_publish_mono > self._cmd_vel_watchdog_timeout_sec:
            self.publish_watchdog_zero_hold('cmd_vel_watchdog_timeout')

    def destroy_node(self):
        try:
            self.publish_emergency_stop('node_destroy')
        except Exception:
            pass
        super().destroy_node()
