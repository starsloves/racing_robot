import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistCmdRelay(Node):
    def __init__(self):
        super().__init__('stage2_test_cmd_relay')

        self.declare_parameter('input_topic', '/stage2_cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')
        self.declare_parameter('cmd_watchdog_timeout_sec', 0.25)
        self.declare_parameter('cmd_stop_burst_count', 10)

        self.input_topic = str(self.get_parameter('input_topic').value).strip() or '/stage2_cmd_vel'
        self.output_topic = str(self.get_parameter('output_topic').value).strip() or '/cmd_vel'
        self._cmd_watchdog_timeout_sec = max(
            0.10,
            float(self.get_parameter('cmd_watchdog_timeout_sec').value),
        )
        self._cmd_stop_burst_count = max(
            1,
            int(self.get_parameter('cmd_stop_burst_count').value),
        )

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)
        self._last_input_mono = self.get_clock().now().nanoseconds / 1e9
        self._forwarding_enabled = True
        self._watchdog_stop_latched = False
        self.create_timer(0.05, self._watchdog_timer_callback)
        self.publish_stop_burst('relay_startup')
        self.get_logger().info(
            f'阶段二测试速度中继已启动: {self.input_topic} -> {self.output_topic}，'
            f'看门狗={self._cmd_watchdog_timeout_sec:.2f}s'
        )

    def cmd_callback(self, msg):
        self._last_input_mono = self.get_clock().now().nanoseconds / 1e9
        self._watchdog_stop_latched = False
        if self._forwarding_enabled:
            self.publisher.publish(msg)

    def publish_stop_burst(self, reason='stop'):
        if not rclpy.ok():
            return
        stop = Twist()
        for _ in range(self._cmd_stop_burst_count):
            self.publisher.publish(stop)
        if reason not in ('relay_startup',):
            self.get_logger().warning(f'速度中继紧急停车: {reason}')

    def _watchdog_timer_callback(self):
        now_mono = self.get_clock().now().nanoseconds / 1e9
        if now_mono - self._last_input_mono > self._cmd_watchdog_timeout_sec:
            if not self._watchdog_stop_latched:
                self._watchdog_stop_latched = True
                self.publish_stop_burst('input_timeout')
            else:
                self.publisher.publish(Twist())
        else:
            self._watchdog_stop_latched = False

    def destroy_node(self):
        self._forwarding_enabled = False
        if rclpy.ok():
            self.publish_stop_burst('relay_destroy')
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TwistCmdRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node._forwarding_enabled = False
            node.publish_stop_burst('main_finally')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
