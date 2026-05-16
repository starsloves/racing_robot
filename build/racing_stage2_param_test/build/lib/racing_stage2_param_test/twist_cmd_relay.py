import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


class TwistCmdRelay(Node):
    def __init__(self):
        super().__init__('stage2_test_cmd_relay')

        self.declare_parameter('input_topic', '/stage2_cmd_vel')
        self.declare_parameter('output_topic', '/cmd_vel')

        self.input_topic = str(self.get_parameter('input_topic').value).strip() or '/stage2_cmd_vel'
        self.output_topic = str(self.get_parameter('output_topic').value).strip() or '/cmd_vel'

        self.publisher = self.create_publisher(Twist, self.output_topic, 10)
        self.create_subscription(Twist, self.input_topic, self.cmd_callback, 10)
        self.get_logger().info(
            f'阶段二测试速度中继已启动: {self.input_topic} -> {self.output_topic}'
        )

    def cmd_callback(self, msg):
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TwistCmdRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.publisher.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()