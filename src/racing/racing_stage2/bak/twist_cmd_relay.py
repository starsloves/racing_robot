import threading

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from racing_stage2.cmd_vel_stop import (
    init_without_ros_signal_handler,
    install_stop_event,
    publish_stop,
    spin_until_stop,
)


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
            f'阶段二测试速度中继已启�? {self.input_topic} -> {self.output_topic}'
        )

    def cmd_callback(self, msg):
        self.publisher.publish(msg)

    def publish_stop_now(self):
        publish_stop(self.publisher)


def main(args=None):
    init_without_ros_signal_handler(args)
    node = TwistCmdRelay()
    stop_event = threading.Event()
    cli_topics = [node.output_topic, node.input_topic]

    request_stop = install_stop_event(
        stop_event,
        node.publish_stop_now,
        cli_topics=cli_topics,
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
