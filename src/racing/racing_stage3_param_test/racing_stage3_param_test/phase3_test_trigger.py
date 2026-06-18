"""Publish competition_phase=3 once (param test without full competition stack)."""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class Phase3TestTrigger(Node):
    def __init__(self):
        super().__init__('phase3_test_trigger')
        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('phase_value', 3)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_period_sec', 0.2)

        topic = self.get_parameter('phase_topic').value
        self.phase_value = int(self.get_parameter('phase_value').value)
        self.publish_count = max(1, int(self.get_parameter('publish_count').value))
        self.publish_period_sec = float(self.get_parameter('publish_period_sec').value)

        self.publisher = self.create_publisher(Int32, topic, 10)
        self.timer = self.create_timer(0.5, self.publish_once)
        self.published = 0

    def publish_once(self):
        if self.published >= self.publish_count:
            self.timer.cancel()
            return

        msg = Int32()
        msg.data = self.phase_value
        self.publisher.publish(msg)
        self.published += 1
        if self.published >= self.publish_count:
            self.get_logger().info(
                f'published competition_phase={self.phase_value} for stage3 param test'
            )
            self.timer.cancel()


def main(args=None):
    rclpy.init(args=args)
    node = Phase3TestTrigger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
