"""Stage3 standalone test simulator - simulates Stage2 ending state so Stage3 can be tested independently.

Simulated content:
  1. Publishes competition_phase=3 (same as phase3_test_trigger)
  2. Optional: publishes competition_qr_task to simulate Stage1 QR scan direction
  3. Optional: publishes initial pose hint after startup
  4. Controls startup timing
"""

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


class Stage3TestSimulator(Node):
    """Simulates complete environment for Stage2->Stage3 phase transition."""

    def __init__(self):
        super().__init__('stage3_test_simulator')

        self.declare_parameter('phase_topic', 'competition_phase')
        self.declare_parameter('task_topic', 'competition_qr_task')
        self.declare_parameter('phase_value', 3)
        self.declare_parameter('publish_count', 5)
        self.declare_parameter('publish_period_sec', 0.2)
        self.declare_parameter('start_delay_sec', 1.0)
        self.declare_parameter('simulate_qr_task', True)
        self.declare_parameter('qr_task_value', 'clockwise')

        phase_topic = str(self.get_parameter('phase_topic').value)
        task_topic = str(self.get_parameter('task_topic').value)
        phase_value = int(self.get_parameter('phase_value').value)
        self.publish_count = max(1, int(self.get_parameter('publish_count').value))
        self.publish_period = float(self.get_parameter('publish_period_sec').value)
        start_delay = float(self.get_parameter('start_delay_sec').value)
        sim_qr = bool(self.get_parameter('simulate_qr_task').value)
        qr_val = str(self.get_parameter('qr_task_value').value)

        qos_latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE)
        self._phase_pub = self.create_publisher(Int32, phase_topic, qos_latched)
        if sim_qr:
            self._task_pub = self.create_publisher(String, task_topic, qos_latched)

        self._phase_value = phase_value
        self._qr_val = qr_val
        self._sim_qr = sim_qr
        self._published = 0

        self._timer = self.create_timer(start_delay, self._publish_once)
        self.get_logger().info(
            f'stage3 simulator ready | '
            f'phase={phase_value} qr={sim_qr} value={qr_val} '
            f'delay={start_delay}s count={self.publish_count}'
        )

    def _publish_once(self):
        if self._published == 0 and self._sim_qr:
            qr_msg = String()
            qr_msg.data = self._qr_val
            self._task_pub.publish(qr_msg)
            self.get_logger().info(f'published qr_task={qr_msg.data} (simulated Stage2 direction)')

        if self._published < self.publish_count:
            msg = Int32()
            msg.data = self._phase_value
            self._phase_pub.publish(msg)
            self._published += 1

        if self._published >= self.publish_count:
            self.get_logger().info(
                f'simulator done: phase={self._phase_value} published {self._published}x'
            )
            try:
                self.destroy_timer(self._timer)
            except Exception:
                pass

    def publish_once(self):
        if hasattr(self, 'timer') and self.timer is not None:
            self._publish_once()

    def destroy_node(self):
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stage3TestSimulator()
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
