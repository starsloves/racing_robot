#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


class StageTestPublisher(Node):
    def __init__(self):
        super().__init__('stage_test_publisher')
        
        self.declare_parameter('stage_number', 2)
        self.declare_parameter('test_direction', 'clockwise')
        self.declare_parameter('phase_topic', '/stage2_test/competition_phase')
        self.declare_parameter('task_topic', '/stage2_test/competition_qr_task')
        
        stage = self.get_parameter('stage_number').value
        direction = self.get_parameter('test_direction').value
        phase_topic = str(self.get_parameter('phase_topic').value).strip() or '/stage2_test/competition_phase'
        task_topic = str(self.get_parameter('task_topic').value).strip() or '/stage2_test/competition_qr_task'
        
        event_qos = QoSProfile(depth=1)
        event_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        
        self.phase_pub = self.create_publisher(Int32, phase_topic, event_qos)
        self.task_pub = self.create_publisher(String, task_topic, event_qos)
        
        self.timer = self.create_timer(0.5, self.publish_topics)
        
        self.phase_msg = Int32()
        self.phase_msg.data = stage
        
        self.task_msg = String()
        self.task_msg.data = direction
        
        self.get_logger().info(
            f'Stage {stage} test publisher started (FIXED QoS), direction: {direction}, '
            f'phase_topic={phase_topic}, task_topic={task_topic}'
        )
    
    def publish_topics(self):
        self.phase_pub.publish(self.phase_msg)
        self.task_pub.publish(self.task_msg)


def main(args=None):
    rclpy.init(args=args)
    node = StageTestPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
