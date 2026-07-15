#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32, String


class StageTestPublisher(Node):
    def __init__(self):
        super().__init__('stage_test_publisher')
        
        self.declare_parameter('stage_number', 3)
        self.declare_parameter('test_direction', 'clockwise')
        
        stage = self.get_parameter('stage_number').value
        direction = self.get_parameter('test_direction').value
        
        event_qos = QoSProfile(depth=1)
        event_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        event_qos.reliability = ReliabilityPolicy.RELIABLE
        
        self.phase_pub = self.create_publisher(Int32, '/competition_phase', event_qos)
        self.task_pub = self.create_publisher(String, '/competition_qr_task', event_qos)
        
        self.timer = self.create_timer(0.5, self.publish_topics)
        
        self.phase_msg = Int32()
        self.phase_msg.data = stage
        
        self.task_msg = String()
        self.task_msg.data = direction
        
        self.get_logger().info(f'Stage {stage} test publisher started (FIXED QoS), direction: {direction}')
    
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