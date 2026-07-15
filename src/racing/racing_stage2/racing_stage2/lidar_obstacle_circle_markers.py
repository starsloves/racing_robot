"""lidar_obstacle_circle_markers.py - Lidar障碍物圆圈可视化"""
import rclpy
from rclpy.node import Node
from racing_common.obstacle_marker_publisher import ObstacleMarkerPublisher

def main(args=None):
    rclpy.init(args=args)
    
    node_instance = Node('lidar_obstacle_circle_markers')
    marker_publisher = ObstacleMarkerPublisher(node_instance)
    
    try:
        rclpy.spin(node_instance)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node_instance.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()
