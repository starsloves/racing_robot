import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


class LidarObstacleCircleMarkers(Node):
    def __init__(self):
        super().__init__('lidar_obstacle_circle_markers')

        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('marker_topic', 'obstacle_circle_markers')
        self.declare_parameter('cluster_distance_threshold', 0.18)
        self.declare_parameter('min_cluster_points', 3)
        self.declare_parameter('min_range', 0.10)
        self.declare_parameter('max_range', 2.50)
        self.declare_parameter('obstacle_padding', 0.04)
        self.declare_parameter('min_obstacle_diameter', 0.33)
        self.declare_parameter('max_circle_radius', 0.50)
        self.declare_parameter('max_cluster_span', 0.90)
        self.declare_parameter('marker_height', 0.05)
        self.declare_parameter('color_r', 1.0)
        self.declare_parameter('color_g', 1.0)
        self.declare_parameter('color_b', 1.0)
        self.declare_parameter('color_a', 0.90)

        self.scan_topic = self.get_parameter('scan_topic').value
        self.marker_topic = self.get_parameter('marker_topic').value
        self.cluster_distance_threshold = float(self.get_parameter('cluster_distance_threshold').value)
        self.min_cluster_points = int(self.get_parameter('min_cluster_points').value)
        self.min_range = float(self.get_parameter('min_range').value)
        self.max_range = float(self.get_parameter('max_range').value)
        self.obstacle_padding = float(self.get_parameter('obstacle_padding').value)
        self.min_obstacle_diameter = float(self.get_parameter('min_obstacle_diameter').value)
        self.max_circle_radius = float(self.get_parameter('max_circle_radius').value)
        self.max_cluster_span = float(self.get_parameter('max_cluster_span').value)
        self.marker_height = float(self.get_parameter('marker_height').value)
        self.color_r = float(self.get_parameter('color_r').value)
        self.color_g = float(self.get_parameter('color_g').value)
        self.color_b = float(self.get_parameter('color_b').value)
        self.color_a = float(self.get_parameter('color_a').value)

        self.last_marker_count = 0
        self.marker_pub = self.create_publisher(MarkerArray, self.marker_topic, 10)
        self.create_subscription(LaserScan, self.scan_topic, self.scan_callback, 10)

        self.get_logger().info('lidar obstacle circle markers ready')

    def scan_callback(self, msg):
        clusters = self.cluster_scan_points(msg)
        circles = self.build_circles(clusters)
        self.publish_markers(msg, circles)

    def cluster_scan_points(self, scan_msg):
        clusters = []
        current_cluster = []
        previous_point = None

        for index, distance in enumerate(scan_msg.ranges):
            if math.isnan(distance) or math.isinf(distance) or distance < self.min_range or distance > self.max_range:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue

            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            point = (distance * math.cos(angle), distance * math.sin(angle))

            if previous_point is None:
                current_cluster = [point]
            elif self.point_distance(previous_point, point) <= self.cluster_distance_threshold:
                current_cluster.append(point)
            else:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = [point]

            previous_point = point

        if current_cluster:
            clusters.append(current_cluster)

        return clusters

    def build_circles(self, clusters):
        circles = []
        min_radius = max(0.0, self.min_obstacle_diameter / 2.0)
        for cluster in clusters:
            if len(cluster) < self.min_cluster_points:
                continue

            span = self.point_distance(cluster[0], cluster[-1])
            if span > self.max_cluster_span:
                continue

            center_x = sum(point[0] for point in cluster) / len(cluster)
            center_y = sum(point[1] for point in cluster) / len(cluster)
            radius = max(self.point_distance((center_x, center_y), point) for point in cluster)
            radius = max(radius + self.obstacle_padding, min_radius)

            if radius <= 0.0 or radius > self.max_circle_radius:
                continue

            circles.append((center_x, center_y, radius))

        return circles

    def publish_markers(self, scan_msg, circles):
        marker_array = MarkerArray()
        frame_id = scan_msg.header.frame_id
        stamp = scan_msg.header.stamp

        for marker_id, (center_x, center_y, radius) in enumerate(circles):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'lidar_obstacle_circles'
            marker.id = marker_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD
            marker.pose.position.x = float(center_x)
            marker.pose.position.y = float(center_y)
            marker.pose.position.z = self.marker_height / 2.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = radius * 2.0
            marker.scale.y = radius * 2.0
            marker.scale.z = self.marker_height
            marker.color.r = self.color_r
            marker.color.g = self.color_g
            marker.color.b = self.color_b
            marker.color.a = self.color_a
            marker_array.markers.append(marker)

        for marker_id in range(len(circles), self.last_marker_count):
            marker = Marker()
            marker.header.frame_id = frame_id
            marker.header.stamp = stamp
            marker.ns = 'lidar_obstacle_circles'
            marker.id = marker_id
            marker.action = Marker.DELETE
            marker_array.markers.append(marker)

        self.last_marker_count = len(circles)
        self.marker_pub.publish(marker_array)

    def point_distance(self, point_a, point_b):
        dx = point_a[0] - point_b[0]
        dy = point_a[1] - point_b[1]
        return math.hypot(dx, dy)


def main(args=None):
    rclpy.init(args=args)
    node = LidarObstacleCircleMarkers()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()