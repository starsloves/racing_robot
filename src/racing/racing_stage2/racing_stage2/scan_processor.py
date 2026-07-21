"""scan_processor.py — 激光雷达扫描处理，输出前/左/右障碍距离和角度。"""

import math
from dataclasses import dataclass
from sensor_msgs.msg import LaserScan


@dataclass
class ScanData:
    front_distance: float
    front_angle_deg: float
    left_clearance: float
    left_angle_deg: float
    right_clearance: float
    right_angle_deg: float


class ScanProcessor:
    def __init__(self, front_angle_deg=18.0, side_window_deg=30.0, side_center_deg=65.0):
        self.front_angle_deg = abs(front_angle_deg)
        self.side_window_deg = abs(side_window_deg)
        self.side_center_deg = abs(side_center_deg)

    def process(self, msg: LaserScan) -> ScanData:
        front_dist, front_angle = self._sector_closest_obstacle(
            msg, -self.front_angle_deg, self.front_angle_deg
        )
        half = self.side_window_deg / 2.0
        left_dist, left_angle = self._sector_min_distance(
            msg, self.side_center_deg - half, self.side_center_deg + half
        )
        right_dist, right_angle = self._sector_min_distance(
            msg, -self.side_center_deg - half, -self.side_center_deg + half
        )
        return ScanData(
            front_distance=front_dist,
            front_angle_deg=front_angle,
            left_clearance=left_dist,
            left_angle_deg=left_angle,
            right_clearance=right_dist,
            right_angle_deg=right_angle,
        )

    def _sector_closest_obstacle(self, scan_msg, min_angle_deg, max_angle_deg):
        min_distance = float('inf')
        min_angle = 0.0
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue
            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
                continue
            if distance < min_distance:
                min_distance = distance
                min_angle = angle_deg
        return min_distance, min_angle

    def _sector_min_distance(self, scan_msg, min_angle_deg, max_angle_deg):
        """返回扇形内最近距离及对应角度"""
        min_distance = float('inf')
        min_angle = 0.0
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue
            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
                continue
            if distance < min_distance:
                min_distance = distance
                min_angle = angle_deg
        return min_distance, min_angle
    
    def cluster_obstacles_in_window(self, scan_msg, min_x, max_x, half_y, gap_tolerance=0.12):
        """聚类窗口内的障碍物点云（复用 Stage1 逻辑）
        
        将激光扫描中的点按照空间距离聚类，用于障碍物识别和可视化。
        
        Args:
            scan_msg: LaserScan 消息
            min_x: 窗口最小 X 坐标（车体前方，m）
            max_x: 窗口最大 X 坐标（m）
            half_y: 窗口左右半宽（m）
            gap_tolerance: 聚类间隙容差（m），相邻点距离超过此值则分为不同聚类
        
        Returns:
            聚类列表 [[(x,y,dist), ...], ...]
        """
        # 参数校验
        if min_x >= max_x:
            return []
        if half_y <= 0.0:
            return []
        if gap_tolerance <= 0.0:
            gap_tolerance = 0.12
        
        clusters = []
        current_cluster = []
        previous_point = None
        
        for index, distance in enumerate(scan_msg.ranges):
            # 过滤无效点
            if math.isinf(distance) or math.isnan(distance) or distance < 0.15:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue
            
            # 转换为笛卡尔坐标
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            
            # 检查是否在窗口内
            if x < min_x or x > max_x or abs(y) > half_y:
                if current_cluster:
                    clusters.append(current_cluster)
                    current_cluster = []
                previous_point = None
                continue
            
            point = (x, y, distance)
            
            # 聚类逻辑：相邻点距离 <= gap_tolerance 则属于同一聚类
            if previous_point is None:
                current_cluster.append(point)
            else:
                point_dist = math.hypot(x - previous_point[0], y - previous_point[1])
                if point_dist <= gap_tolerance:
                    current_cluster.append(point)
                else:
                    if current_cluster:
                        clusters.append(current_cluster)
                    current_cluster = [point]
            
            previous_point = point
        
        # 添加最后一个聚类
        if current_cluster:
            clusters.append(current_cluster)
        
        return clusters

    def nearest_filtered_cluster(self, scan_msg, *, min_x, max_x, min_y, max_y,
                               gap_tolerance, min_points, min_width, max_width,
                               min_valid_range=0.15):
        """Return the nearest Stage1-style valid cluster in a rectangular window."""
        clusters = []
        current_cluster = []
        previous_point = None
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance < min_valid_range:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = []
                previous_point = None
                continue
            angle = scan_msg.angle_min + index * scan_msg.angle_increment
            x = distance * math.cos(angle)
            y = distance * math.sin(angle)
            if x < min_x or x > max_x or y < min_y or y > max_y:
                if current_cluster:
                    clusters.append(current_cluster)
                current_cluster = []
                previous_point = None
                continue
            point = (x, y, distance)
            if previous_point is None or math.hypot(
                    x - previous_point[0], y - previous_point[1]) <= gap_tolerance:
                current_cluster.append(point)
            else:
                clusters.append(current_cluster)
                current_cluster = [point]
            previous_point = point
        if current_cluster:
            clusters.append(current_cluster)

        nearest = None
        for cluster in clusters:
            if len(cluster) < min_points:
                continue
            span = math.hypot(cluster[-1][0] - cluster[0][0],
                              cluster[-1][1] - cluster[0][1])
            if span < min_width or span > max_width:
                continue
            center_x = sum(point[0] for point in cluster) / len(cluster)
            center_y = sum(point[1] for point in cluster) / len(cluster)
            candidate = {
                'distance': min(point[2] for point in cluster),
                'center_x': center_x,
                'center_y': center_y,
                'span': span,
                'danger_angle_deg': math.degrees(math.atan2(center_y, max(center_x, 1e-6))),
            }
            if nearest is None or candidate['distance'] < nearest['distance']:
                nearest = candidate
        return nearest
