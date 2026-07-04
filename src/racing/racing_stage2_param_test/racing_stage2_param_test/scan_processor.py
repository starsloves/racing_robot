"""scan_processor.py — 激光雷达扫描处理，输出前/左/右障碍距离。"""

import math
from dataclasses import dataclass
from sensor_msgs.msg import LaserScan


@dataclass
class ScanData:
    front_distance: float
    front_angle_deg: float
    left_clearance: float
    right_clearance: float


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
        left = self._sector_min_distance(
            msg, self.side_center_deg - half, self.side_center_deg + half
        )
        right = self._sector_min_distance(
            msg, -self.side_center_deg - half, -self.side_center_deg + half
        )
        return ScanData(
            front_distance=front_dist,
            front_angle_deg=front_angle,
            left_clearance=left,
            right_clearance=right,
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
        min_distance = float('inf')
        for index, distance in enumerate(scan_msg.ranges):
            if math.isinf(distance) or math.isnan(distance) or distance <= 0.0:
                continue
            angle_deg = math.degrees(scan_msg.angle_min + index * scan_msg.angle_increment)
            angle_deg = (angle_deg + 180.0) % 360.0 - 180.0
            if angle_deg < min_angle_deg or angle_deg > max_angle_deg:
                continue
            if distance < min_distance:
                min_distance = distance
        return min_distance
