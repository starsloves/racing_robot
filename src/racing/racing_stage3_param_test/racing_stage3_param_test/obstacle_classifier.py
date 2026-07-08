"""障碍物分类器：区分锥桶（小障碍）和墙壁（大障碍）"""

import math


class ObstacleClassifier:
    """基于聚类宽度和距离分类障碍物类型"""

    def __init__(self, cfg: dict):
        self.cone_max_width = float(cfg.get('cone_max_width', 0.15))
        self.cone_max_distance = float(cfg.get('cone_max_distance', 0.80))
        self.wall_min_width = float(cfg.get('wall_min_width', 0.30))
        self.wall_detect_distance = float(cfg.get('wall_detect_distance', 2.5))
        self.min_cluster_points = int(cfg.get('phase1_min_cluster_points', 3))
        self.min_valid_range = float(cfg.get('min_valid_range', 0.15))
        self.cluster_gap = 0.12

    def classify(self, cluster):
        """分类障碍物 → 'cone' / 'wall' / 'none'"""
        if len(cluster) < self.min_cluster_points:
            return 'none'

        span = math.hypot(cluster[-1][0] - cluster[0][0],
                          cluster[-1][1] - cluster[0][1])
        nearest = min(p[2] for p in cluster)

        if span < self.cone_max_width and nearest < self.cone_max_distance:
            return 'cone'

        if span > self.wall_min_width and nearest < self.wall_detect_distance:
            return 'wall'

        return 'none'

    def get_cluster_center(self, cluster):
        """返回聚类中心 (center_x, center_y, nearest_distance, danger_angle_deg)"""
        nearest = min(p[2] for p in cluster)
        cx = sum(p[0] for p in cluster) / len(cluster)
        cy = sum(p[1] for p in cluster) / len(cluster)
        danger_angle = math.degrees(math.atan2(cy, max(cx, 1e-6)))
        return cx, cy, nearest, danger_angle

    def collect_clusters(self, scan_msg, min_x, max_x, half_width):
        """在指定窗口内提取聚类（复用 Stage1 算法）"""
        clusters = []
        current = []
        prev = None

        for i, d in enumerate(scan_msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < self.min_valid_range:
                if current:
                    clusters.append(current)
                    current = []
                prev = None
                continue

            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = d * math.cos(angle)
            y = d * math.sin(angle)

            if x < min_x or x > max_x or abs(y) > half_width:
                if current:
                    clusters.append(current)
                    current = []
                prev = None
                continue

            pt = (x, y, d)
            if prev is None or math.hypot(prev[0] - pt[0], prev[1] - pt[1]) <= self.cluster_gap:
                current.append(pt)
            else:
                if current:
                    clusters.append(current)
                current = [pt]
            prev = pt

        if current:
            clusters.append(current)

        return clusters

    def find_nearest_cone(self, scan_msg, min_x=0.18, max_x=0.85, half_width=0.22):
        """找最近的锥桶 → (cx, cy, dist) 或 None"""
        clusters = self.collect_clusters(scan_msg, min_x, max_x, half_width)
        best = None
        best_dist = float('inf')

        for cluster in clusters:
            typ = self.classify(cluster)
            if typ != 'cone':
                continue

            nearest = min(p[2] for p in cluster)
            if nearest < best_dist:
                cx = sum(p[0] for p in cluster) / len(cluster)
                cy = sum(p[1] for p in cluster) / len(cluster)
                best = (cx, cy, nearest)
                best_dist = nearest

        return best