"""墙角检测模块：雷达点云 → Douglas-Peucker 简化 → 最小二乘直线拟合 → 90° 夹角检测"""

import math
import time


class CornerDetector:
    """检测 90° 墙角（两条墙壁形成的夹角）"""

    def __init__(self, cfg: dict):
        self.epsilon = float(cfg.get('douglas_peucker_epsilon', 0.05))
        self.min_line_length = float(cfg.get('corner_min_line_length', 0.30))
        self.angle_tolerance = math.radians(float(cfg.get('corner_angle_tolerance_deg', 15.0)))
        self.min_distance = float(cfg.get('corner_min_distance', 0.30))
        self.max_distance = float(cfg.get('corner_max_distance', 2.5))
        self.min_points = int(cfg.get('corner_min_points', 8))
        self.min_valid_range = float(cfg.get('min_valid_range', 0.15))

        # 检测历史（用于双重确认）
        self._history = []

    def detect(self, scan_msg):
        """扫描 → 返回 (detected, corner_x, corner_y, confidence)

        所有坐标在雷达坐标系（车体前方 +X，左侧 +Y）
        """
        points = self._extract_points(scan_msg)
        segments = self._segment_by_gap(points, gap=0.15)
        lines = self._fit_lines(segments)
        detected, cx, cy, conf = self._find_corner(lines)

        now = time.time()
        if detected:
            self._history.append((cx, cy, conf, now))
        self._history = [h for h in self._history if now - h[3] < 0.6]

        return detected, cx, cy, conf

    def get_confirmed_corner(self, confirmation_frames=3, position_tolerance=0.15):
        """双重确认：连续 N 帧 + 位置一致 → 返回 (cx, cy) 或 None"""
        if len(self._history) < confirmation_frames:
            return None

        recent = self._history[-confirmation_frames:]
        positions = [(p[0], p[1]) for p in recent]

        for i in range(len(positions) - 1):
            d = math.hypot(positions[i + 1][0] - positions[i][0],
                           positions[i + 1][1] - positions[i][1])
            if d > position_tolerance:
                return None

        avg_x = sum(p[0] for p in positions) / len(positions)
        avg_y = sum(p[1] for p in positions) / len(positions)
        return avg_x, avg_y

    def _extract_points(self, scan_msg):
        """提取有效点云 (x, y, angle, range)，车体坐标系"""
        points = []
        for i, d in enumerate(scan_msg.ranges):
            if math.isinf(d) or math.isnan(d) or d < self.min_valid_range or d > self.max_distance:
                continue
            angle = scan_msg.angle_min + i * scan_msg.angle_increment
            x = d * math.cos(angle)
            y = d * math.sin(angle)
            points.append((x, y, angle, d))
        return points

    def _segment_by_gap(self, points, gap):
        """按角度排序后，间距 > gap 切分"""
        if not points:
            return []
        sorted_pts = sorted(points, key=lambda p: p[2])
        segments, seg = [], [sorted_pts[0]]
        for i in range(1, len(sorted_pts)):
            dist = math.hypot(sorted_pts[i][0] - sorted_pts[i - 1][0],
                              sorted_pts[i][1] - sorted_pts[i - 1][1])
            if dist > gap:
                segments.append(seg)
                seg = [sorted_pts[i]]
            else:
                seg.append(sorted_pts[i])
        if seg:
            segments.append(seg)
        return segments

    def _simplify(self, points, epsilon):
        """Douglas-Peucker 点云简化"""
        if len(points) < 3:
            return points
        max_dist, max_i = 0.0, 0
        x0, y0 = points[0]
        x1, y1 = points[-1]
        for i in range(1, len(points) - 1):
            dist = abs((y1 - y0) * points[i][0] - (x1 - x0) * points[i][1]
                       + x1 * y0 - y1 * x0) / max(math.hypot(x1 - x0, y1 - y0), 1e-6)
            if dist > max_dist:
                max_dist, max_i = dist, i
        if max_dist > epsilon:
            left = self._simplify(points[:max_i + 1], epsilon)
            right = self._simplify(points[max_i:], epsilon)
            return left[:-1] + right
        return [points[0], points[-1]]

    def _fit_line(self, points):
        """最小二乘直线拟合 → 返回 dict 或 None"""
        xs = [p[0] for p in points]
        ys = [p[1] for p in points]
        n = len(xs)
        if n < 2:
            return None

        sx = sum(xs)
        sy = sum(ys)
        sxx = sum(x * x for x in xs)
        sxy = sum(xs[i] * ys[i] for i in range(n))

        denom = n * sxx - sx * sx
        if abs(denom) < 1e-8:
            # 垂直线
            if n < 2:
                return None
            start = (xs[0], ys[0])
            end = (xs[-1], ys[-1])
            angle = math.pi / 2
        else:
            k = (n * sxy - sx * sy) / denom
            b = (sy - k * sx) / n
            start = (xs[0], k * xs[0] + b)
            end = (xs[-1], k * xs[-1] + b)
            angle = math.atan(k)

        length = math.hypot(end[0] - start[0], end[1] - start[1])

        return {
            'start': start,
            'end': end,
            'angle': self._normalize_angle(angle),
            'length': length,
            'points': points,
        }

    def _fit_lines(self, segments):
        """多段拟合 → 返回长直线列表"""
        lines = []
        for seg in segments:
            pts = [(p[0], p[1]) for p in seg]
            if len(pts) < self.min_points:
                continue
            simplified = self._simplify(pts, self.epsilon)
            line = self._fit_line(simplified if len(simplified) >= 2 else pts)
            if line and line['length'] >= self.min_line_length:
                lines.append(line)
        return lines

    def _line_intersection(self, l1, l2):
        """两直线交点（xy 坐标系）"""
        dx1 = l1['end'][0] - l1['start'][0]
        dy1 = l1['end'][1] - l1['start'][1]
        dx2 = l2['end'][0] - l2['start'][0]
        dy2 = l2['end'][1] - l2['start'][1]

        denom = dx1 * dy2 - dy1 * dx2
        if abs(denom) < 1e-8:
            return None

        t = ((l2['start'][0] - l1['start'][0]) * dy2
             - (l2['start'][1] - l1['start'][1]) * dx2) / denom
        return (l1['start'][0] + t * dx1, l1['start'][1] + t * dy1)

    def _find_corner(self, lines):
        """找 90° 夹角墙角，返回 (detected, x, y, confidence)"""
        best = None
        best_conf = 0.0

        for i, l1 in enumerate(lines):
            for j, l2 in enumerate(lines):
                if i >= j:
                    continue

                diff = abs(self._normalize_angle(l1['angle'] - l2['angle']))
                half_pi = math.pi / 2
                if not (half_pi - self.angle_tolerance < diff < half_pi + self.angle_tolerance):
                    continue

                corner = self._line_intersection(l1, l2)
                if corner is None:
                    continue

                dist = math.hypot(corner[0], corner[1])
                if not (self.min_distance < dist < self.max_distance):
                    continue

                angle_score = 1.0 - abs(diff - half_pi) / self.angle_tolerance
                length_score = min(1.0, (l1['length'] + l2['length']) / 1.5)
                dist_score = 1.0 / (1.0 + dist)
                conf = angle_score * 0.5 + length_score * 0.3 + dist_score * 0.2

                if conf > best_conf:
                    best_conf = conf
                    best = (corner[0], corner[1])

        if best and best_conf > 0.6:
            return True, best[0], best[1], best_conf
        return False, None, None, 0.0

    def get_debug_lines(self):
        """返回当前检测到的直线（用于 RViz 调试）"""
        return []

    @staticmethod
    def _normalize_angle(a):
        return math.atan2(math.sin(a), math.cos(a))