"""Hardware-only simulation: diff-drive odom/IMU integration and ray-circle lidar."""

from __future__ import annotations

import math

from geometry_msgs.msg import Quaternion, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import Header

SIM_SCAN_ANGLE_MIN = -math.pi
SIM_SCAN_ANGLE_MAX = math.pi
SIM_SCAN_NUM_RAYS = 360
SIM_SCAN_RANGE_MIN = 0.20
SIM_SCAN_RANGE_MAX = 3.0


class DiffDriveSim:
    """Integrate (v, ω) with simple acceleration limits."""

    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.linear = 0.0
        self.angular = 0.0

    def step(
        self,
        target_linear,
        target_angular,
        dt,
        max_lin_accel=0.8,
        max_ang_accel=2.5,
    ):
        dt = float(dt)
        self.linear += max(-max_lin_accel * dt, min(max_lin_accel * dt, target_linear - self.linear))
        self.angular += max(-max_ang_accel * dt, min(max_ang_accel * dt, target_angular - self.angular))
        self.x += self.linear * math.cos(self.yaw) * dt
        self.y += self.linear * math.sin(self.yaw) * dt
        self.yaw = math.atan2(math.sin(self.yaw + self.angular * dt), math.cos(self.yaw + self.angular * dt))


def ray_circle_range(ox, oy, radius, rx, ry, ray_angle, max_range):
    dx = ox - rx
    dy = oy - ry
    ray_dx = math.cos(ray_angle)
    ray_dy = math.sin(ray_angle)
    projection = dx * ray_dx + dy * ray_dy
    if projection <= 0.0:
        return max_range
    perp = abs(dx * ray_dy - dy * ray_dx)
    if perp >= radius:
        return max_range
    chord_half = math.sqrt(max(radius * radius - perp * perp, 0.0))
    hit = projection - chord_half
    if hit <= 0.0:
        return max_range
    return min(hit, max_range)


def build_laserscan(robot: DiffDriveSim, obstacles, stamp, frame_id='base_scan') -> LaserScan:
    angle_min = SIM_SCAN_ANGLE_MIN
    angle_max = SIM_SCAN_ANGLE_MAX
    num_rays = SIM_SCAN_NUM_RAYS
    angle_increment = (angle_max - angle_min) / max(num_rays - 1, 1)
    max_range = SIM_SCAN_RANGE_MAX
    ranges = []
    for index in range(num_rays):
        local_angle = angle_min + index * angle_increment
        global_angle = robot.yaw + local_angle
        nearest = max_range
        for obstacle in obstacles:
            nearest = min(
                nearest,
                ray_circle_range(
                    obstacle['x'],
                    obstacle['y'],
                    obstacle['r'],
                    robot.x,
                    robot.y,
                    global_angle,
                    max_range,
                ),
            )
        ranges.append(float(nearest))

    scan = LaserScan()
    scan.header = Header()
    scan.header.stamp = stamp
    scan.header.frame_id = frame_id
    scan.angle_min = angle_min
    scan.angle_max = angle_max
    scan.angle_increment = angle_increment
    scan.range_min = SIM_SCAN_RANGE_MIN
    scan.range_max = max_range
    scan.ranges = ranges
    return scan


def build_odometry(robot: DiffDriveSim, stamp) -> Odometry:
    odom = Odometry()
    odom.header.stamp = stamp
    odom.header.frame_id = 'odom'
    odom.child_frame_id = 'base_link'
    odom.pose.pose.position.x = robot.x
    odom.pose.pose.position.y = robot.y
    odom.pose.pose.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(robot.yaw / 2.0),
        w=math.cos(robot.yaw / 2.0),
    )
    return odom


def build_imu(robot: DiffDriveSim, stamp) -> Imu:
    imu = Imu()
    imu.header.stamp = stamp
    imu.header.frame_id = 'imu_link'
    imu.orientation = Quaternion(
        x=0.0,
        y=0.0,
        z=math.sin(robot.yaw / 2.0),
        w=math.cos(robot.yaw / 2.0),
    )
    imu.angular_velocity = Vector3(x=0.0, y=0.0, z=robot.angular)
    imu.linear_acceleration = Vector3(x=robot.linear, y=0.0, z=0.0)
    return imu
