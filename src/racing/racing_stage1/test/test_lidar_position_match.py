from types import SimpleNamespace

import cv2
import numpy as np

from racing_stage1.competition_controller import CompetitionController
from geometry_msgs.msg import Twist


def test_fixed_heading_xy_match_recovers_wall_offset():
    controller = CompetitionController.__new__(CompetitionController)
    controller.lidar_position_max_score_distance = 0.35
    controller.lidar_position_inlier_distance = 0.10
    controller.lidar_position_min_inlier_ratio = 0.45
    controller.lidar_position_max_mean_distance = 0.12

    edge_image = np.full((500, 500), 255, dtype=np.uint8)
    edge_image[0, :] = 0
    edge_image[:, 0] = 0
    distance_map = cv2.distanceTransform(edge_image, cv2.DIST_L2, 5) * 0.01
    info = SimpleNamespace(
        width=500,
        height=500,
        resolution=0.01,
        origin=SimpleNamespace(position=SimpleNamespace(x=0.0, y=0.0)),
    )
    wall_samples = np.linspace(0.3, 2.7, 50, dtype=np.float32)
    rotated_x = np.concatenate((np.full(50, -1.0, dtype=np.float32), wall_samples - 1.0))
    rotated_y = np.concatenate((wall_samples - 1.0, np.full(50, -1.0, dtype=np.float32)))

    coarse = controller._search_lidar_position(
        rotated_x, rotated_y, (1.12, 1.08), 0.25, 0.05, info, distance_map)
    fine = controller._search_lidar_position(
        rotated_x, rotated_y, (coarse['x'], coarse['y']), 0.06, 0.01,
        info, distance_map)

    assert controller._lidar_match_valid(fine)
    assert abs(fine['x'] - 1.0) <= 0.02
    assert abs(fine['y'] - 1.0) <= 0.02


def test_steering_reversal_must_cross_zero():
    controller = CompetitionController.__new__(CompetitionController)
    controller._last_cmd = Twist()
    controller._last_cmd.angular.z = 0.30
    controller._last_cmd_time = 0.0
    controller.control_rate_hz = 20.0
    controller.max_linear_accel = 1.5
    controller.max_angular_accel = 2.5
    controller.angular_reversal_deadband = 0.08
    controller._now = lambda: 0.05

    _, limited_w = controller._rate_limit_command(0.35, -0.30)

    assert limited_w > 0.0
