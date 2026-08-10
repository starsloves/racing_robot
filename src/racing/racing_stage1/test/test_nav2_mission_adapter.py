import math
import threading
from types import SimpleNamespace

from lifecycle_msgs.msg import State
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import racing_stage1.competition_controller as controller_module
from racing_stage1.competition_controller import CompetitionController, normalize_angle


def test_normalize_angle_wraps_without_changing_heading_error():
    assert math.isclose(abs(normalize_angle(3.0 * math.pi)), math.pi, abs_tol=1e-9)
    assert math.isclose(abs(normalize_angle(-3.0 * math.pi)), math.pi, abs_tol=1e-9)


def test_nav2_goal_uses_map_frame_and_requested_yaw():
    controller = CompetitionController.__new__(CompetitionController)
    controller.map_frame = 'map'

    goal = controller._make_goal(2.5, 2.5, math.pi / 2.0)

    assert goal.pose.header.frame_id == 'map'
    assert math.isclose(goal.pose.pose.position.x, 2.5)
    assert math.isclose(goal.pose.pose.position.y, 2.5)
    assert math.isclose(goal.pose.pose.orientation.z, math.sin(math.pi / 4.0))
    assert math.isclose(goal.pose.pose.orientation.w, math.cos(math.pi / 4.0))


def test_nav2_lifecycle_callback_marks_only_active_state_ready():
    controller = CompetitionController.__new__(CompetitionController)
    controller._lock = threading.RLock()
    controller._nav2_state_future = object()
    controller._nav2_active = False
    controller.get_logger = lambda: SimpleNamespace(info=lambda _message: None)
    active = SimpleNamespace(
        result=lambda: SimpleNamespace(
            current_state=SimpleNamespace(id=State.PRIMARY_STATE_ACTIVE)))

    controller._nav2_state_cb(active)

    assert controller._nav2_active is True
    assert controller._nav2_state_future is None


def test_real_pose_log_uses_map_tf_value_once_per_period(monkeypatch):
    controller = CompetitionController.__new__(CompetitionController)
    controller._last_pose_log_at = 0.0
    controller.pose_log_period = 60.0
    messages = []
    monkeypatch.setattr(controller_module, 'terminal_write', messages.append)

    controller._log_real_pose((1.25, 2.50, math.pi / 2.0))
    controller._log_real_pose((9.0, 9.0, 0.0))

    assert messages == ['[POSE_REAL] real_map=(1.250,2.500) yaw=90.0deg source=map_tf']


def test_nearest_path_distance_uses_base_link_xy():
    path = Path()
    for x, y in ((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)):
        point = PoseStamped()
        point.pose.position.x = x
        point.pose.position.y = y
        path.poses.append(point)

    distance = CompetitionController._nearest_path_distance((1.0, 0.25, 0.0), path)
    midpoint_distance = CompetitionController._nearest_path_distance((0.5, 0.25, 0.0), path)

    assert math.isclose(distance, 0.25, abs_tol=1e-9)
    assert math.isclose(midpoint_distance, 0.25, abs_tol=1e-9)


def test_watchdog_cancels_goal_without_initial_plan(monkeypatch):
    controller = CompetitionController.__new__(CompetitionController)
    controller._watchdog_latched = False
    controller._goal_handle = object()
    controller._goal_future = None
    controller._goal_sent_at = 10.0
    controller._last_plan_received_at = 0.0
    controller.initial_plan_timeout = 3.0
    controller._goal_kind = 'qr_search'
    controller._goal_generation = 1
    controller._last_plan = object()
    controller._lock = threading.RLock()
    controller._cancel_goal_locked = lambda: None
    controller._schedule_goal_retry_locked = lambda _kind, _reason: None
    controller.get_logger = lambda: SimpleNamespace(error=lambda _message: None)
    monkeypatch.setattr(controller_module, 'terminal_write', lambda _message: None)

    assert controller._check_goal_watchdog_locked(14.0, None) is True
    assert controller._watchdog_latched is True
