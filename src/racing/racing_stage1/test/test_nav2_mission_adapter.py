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


def test_entry_route_uses_left_approach_then_corridor_then_handoff():
    controller = CompetitionController.__new__(CompetitionController)
    controller.entry_turn_goal = (4.55, 1.35)
    controller.entry_turn_yaw = math.radians(330.0)
    controller.entry_lower_turn_goal = (4.45, 0.75)
    controller.entry_lower_turn_yaw = math.radians(225.0)
    controller.entry_lower_approach_goal = (3.50, 0.90)
    controller.entry_lower_approach_yaw = math.radians(150.0)
    controller.entry_align_goal = (2.25, 1.5)
    controller.entry_align_yaw = math.pi / 2.0
    controller.entry_corridor_approach_goal = (2.25, 2.1)
    controller.entry_corridor_approach_yaw = math.pi / 2.0
    controller.entry_corridor_goal = (2.5, 2.1)
    controller.entry_corridor_yaw = math.pi / 2.0
    controller.entry_goal = (2.5, 2.5)
    controller.entry_yaw = math.pi / 2.0

    route = controller._entry_route_targets()

    assert [item[0] for item in route] == [
        'entry_turn', 'entry_lower_turn', 'entry_lower_approach',
        'entry_align', 'entry_corridor_approach', 'entry_corridor', 'entry']
    assert route[-1][1:] == ((2.5, 2.5), math.pi / 2.0)


def _entry_result_controller():
    controller = CompetitionController.__new__(CompetitionController)
    controller._lock = threading.RLock()
    controller._released = False
    controller._goal_generation = 1
    controller._goal_handle = object()
    controller._goal_future = object()
    controller._goal_sent_at = 10.0
    controller._goal_result = None
    controller._goal_retry_count = 0
    controller.goal_retry_limit = 3
    controller._goal_retry_at = 0.0
    controller._watchdog_latched = False
    controller._entry_route_index = 0
    controller._pending_entry = False
    controller._entry_goal_done = False
    controller._entry_stable_since = None
    controller.entry_align_goal = (2.25, 1.5)
    controller.entry_align_yaw = math.pi / 2.0
    controller.entry_turn_goal = (4.55, 1.35)
    controller.entry_turn_yaw = math.radians(330.0)
    controller.entry_lower_turn_goal = (4.45, 0.75)
    controller.entry_lower_turn_yaw = math.radians(225.0)
    controller.entry_lower_approach_goal = (3.50, 0.90)
    controller.entry_lower_approach_yaw = math.radians(150.0)
    controller.entry_corridor_approach_goal = (2.25, 2.1)
    controller.entry_corridor_approach_yaw = math.pi / 2.0
    controller.entry_corridor_goal = (2.5, 2.1)
    controller.entry_corridor_yaw = math.pi / 2.0
    controller.entry_goal = (2.5, 2.5)
    controller.entry_yaw = math.pi / 2.0
    controller.goal_retry_delay = 1.0
    controller._cancel_waiting = False
    controller.get_logger = lambda: SimpleNamespace(
        info=lambda _message: None,
        warning=lambda _message: None,
        error=lambda _message: None,
    )
    return controller


def test_entry_success_advances_each_segment_and_handoff_only_after_final():
    controller = _entry_result_controller()
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=controller_module.GoalStatus.STATUS_SUCCEEDED))

    route = controller._entry_route_targets()
    for generation, (kind, _target, _yaw) in enumerate(route[:-1], start=1):
        controller._goal_generation = generation
        controller._goal_handle = object()
        controller._goal_future = object()
        controller._goal_kind = kind
        controller._goal_result_cb(future, generation, kind)
        assert controller._entry_route_index == generation
        assert controller._pending_entry is True
        assert controller._entry_goal_done is False

    controller._goal_generation = len(route)
    controller._goal_handle = object()
    controller._goal_future = object()
    controller._goal_kind = 'entry'
    controller._goal_result_cb(future, len(route), 'entry')
    assert controller._pending_entry is True
    assert controller._entry_goal_done is True


def test_entry_failure_retries_same_segment_without_restarting_route():
    controller = _entry_result_controller()
    controller._entry_route_index = 1
    controller._goal_kind = 'entry_lower_turn'
    future = SimpleNamespace(result=lambda: SimpleNamespace(
        status=controller_module.GoalStatus.STATUS_ABORTED))

    controller._goal_result_cb(future, 1, 'entry_lower_turn')

    assert controller._entry_route_index == 1
    assert controller._pending_entry is True
    assert controller._goal_retry_count == 1
    assert controller._goal_result is None


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
