import math
import threading
from types import SimpleNamespace

from lifecycle_msgs.msg import State
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
