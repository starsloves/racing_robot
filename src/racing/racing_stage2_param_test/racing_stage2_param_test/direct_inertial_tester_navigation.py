"""Mission vs avoidance mode — thin FSM only."""

import math


class DirectInertialTesterNavigationMixin:
    """Delegates move-segment detour to goal_direct in direct_inertial_tester_avoidance.py."""

    def reset_navigation_state(self):
        self.reset_avoidance_runtime()

    def navigation_step(self, now_sec):
        """True = avoidance owns this tick (do not run mission move/turn)."""
        return self.avoidance_step(now_sec)

    # Legacy names used by obstacle trigger logging
    def navigation_can_detour(self):
        return self.avoidance_can_run()

    def detour_enabled_for_current_segment(self):
        return self.avoidance_can_run()

    def navigation_in_trigger_envelope(self):
        if not self.avoidance_can_run():
            return False
        front = self.front_obstacle_distance
        return math.isfinite(front) and front <= self.detour_obstacle_detect_distance

    def obstacle_is_active(self):
        return self.avoidance_active or self.template_path_blocker_imminent()

    def detour_trigger_allowed(self):
        return self.avoidance_active
