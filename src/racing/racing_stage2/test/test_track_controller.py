import math
import unittest

from racing_stage2.track_controller import (
    ImuDistancePose,
    RoundedRectangleTrack,
    Stage2TrackController,
    wrap_angle,
)


class TrackControllerTest(unittest.TestCase):
    def _enter_first_line(self, controller):
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        return controller.step(
            1.0, (-0.14, 0.54), math.pi / 2.0,
            yaw_rate=0.45, distance_m=0.61,
        )

    def test_entry_commands_nonzero_forward_motion_and_correct_turn_sign(self):
        clockwise = Stage2TrackController()
        clockwise.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        command = clockwise.step(0.1, (0.0, 0.0), 0.0, distance_m=0.0)
        self.assertEqual(command.state, clockwise.ENTRY)
        self.assertGreater(command.linear, 0.0)
        self.assertGreater(command.angular, 0.0)

        counterclockwise = Stage2TrackController()
        counterclockwise.start('counterclockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        command = counterclockwise.step(0.1, (0.0, 0.0), 0.0, distance_m=0.0)
        self.assertLess(command.angular, 0.0)

    def test_entry_transfers_at_first_valid_distance_angle_crossing(self):
        controller = Stage2TrackController()
        command = self._enter_first_line(controller)
        self.assertEqual(command.state, controller.TRACK)
        self.assertEqual(command.segment, 'line_1')
        self.assertGreater(command.linear, 0.0)
        # The following line owns the yaw-rate damping instead of waiting in
        # the entry arc while a moving chassis continues to travel.
        self.assertLess(command.angular, 0.0)

    def test_first_line_is_distance_locked(self):
        controller = Stage2TrackController()
        self._enter_first_line(controller)
        command = controller.step(1.1, (-0.40, 1.10), math.pi / 2.0, distance_m=1.10)
        self.assertEqual(command.segment, 'line_1')
        command = controller.step(1.2, (-0.50, 1.25), math.pi / 2.0, yaw_rate=0.0, distance_m=1.31)
        self.assertEqual(command.segment, 'corner_1')

    def test_corner_requires_both_distance_and_imu_angle(self):
        controller = Stage2TrackController()
        self._enter_first_line(controller)
        controller.step(1.1, (-0.50, 1.25), math.pi / 2.0, yaw_rate=0.0, distance_m=1.31)
        # Arc distance alone cannot advance a corner.
        command = controller.step(1.2, (-0.50, 1.25), math.pi / 2.0, distance_m=1.95)
        self.assertEqual(command.segment, 'corner_1')
        # Clockwise corner completes at -90 degrees relative to its start.
        command = controller.step(1.3, (-0.50, 1.25), 0.0, distance_m=1.95)
        self.assertNotEqual(command.segment, 'corner_1')

    def test_arc_mismatch_safely_stops_before_unbounded_motion(self):
        controller = Stage2TrackController()
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        command = controller.step(1.0, (0.0, 0.8), 0.0, distance_m=0.8)
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, 'entry_arc_distance_yaw_mismatch')

    def test_line_yaw_rate_damping_keeps_motion_nonzero(self):
        controller = Stage2TrackController()
        command = self._enter_first_line(controller)
        self.assertAlmostEqual(command.linear, controller.corner_speed)
        self.assertLess(command.angular, 0.0)

    def test_left_of_line_is_logged_without_inertial_cross_steering(self):
        controller = Stage2TrackController()
        self._enter_first_line(controller)
        command = controller.step(1.1, (-0.34, 0.54), math.pi / 2.0, distance_m=0.70)
        self.assertGreater(command.cross_track_m, 0.0)
        self.assertAlmostEqual(command.angular, 0.0, places=6)

    def test_line_does_not_enter_a_corner_with_large_heading_or_yaw_rate(self):
        controller = Stage2TrackController()
        self._enter_first_line(controller)
        command = controller.step(
            1.1, (-0.50, 1.25), math.radians(55.0),
            yaw_rate=0.40, distance_m=1.31,
        )
        self.assertEqual(command.segment, 'line_1')

    def test_line_ignores_cross_track_recovery_until_arc_yaw_rate_settles(self):
        controller = Stage2TrackController()
        self._enter_first_line(controller)
        command = controller.step(
            1.1, (-0.34, 0.54), math.pi / 2.0,
            yaw_rate=0.45, distance_m=0.70,
        )
        self.assertGreater(command.angular, -0.30)

    def test_inertial_cross_track_never_changes_line_steering(self):
        controller = Stage2TrackController()
        self._enter_first_line(controller)
        centered = controller.step(
            1.1, (-0.17, 0.57), math.pi / 2.0,
            yaw_rate=0.0, distance_m=0.70,
        )
        offset = controller.step(
            1.2, (-0.47, 0.57), math.pi / 2.0,
            yaw_rate=0.0, distance_m=0.71,
        )
        self.assertNotEqual(centered.cross_track_m, offset.cross_track_m)
        self.assertAlmostEqual(centered.angular, offset.angular, places=6)

    def test_odom_combined_distance_is_integrated_only_with_imu_yaw(self):
        pose = ImuDistancePose()
        pose.reset((2.0, 3.0), 0.0)
        x, y = pose.update((2.1, 3.0), 0.0)
        self.assertAlmostEqual(x, 0.1, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        x, y = pose.update((2.1, 3.1), math.pi / 2.0)
        self.assertAlmostEqual(x, 0.1 + 0.1 / math.sqrt(2.0), places=6)
        self.assertAlmostEqual(y, 0.1 / math.sqrt(2.0), places=6)
        self.assertAlmostEqual(pose.total_distance_m, 0.2, places=6)

    def test_sampled_compatibility_geometry_contains_arcs(self):
        track = RoundedRectangleTrack((0.0, 0.0), math.pi, True)
        self.assertGreater(len(track.points), 100)
        self.assertTrue(any(abs(point.curvature) > 1.0 for point in track.points))

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), -math.pi)


if __name__ == '__main__':
    unittest.main()
