import math
import unittest

from racing_stage2.track_controller import (
    ImuDistancePose,
    RoundedRectangleTrack,
    Stage2TrackController,
    wrap_angle,
)


class TrackControllerTest(unittest.TestCase):
    def _enter_medium(self, controller):
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        return controller.step(
            1.0, (-0.14, 0.54), math.pi / 2.0,
            yaw_rate=0.45, distance_m=0.61,
        )

    def test_entry_is_nonzero_and_has_correct_turn_sign(self):
        cw = Stage2TrackController()
        cw.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        command = cw.step(0.1, (0.0, 0.0), 0.0, distance_m=0.0)
        self.assertEqual(command.state, cw.ENTRY)
        self.assertGreater(command.linear, 0.0)
        self.assertGreater(command.angular, 0.0)

        ccw = Stage2TrackController()
        ccw.start('counterclockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        self.assertLess(ccw.step(0.1, (0.0, 0.0), 0.0, distance_m=0.0).angular, 0.0)

    def test_entry_transfers_to_named_medium_line(self):
        controller = Stage2TrackController()
        command = self._enter_medium(controller)
        self.assertEqual(command.state, controller.TRACK)
        self.assertEqual(command.segment, 'entry_medium')
        self.assertGreater(command.linear, 0.0)

    def test_entry_arc_hands_off_after_lead_exit_reaches_target(self):
        controller = Stage2TrackController()
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        # The exit section starts before 90 degrees.  Once the measured turn
        # reaches the completion tolerance, residual yaw rate must not hold
        # Stage2 in the entry corner.
        command = controller.step(
            1.0, (-0.34, 0.30), math.radians(88.0),
            yaw_rate=1.20, distance_m=0.61,
        )
        self.assertEqual(command.state, controller.TRACK)
        self.assertEqual(command.segment, 'entry_medium')

    def test_valid_vision_reanchors_the_first_medium_heading(self):
        controller = Stage2TrackController(entry_align_hold_sec=0.10)
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        visual_off_center = {'valid': True, 'error': 0.25, 'confidence': 0.8, 'age': 0.02}
        command = controller.step(1.0, (-0.14, 0.54), math.pi / 2.0,
                                  yaw_rate=0.0, distance_m=0.61, visual=visual_off_center)
        self.assertEqual(command.state, controller.ALIGN)
        self.assertEqual(command.segment, 'entry_align')
        visual_centered = {'valid': True, 'error': 0.0, 'confidence': 0.8, 'age': 0.02}
        controller.step(1.1, (-0.14, 0.56), math.radians(80.0),
                        yaw_rate=0.0, distance_m=0.63, visual=visual_centered)
        command = controller.step(1.25, (-0.14, 0.58), math.radians(80.0),
                                  yaw_rate=0.0, distance_m=0.65, visual=visual_centered)
        self.assertEqual(command.segment, 'entry_medium')
        self.assertAlmostEqual(command.heading_error_rad, 0.0, places=6)

    def test_entry_align_distance_limit_falls_forward_with_best_visual_heading(self):
        controller = Stage2TrackController(entry_align_hold_sec=0.10)
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        off_center = {'valid': True, 'error': 0.32, 'confidence': 0.75, 'age': 0.03}
        command = controller.step(
            1.0, (-0.34, 0.30), math.radians(86.4),
            yaw_rate=0.48, distance_m=0.487, visual=off_center,
        )
        self.assertEqual(command.state, controller.ALIGN)
        better = {'valid': True, 'error': 0.18, 'confidence': 0.70, 'age': 0.03}
        controller.step(1.1, (-0.78, 0.30), math.radians(177.8),
                        yaw_rate=0.34, distance_m=0.72, visual=better)
        worse = {'valid': True, 'error': 0.31, 'confidence': 0.54, 'age': 0.03}
        command = controller.step(1.2, (-1.05, 0.29), math.radians(-175.8),
                                  yaw_rate=0.03, distance_m=0.95, visual=worse)
        self.assertFalse(command.safe_stop)
        self.assertEqual(command.state, controller.TRACK)
        self.assertEqual(command.segment, 'entry_medium')

    def test_medium_line_uses_full_110m_not_radius_subtracted_length(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        command = controller.step(1.1, (-0.60, 1.15), math.pi / 2.0, distance_m=1.65)
        self.assertEqual(command.segment, 'entry_medium')
        command = controller.step(1.2, (-0.85, 1.35), math.pi / 2.0,
                                  yaw_rate=0.0, distance_m=1.71)
        self.assertEqual(command.segment, 'left_side_arc')

    @staticmethod
    def _front_boundary_visual(**overrides):
        visual = {
            'valid': True,
            'confidence': 0.80,
            'age': 0.02,
            'boundary_ahead': True,
            'boundary_top_y_ratio': 0.20,
            'boundary_angle_deg': 0.0,
        }
        visual.update(overrides)
        return visual

    def _entry_boundary_controller(self, direction='clockwise'):
        controller = Stage2TrackController(
            max_speed=0.66,
            entry_medium_distance_m=0.65,
            entry_boundary_trigger_enabled=True,
            entry_boundary_guard_half_width_m=0.15,
            entry_boundary_top_y_ratio=0.18,
            entry_boundary_max_angle_deg=20.0,
            entry_boundary_confirm_frames=3,
        )
        self._enter_medium(controller)
        self.assertEqual(controller.active_segment_name, 'entry_medium')
        return controller, direction

    def test_entry_boundary_is_ignored_before_guard_window(self):
        controller, _ = self._entry_boundary_controller()
        command = controller.step(1.1, (-0.20, 0.60), math.pi / 2.0,
                                  distance_m=1.05,
                                  visual=self._front_boundary_visual())
        self.assertEqual(command.segment, 'entry_medium')
        self.assertEqual(command.entry_boundary_trigger, 'below_guard_min')
        self.assertEqual(command.entry_boundary_confirm_frames, 0)

    def test_entry_boundary_confirms_in_window_and_enters_first_180(self):
        controller, _ = self._entry_boundary_controller()
        visual = self._front_boundary_visual()
        for now, distance in ((1.1, 1.12), (1.2, 1.14)):
            command = controller.step(now, (-0.20, 0.60), math.pi / 2.0,
                                      distance_m=distance, visual=visual)
            self.assertEqual(command.segment, 'entry_medium')
            self.assertEqual(command.entry_boundary_trigger, 'vision_candidate')
        command = controller.step(1.3, (-0.20, 0.60), math.pi / 2.0,
                                  distance_m=1.16, visual=visual)
        self.assertEqual(command.segment, 'left_side_arc')
        self.assertEqual(command.entry_boundary_trigger, 'vision_confirmed')
        self.assertEqual(command.entry_boundary_angle_deg, 0.0)

    def test_side_arc_boundary_only_triggers_in_pre_corner_gate_at_reduced_speed(self):
        controller = Stage2TrackController(
            max_speed=0.66,
            entry_medium_distance_m=0.65,
            entry_boundary_trigger_enabled=True,
            entry_boundary_confirm_frames=1,
            side_arc_vision_enabled=True,
            side_arc_vision_trigger_lead_m=0.002,
            side_arc_vision_trigger_speed_mps=0.45,
        )
        self._enter_medium(controller)
        visual = self._front_boundary_visual()

        # The medium segment starts at total distance 0.61m. At progress
        # 0.647m, valid SEG is still outside the 2mm pre-corner gate.
        command = controller.step(
            1.1, (-0.20, 0.60), math.pi / 2.0,
            distance_m=1.257, visual=visual,
        )
        self.assertEqual(command.segment, 'entry_medium')
        self.assertEqual(command.entry_boundary_trigger, 'before_vision_trigger_gate')
        self.assertAlmostEqual(command.linear, controller.max_speed)

        command = controller.step(
            1.2, (-0.20, 0.60), math.pi / 2.0,
            distance_m=1.258, visual=self._front_boundary_visual(valid=False),
        )
        self.assertEqual(command.segment, 'entry_medium')
        self.assertEqual(command.entry_boundary_trigger, 'vision_rejected')
        self.assertAlmostEqual(command.linear, 0.45)

        command = controller.step(
            1.25, (-0.20, 0.60), math.pi / 2.0,
            distance_m=1.258, visual=visual,
        )
        self.assertEqual(command.segment, 'left_side_arc')
        self.assertEqual(command.entry_boundary_trigger, 'vision_confirmed')

        controller = Stage2TrackController(
            max_speed=0.66,
            entry_medium_distance_m=0.65,
            entry_boundary_trigger_enabled=True,
            entry_boundary_confirm_frames=1,
            side_arc_vision_enabled=True,
            side_arc_vision_trigger_lead_m=0.002,
            side_arc_vision_trigger_speed_mps=0.45,
            turn_force_map_x_enabled=True,
        )
        self._enter_medium(controller)
        command = controller.step(
            1.3, (-0.20, 0.60), math.pi / 2.0,
            distance_m=1.261, visual=visual, map_x=2.50,
        )
        self.assertEqual(command.segment, 'entry_medium')
        self.assertEqual(command.entry_boundary_trigger, 'after_vision_trigger_gate')
        self.assertAlmostEqual(command.linear, 0.45)

    def test_entry_boundary_confirmation_resets_after_invalid_frame(self):
        controller, _ = self._entry_boundary_controller()
        visual = self._front_boundary_visual()
        controller.step(1.1, (-0.20, 0.60), math.pi / 2.0,
                        distance_m=1.12, visual=visual)
        command = controller.step(1.2, (-0.20, 0.60), math.pi / 2.0,
                                  distance_m=1.14,
                                  visual=self._front_boundary_visual(valid=False))
        self.assertEqual(command.entry_boundary_confirm_frames, 0)
        controller.step(1.3, (-0.20, 0.60), math.pi / 2.0,
                        distance_m=1.16, visual=visual)
        command = controller.step(1.4, (-0.20, 0.60), math.pi / 2.0,
                                  distance_m=1.18, visual=visual)
        self.assertEqual(command.segment, 'entry_medium')
        self.assertEqual(command.entry_boundary_confirm_frames, 2)

    def test_entry_boundary_uses_distance_fallback_at_guard_max(self):
        controller, _ = self._entry_boundary_controller()
        command = controller.step(1.1, (-0.20, 0.60), math.pi / 2.0,
                                  distance_m=1.41,
                                  visual=self._front_boundary_visual(valid=False))
        self.assertEqual(command.segment, 'left_side_arc')
        self.assertEqual(command.entry_boundary_trigger, 'distance_fallback')

    def test_side_arc_vision_switch_uses_only_map_x_when_disabled(self):
        controller = Stage2TrackController(
            entry_medium_distance_m=0.65,
            entry_boundary_trigger_enabled=True,
            side_arc_vision_enabled=False,
            turn_force_map_x_enabled=True,
            turn_force_min_map_x=2.30,
            turn_force_max_map_x=3.90,
        )
        self._enter_medium(controller)
        command = controller.step(
            1.1, (-0.20, 0.60), math.pi / 2.0, distance_m=1.12,
            visual=self._front_boundary_visual(), map_x=2.50,
        )
        self.assertEqual(command.segment, 'entry_medium')
        self.assertEqual(command.entry_boundary_trigger, 'vision_disabled_wait_map_x')
        command = controller.step(
            1.2, (-0.20, 0.60), math.pi / 2.0, distance_m=1.14,
            visual=self._front_boundary_visual(), map_x=2.30,
        )
        self.assertEqual(command.segment, 'left_side_arc')
        self.assertEqual(command.entry_boundary_trigger, 'map_x_fallback')

    def test_side_arc_vision_switch_keeps_tf_loss_distance_fallback(self):
        controller = Stage2TrackController(
            entry_medium_distance_m=0.65,
            entry_boundary_trigger_enabled=True,
            entry_boundary_guard_half_width_m=0.15,
            side_arc_vision_enabled=False,
            turn_force_map_x_enabled=True,
        )
        self._enter_medium(controller)
        command = controller.step(
            1.1, (-0.20, 0.60), math.pi / 2.0, distance_m=1.41,
            visual=self._front_boundary_visual(), map_x=None,
        )
        self.assertEqual(command.segment, 'left_side_arc')
        self.assertEqual(command.entry_boundary_trigger, 'distance_fallback_no_map_tf')

    def test_entry_boundary_keeps_qr_selected_turn_direction(self):
        for direction, entry_sign, corner_sign in (
                ('clockwise', 1.0, -1.0),
                ('counterclockwise', -1.0, 1.0)):
            controller = Stage2TrackController(
                entry_medium_distance_m=0.65,
                entry_boundary_trigger_enabled=True,
                entry_boundary_confirm_frames=1,
            )
            controller.start(direction, (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
            controller.step(1.0, (-0.14, 0.54), entry_sign * math.pi / 2.0,
                            yaw_rate=0.45, distance_m=0.61)
            command = controller.step(1.1, (-0.20, 0.60), entry_sign * math.pi / 2.0,
                                      distance_m=1.12,
                                      visual=self._front_boundary_visual())
            self.assertEqual(command.segment, 'left_side_arc')
            self.assertGreater(command.angular * corner_sign, 0.0)

    def test_side_is_one_180_degree_arc(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        controller.step(1.1, (-0.85, 1.35), math.pi / 2.0, yaw_rate=0.0, distance_m=1.71)
        command = controller.step(1.2, (-0.85, 1.35), math.pi / 2.0, distance_m=3.00)
        self.assertEqual(command.segment, 'left_side_arc')
        command = controller.step(1.3, (-0.85, 1.35), -math.pi / 2.0, distance_m=3.00)
        self.assertEqual(command.segment, 'top_long')

    def test_side_arc_does_not_wait_for_yaw_rate_to_settle_at_full_turn(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        controller.step(1.1, (-0.85, 1.35), math.pi / 2.0, yaw_rate=0.0, distance_m=1.71)
        command = controller.step(
            1.2, (-0.85, 1.35), -math.pi / 2.0,
            yaw_rate=-1.0, distance_m=3.00,
        )
        self.assertEqual(command.segment, 'top_long')

    def test_side_arc_uses_configured_geometric_curvature(self):
        controller = Stage2TrackController(
            corner_speed=0.35,
            corner_radius=0.25,
            corner_angular=1.5,
        )
        self._enter_medium(controller)
        controller.step(1.1, (-0.85, 1.35), math.pi / 2.0, yaw_rate=0.0, distance_m=1.71)
        command = controller.step(
            1.2, (-0.85, 1.35), 0.0,
            yaw_rate=-1.46, distance_m=2.00,
        )
        self.assertAlmostEqual(command.linear / abs(command.angular), 0.25, places=6)

    def test_side_arc_latches_actual_entry_yaw_without_counting_entry_error(self):
        controller = Stage2TrackController(
            corner_speed=0.35,
            corner_radius=0.25,
        )
        self._enter_medium(controller)
        # The boundary can be reached while the vehicle has a residual 17°
        # heading error.  That yaw is the zero point for this relative 180°
        # manoeuvre, rather than fake progress already completed at entry.
        entry_yaw = math.radians(107.0)
        command = controller.step(
            1.1, (-0.85, 1.35), entry_yaw, yaw_rate=0.0, distance_m=1.71,
        )
        self.assertEqual(command.segment, 'left_side_arc')
        self.assertAlmostEqual(command.turn_progress_rad, 0.0, places=6)
        self.assertAlmostEqual(
            command.arc_final_heading_error_rad, math.radians(180.0), places=6
        )

        command = controller.step(
            1.2, (-0.85, 1.35), math.radians(-62.0), yaw_rate=-0.8,
            distance_m=3.15,
        )
        self.assertGreater(command.linear, 0.08)
        self.assertLess(command.linear, 0.35)

        # The corner stays active until the complete relative 180 degree turn.
        command = controller.step(
            1.3, (-0.85, 1.35), math.radians(-62.0), yaw_rate=-0.10,
            distance_m=3.20,
        )
        self.assertEqual(command.segment, 'left_side_arc')

        # Crossing the full 180-degree target advances to top_long.
        command = controller.step(
            1.4, (-0.85, 1.35), math.radians(-73.0), yaw_rate=-0.10,
            distance_m=3.25,
        )
        self.assertEqual(command.segment, 'top_long')
        self.assertEqual(command.arc_completion_reason, 'lead_exit_complete')

    def test_side_arc_does_not_finish_before_full_180_degrees(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        controller.step(1.1, (-0.85, 1.35), math.pi / 2.0,
                        yaw_rate=0.0, distance_m=1.71)
        command = controller.step(
            1.2, (-0.85, 1.35), -math.radians(82.0),
            yaw_rate=-0.10, distance_m=3.20,
        )
        self.assertEqual(command.segment, 'left_side_arc')
        command = controller.step(
            1.3, (-0.85, 1.35), -math.pi / 2.0,
            yaw_rate=-0.10, distance_m=3.20,
        )
        self.assertEqual(command.segment, 'top_long')
        self.assertEqual(command.arc_completion_reason, 'lead_exit_complete')

    def test_top_long_uses_full_259m(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        controller.step(1.1, (-0.85, 1.35), math.pi / 2.0, yaw_rate=0.0, distance_m=1.71)
        controller.step(1.2, (-0.85, 1.35), -math.pi / 2.0, distance_m=3.00)
        command = controller.step(1.3, (1.0, 1.35), -math.pi / 2.0, distance_m=5.40)
        self.assertEqual(command.segment, 'top_long')

    def test_arc_mismatch_stops_before_unbounded_motion(self):
        controller = Stage2TrackController()
        controller.start('clockwise', (0.0, 0.0), 0.0, 0.0, distance_m=0.0)
        command = controller.step(1.0, (0.0, 0.8), 0.0, distance_m=0.8)
        self.assertTrue(command.safe_stop)
        self.assertEqual(command.reason, 'entry_arc_distance_yaw_mismatch')

    def test_inertial_cross_is_logged_but_does_not_steer_a_line(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        centered = controller.step(1.1, (-0.17, 0.57), math.pi / 2.0,
                                   yaw_rate=0.0, distance_m=0.70)
        offset = controller.step(1.2, (-0.47, 0.57), math.pi / 2.0,
                                 yaw_rate=0.0, distance_m=0.71)
        self.assertNotEqual(centered.cross_track_m, offset.cross_track_m)
        self.assertAlmostEqual(centered.angular, offset.angular, places=6)

    def test_line_does_not_enter_side_arc_with_large_heading_error(self):
        controller = Stage2TrackController()
        self._enter_medium(controller)
        command = controller.step(1.1, (-0.85, 1.35), math.radians(55.0),
                                  yaw_rate=0.40, distance_m=1.71)
        self.assertEqual(command.segment, 'entry_medium')

    def test_odom_combined_distance_is_integrated_only_with_imu_yaw(self):
        pose = ImuDistancePose()
        pose.reset((2.0, 3.0), 0.0)
        x, y = pose.update((2.1, 3.0), 0.0)
        self.assertAlmostEqual(x, 0.1, places=6)
        self.assertAlmostEqual(y, 0.0, places=6)
        x, y = pose.update((2.1, 3.1), math.pi / 2.0)
        self.assertAlmostEqual(x, 0.1 + 0.1 / math.sqrt(2.0), places=6)
        self.assertAlmostEqual(y, 0.1 / math.sqrt(2.0), places=6)

    def test_sampled_compatibility_geometry_contains_arcs(self):
        track = RoundedRectangleTrack((0.0, 0.0), math.pi, True)
        self.assertGreater(len(track.points), 100)
        self.assertTrue(any(abs(point.curvature) > 1.0 for point in track.points))

    def test_wrap_angle(self):
        self.assertAlmostEqual(wrap_angle(3.0 * math.pi), -math.pi)


if __name__ == '__main__':
    unittest.main()
