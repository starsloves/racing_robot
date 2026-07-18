import math
import unittest

from racing_stage2.stage2_hybrid_controller import HybridConfig, Stage2HybridController


def line(**overrides):
    value = {'valid': True, 'age': 0.02, 'confidence': 0.72, 'valid_rows': 7,
             'error': 0.0, 'curve': 0.0, 'boundary_ahead': False,
             'edge_is_front': False, 'boundary_far_ratio': 0.58,
             'boundary_near_ratio': 0.72, 'front_score': 0.10,
             'top20_seg_fill': 0.10}
    value.update(overrides)
    return value


class RouteControllerTest(unittest.TestCase):
    def setUp(self):
        self.cfg = HybridConfig(
            leg_lengths_csv='1.00,0.80,2.00,0.80,1.00',
        )

    def new_leg_controller(self):
        controller = Stage2HybridController(self.cfg)
        controller.reset('clockwise', 0.0, (0.0, 0.0), 0.0)
        controller.state = controller.LEG
        controller.leg_heading_yaw = 0.0
        return controller

    def test_top20_seg_present_keeps_tracking(self):
        controller = self.new_leg_controller()
        command = controller.step(0.1, line(top20_seg_fill=0.10), 0.0, (0.46, 0.0))
        self.assertEqual(command.state, controller.LEG)

    def test_top20_without_seg_starts_turn_immediately(self):
        controller = self.new_leg_controller()
        command = controller.step(0.1, line(top20_seg_fill=0.0), 0.0, (0.02, 0.0))
        self.assertEqual(command.state, controller.TURN)
        self.assertLess(command.angular, 0.0)

    def test_turn_countersteers_after_nominal_angle_until_vision_exit(self):
        controller = Stage2HybridController(self.cfg)
        controller.reset('clockwise', 0.0, (0.0, 0.0), 0.0)
        controller._start_turn(-1.0, 90.0, 0.0, 0.0)

        command = controller.step(0.2, line(apex_left30_fill=0.10, apex_right30_fill=0.10), -math.radians(100.0), (0.02, 0.0))

        self.assertEqual(command.state, controller.TURN)
        self.assertGreater(command.angular, 0.0)

    def test_first_ring_corner_bridges_directly_to_long_edge(self):
        controller = Stage2HybridController(self.cfg)
        controller.reset('clockwise', 0.0, (0.0, 0.0), 0.0)
        controller._start_turn(-1.0, 90.0, 0.0, 0.0)
        opened = line(apex_left30_fill=0.72, apex_right30_fill=0.71)
        for index in range(3):
            command = controller.step(0.1 + index * 0.1, opened, -math.radians(45.0), (0.01 + index * 0.01, 0.0))
        self.assertEqual(command.state, controller.LEG)
        self.assertEqual(controller.turn_count, 1)
        self.assertTrue(controller.is_bridge_active())
        self.assertLess(command.angular, 0.0)

        # No far SEG on the physical short side must not cause another
        # immediate corner state.
        command = controller.step(0.4, line(top20_seg_fill=0.0), -math.radians(55.0), (0.05, 0.0))
        self.assertEqual(command.state, controller.LEG)
        self.assertTrue(controller.is_bridge_active())

        # Acquire the following long edge and account for its second corner.
        command = controller.step(0.5, line(top20_seg_fill=0.40, curve=0.10), -math.radians(100.0), (0.10, 0.0))
        self.assertEqual(command.state, controller.LEG)
        self.assertFalse(controller.is_bridge_active())
        self.assertEqual(controller.turn_count, 2)
        self.assertEqual(controller.leg_index, 2)

    def test_turn_does_not_safe_stop_when_imu_has_startup_lag(self):
        controller = Stage2HybridController(self.cfg)
        controller.reset('clockwise', 0.0, (0.0, 0.0), 0.0)
        no_exit_yet = line(apex_left30_fill=0.10, apex_right30_fill=0.10)

        # The drive/relay can take longer than a second to start yawing. It
        # must keep commanding the committed entry turn, not latch SAFE_STOP.
        command = controller.step(2.0, no_exit_yet, 0.0, (0.08, 0.0))

        self.assertEqual(command.state, controller.ENTRY)
        self.assertFalse(command.safe_stop)
        self.assertGreater(command.angular, 0.0)

    def test_vision_loss_is_bounded_not_blind_forever(self):
        controller = self.new_leg_controller()
        lost = line(valid=False, age=2.0, confidence=0.0, valid_rows=0)
        command = controller.step(0.1, lost, 0.0, (0.01, 0.0))
        self.assertEqual(command.state, controller.RECOVER)
        controller.step(0.2, lost, 0.0, (0.20, 0.0))
        command = controller.step(0.3, lost, 0.0, (0.30, 0.0))
        self.assertTrue(command.safe_stop)
        self.assertEqual(controller.safe_reason, 'vision_lost_distance_limit')

    def test_last_leg_completes_after_four_turns(self):
        controller = self.new_leg_controller()
        controller.turn_count = 4
        controller.leg_index = 4
        controller.path_m = 1.01
        command = controller.step(0.1, line(), 0.0, (0.01, 0.0))
        self.assertTrue(command.completed)


if __name__ == '__main__':
    unittest.main()
