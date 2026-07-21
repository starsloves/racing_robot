import math
import unittest

from racing_stage2.straight_avoidance import StraightAvoidanceController


class StraightAvoidanceTest(unittest.TestCase):
    def setUp(self):
        self.controller = StraightAvoidanceController(
            enabled=True,
            angular_speed=0.80,
            yaw_offset_deg=30.0,
            yaw_tolerance_deg=0.5,
            start_heading_tolerance_deg=15.0,
            max_turn_travel_deg=35.0,
        )

    def test_plan_keeps_speed_and_completes_shift_before_obstacle(self):
        plan = StraightAvoidanceController.plan_for_offset(
            lateral_shift_m=0.10,
            obstacle_distance_m=1.50,
            linear_speed=0.65,
            angular_speed=0.80,
            max_yaw_offset_rad=math.radians(30.0),
            forward_margin_m=0.25,
        )
        self.assertIsNotNone(plan)
        self.assertLess(plan.required_forward_m + 0.25, plan.obstacle_distance_m)
        self.assertLess(plan.yaw_offset_rad, math.radians(30.0))

        first = self.controller.step(
            yaw=0.0, line_heading=0.0, line_speed=0.65, plan=plan,
        )
        self.assertEqual(first.state, 'turn_away')
        self.assertAlmostEqual(first.linear, 0.65)
        self.assertGreater(first.angular, 0.0)

        away = plan.yaw_offset_rad
        reverse = self.controller.step(
            yaw=away, line_heading=0.0, line_speed=0.65, plan=None,
        )
        self.assertEqual(reverse.state, 'turn_reverse')
        self.assertAlmostEqual(reverse.linear, 0.65)
        self.assertLess(reverse.angular, 0.0)

        returning = self.controller.step(
            yaw=-away, line_heading=0.0, line_speed=0.65, plan=None,
        )
        self.assertEqual(returning.state, 'return_heading')
        self.assertAlmostEqual(returning.linear, 0.65)
        self.assertGreater(returning.angular, 0.0)

    def test_plan_rejects_late_or_overwide_shift(self):
        late = StraightAvoidanceController.plan_for_offset(
            lateral_shift_m=0.10,
            obstacle_distance_m=0.80,
            linear_speed=0.65,
            angular_speed=0.80,
            max_yaw_offset_rad=math.radians(30.0),
            forward_margin_m=0.25,
        )
        too_wide = StraightAvoidanceController.plan_for_offset(
            lateral_shift_m=0.30,
            obstacle_distance_m=2.00,
            linear_speed=0.65,
            angular_speed=0.80,
            max_yaw_offset_rad=math.radians(30.0),
            forward_margin_m=0.25,
        )
        self.assertIsNone(late)
        self.assertIsNone(too_wide)


if __name__ == '__main__':
    unittest.main()
