import unittest

from racing_stage2.mppi_straight_avoidance import (
    MppiStraightAvoidanceConfig,
    MppiStraightAvoidanceController,
)


class MppiStraightAvoidanceTest(unittest.TestCase):
    def setUp(self):
        self.controller = MppiStraightAvoidanceController(
            MppiStraightAvoidanceConfig(
                horizon_steps=50,
                batch_size=160,
                linear_speed_mps=0.42,
            )
        )
        self.corridor = {'left': 0.62, 'right': -0.62}

    def test_selects_a_nonzero_safe_turn_for_center_obstacle(self):
        command = self.controller.step(
            now_sec=0.0, yaw=0.0, line_heading=0.0, line_speed=0.66,
            obstacle={'center_x': 0.85, 'center_y': 0.0, 'span': 0.08},
            corridor=self.corridor,
        )
        self.assertIsNotNone(command)
        self.assertEqual(command.state, 'avoiding')
        self.assertGreater(command.linear, 0.0)
        self.assertNotEqual(command.angular, 0.0)
        self.assertGreater(command.min_clearance_m, 0.0)

    def test_blocks_when_corridor_has_no_collision_free_rollout(self):
        command = self.controller.step(
            now_sec=0.0, yaw=0.0, line_heading=0.0, line_speed=0.66,
            obstacle={'center_x': 0.35, 'center_y': 0.0, 'span': 0.10},
            corridor={'left': 0.30, 'right': -0.30},
        )
        self.assertIsNotNone(command)
        self.assertEqual(command.state, 'blocked')
        self.assertEqual(command.linear, 0.0)


if __name__ == '__main__':
    unittest.main()
