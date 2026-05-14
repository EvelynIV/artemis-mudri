import unittest

from artemis_mudri.domains.vehicle import BodyMotionCommand, DifferentialDriveChassis


class ChassisModelTest(unittest.TestCase):
    def test_differential_chassis_preserves_turn_direction(self) -> None:
        chassis = DifferentialDriveChassis()
        chassis.reset((0.0, 0.0, 0.0))
        result = chassis.step(
            BodyMotionCommand(target_speed_mps=0.3, target_yaw_rate_rps=-0.7),
            0.01,
        )
        self.assertLess(result.state.yaw_rate, 0.0)
        self.assertGreater(result.actuation.left_speed, result.actuation.right_speed)


if __name__ == "__main__":
    unittest.main()
