import unittest

from artemis_mudri.domains.vehicle import IncrementalMotorPid, MotorMode


class IncrementalMotorPidTest(unittest.TestCase):
    def test_pid_output_follows_c_incremental_formula_and_limits(self) -> None:
        pid = IncrementalMotorPid()
        first = pid.compute(target_speed=7.0, measure_speed=0.0, mode=MotorMode.DEG)
        self.assertAlmostEqual(first.output, 231.0)
        limited = pid.compute(target_speed=100.0, measure_speed=-100.0, mode=MotorMode.DEG)
        self.assertLessEqual(limited.output, 999.0)

    def test_stop_mode_resets_output(self) -> None:
        pid = IncrementalMotorPid()
        pid.compute(target_speed=7.0, measure_speed=0.0, mode=MotorMode.TRACK)
        stopped = pid.compute(target_speed=0.0, measure_speed=5.0, mode=MotorMode.STOP)
        self.assertEqual(stopped.output, 0.0)


if __name__ == "__main__":
    unittest.main()
