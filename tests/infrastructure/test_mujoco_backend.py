import unittest

try:
    from artemis_mudri.domains.task import build_route_plan
    from artemis_mudri.infrastructure.mujoco.backend import MuJoCoTeachingDemo
except ModuleNotFoundError:
    MuJoCoTeachingDemo = None


@unittest.skipIf(MuJoCoTeachingDemo is None, "MuJoCo dependency is not installed")
class MuJoCoTeachingDemoTest(unittest.TestCase):
    def test_task1_finishes_headless_with_default_ackermann(self) -> None:
        demo = MuJoCoTeachingDemo(route=build_route_plan("1"))
        summary = demo.run(max_time_s=6.0, render=False)
        self.assertTrue(summary.reached_goal)
        self.assertLess(summary.max_cross_track_error_m, 0.08)
        self.assertEqual(summary.events[-1].name, "B")

    def test_task3_finishes_headless_with_ackermann_corner_drift(self) -> None:
        demo = MuJoCoTeachingDemo(route=build_route_plan("3"), chassis="ackermann", drift_mode="corner")
        summary = demo.run(max_time_s=30.0, render=False)
        self.assertTrue(summary.reached_goal)
        self.assertLess(summary.max_cross_track_error_m, 0.12)
        self.assertEqual([event.name for event in summary.events], ["C", "B", "D", "A"])

    def test_task3_finishes_headless_with_differential_compatibility(self) -> None:
        demo = MuJoCoTeachingDemo(route=build_route_plan("3"), chassis="differential", drift_mode="off")
        summary = demo.run(max_time_s=30.0, render=False)
        self.assertTrue(summary.reached_goal)
        self.assertLess(summary.max_cross_track_error_m, 0.08)
        self.assertEqual([event.name for event in summary.events], ["C", "B", "D", "A"])


if __name__ == "__main__":
    unittest.main()
