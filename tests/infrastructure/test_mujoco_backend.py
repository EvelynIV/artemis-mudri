import unittest

try:
    from artemis_mudri.domains.task import build_route_plan
    from artemis_mudri.infrastructure.mujoco.backend import DifferentialMuJoCoSimulation
except ModuleNotFoundError:
    DifferentialMuJoCoSimulation = None


@unittest.skipIf(DifferentialMuJoCoSimulation is None, "MuJoCo dependency is not installed")
class DifferentialMuJoCoSimulationTest(unittest.TestCase):
    def test_random_seed_makes_initial_yaw_reproducible(self) -> None:
        first = DifferentialMuJoCoSimulation(
            route=build_route_plan("1"),
            random_seed=7,
            initial_yaw_noise_deg=5.0,
        )
        second = DifferentialMuJoCoSimulation(
            route=build_route_plan("1"),
            random_seed=7,
            initial_yaw_noise_deg=5.0,
        )
        self.assertAlmostEqual(first.current_state().yaw, second.current_state().yaw)

    def test_zero_motor_command_keeps_vehicle_parked(self) -> None:
        simulation = DifferentialMuJoCoSimulation(route=build_route_plan("1"))
        start_x = simulation.current_state().x
        start_y = simulation.current_state().y
        simulation.step_motor_command(0.0, 0.0)
        self.assertAlmostEqual(simulation.current_state().x, start_x)
        self.assertAlmostEqual(simulation.current_state().y, start_y)

    def test_equal_motor_command_advances_forward(self) -> None:
        simulation = DifferentialMuJoCoSimulation(route=build_route_plan("1"))
        start_x = simulation.current_state().x
        observation = simulation.step_motor_command(0.5, 0.5)
        self.assertGreater(simulation.current_state().x, start_x)
        self.assertEqual(observation.sequence_id, 1)

    def test_opposite_motor_command_turns_in_place(self) -> None:
        simulation = DifferentialMuJoCoSimulation(route=build_route_plan("1"))
        observation = simulation.step_motor_command(-0.5, 0.5)
        self.assertGreater(observation.imu.yaw_rate, 0.0)


if __name__ == "__main__":
    unittest.main()
