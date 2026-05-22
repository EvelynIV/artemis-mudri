import unittest
from math import pi

import numpy as np

from artemis_mudri.simulation.episode import DifferentialSimulation
from artemis_mudri.simulation.presets import DEFAULT_SIMULATION_PRESET
from artemis_mudri.track import build_default_route
from artemis_mudri.vehicle import DifferentialMotorCommand


def _command(left_speed: float, right_speed: float) -> DifferentialMotorCommand:
    return DifferentialMotorCommand(
        velocity=0.5 * (left_speed + right_speed),
        turn=0.5 * (right_speed - left_speed),
        rear_left_target_speed=left_speed,
        rear_right_target_speed=right_speed,
    )


class DifferentialSimulationTest(unittest.TestCase):
    def test_default_preset_builds_simulation_with_reference_route_visual(self) -> None:
        simulation = DEFAULT_SIMULATION_PRESET.build_simulation()

        self.assertEqual(DEFAULT_SIMULATION_PRESET.name, "default")
        self.assertGreater(simulation.route.path.total_length, 0.0)
        self.assertIn("reference_route_0", simulation.model.geom("reference_route_0").name)

    def test_default_initial_yaw_is_zero(self) -> None:
        simulation = DifferentialSimulation(route=build_default_route(), random_seed=7)

        self.assertAlmostEqual(simulation.current_state().yaw, 0.0)

    def test_accepts_manual_initial_pose(self) -> None:
        simulation = DifferentialSimulation(
            route=build_default_route(),
            initial_pose=(1.0, -0.5, pi / 2),
        )

        state = simulation.current_state()
        self.assertAlmostEqual(state.x, 1.0)
        self.assertAlmostEqual(state.y, -0.5)
        self.assertAlmostEqual(state.yaw, pi / 2)

    def test_zero_motor_command_keeps_vehicle_parked(self) -> None:
        simulation = DifferentialSimulation(route=build_default_route())
        start_x = simulation.current_state().x
        start_y = simulation.current_state().y
        simulation.step_control_command(_command(0.0, 0.0))
        self.assertAlmostEqual(simulation.current_state().x, start_x)
        self.assertAlmostEqual(simulation.current_state().y, start_y)

    def test_equal_motor_command_advances_forward(self) -> None:
        simulation = DifferentialSimulation(route=build_default_route())
        start_x = simulation.current_state().x
        observation = simulation.step_control_command(_command(17.5, 17.5))
        self.assertGreater(simulation.current_state().x, start_x)
        self.assertEqual(observation.sequence_id, 1)

    def test_opposite_motor_command_turns_in_place(self) -> None:
        simulation = DifferentialSimulation(route=build_default_route())
        observation = simulation.step_control_command(_command(-17.5, 17.5))
        self.assertGreater(observation.imu.yaw_rate, 0.0)

    def test_cross_track_error_is_signed(self) -> None:
        route = build_default_route()
        simulation = DifferentialSimulation(route=route)
        start = route.path.points[0]

        simulation._update_metrics(start + np.array([0.0, 0.1], dtype=np.float64))
        left_error = simulation.cross_track_errors[-1]
        simulation._update_metrics(start + np.array([0.0, -0.1], dtype=np.float64))
        right_error = simulation.cross_track_errors[-1]

        self.assertGreater(left_error, 0.0)
        self.assertLess(right_error, 0.0)

    def test_summary_uses_cross_track_error_magnitude(self) -> None:
        simulation = DifferentialSimulation(route=build_default_route())
        simulation.cross_track_errors.extend([-0.3, 0.4])

        summary = simulation.summary()

        self.assertAlmostEqual(summary.max_cross_track_error_m, 0.4)
        self.assertGreater(summary.rms_cross_track_error_m, 0.0)


if __name__ == "__main__":
    unittest.main()
