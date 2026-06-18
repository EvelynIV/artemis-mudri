import tempfile
import unittest
from pathlib import Path

import numpy as np

from artemis_mudri.simulation.noise import (
    ActuatorNoiseConfig,
    ImuNoiseConfig,
    LineSensorNoiseConfig,
    NoiseConfig,
    enabled_noise_modules,
    load_noise_config,
)
from artemis_mudri.simulation.episode import DifferentialSimulation
from artemis_mudri.track import build_default_route
from artemis_mudri.vehicle import DifferentialMotorCommand


REPO_ROOT = Path(__file__).resolve().parents[2]


def _command(left_speed: float, right_speed: float) -> DifferentialMotorCommand:
    return DifferentialMotorCommand(
        velocity=0.5 * (left_speed + right_speed),
        turn=0.5 * (right_speed - left_speed),
        rear_left_target_speed=left_speed,
        rear_right_target_speed=right_speed,
    )


class NoiseConfigTest(unittest.TestCase):
    def test_noise_presets_load(self) -> None:
        for preset in ("none", "weak", "strong"):
            config = load_noise_config(REPO_ROOT / "examples" / "configs" / "noise" / f"{preset}.yaml")
            self.assertEqual(config.preset, preset)

    def test_none_preset_disables_all_modules(self) -> None:
        config = load_noise_config(REPO_ROOT / "examples" / "configs" / "noise" / "none.yaml")
        self.assertEqual(enabled_noise_modules(config), ())

    def test_yaml_overrides_single_value(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "noise.yaml"
            path.write_text(
                """
noise:
  preset: custom
  imu:
    enabled: true
    yaw_std_deg: 1.5
""".strip(),
                encoding="utf-8",
            )

            config = load_noise_config(path)

        self.assertTrue(config.imu.enabled)
        self.assertEqual(config.imu.yaw_std_deg, 1.5)
        self.assertFalse(config.line_sensor.enabled)

    def test_unknown_key_raises_clear_error(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "noise.yaml"
            path.write_text("noise:\n  imu:\n    missing: 1\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "noise.imu.missing"):
                load_noise_config(path)

    def test_invalid_probability_raises_clear_error(self) -> None:
        with tempfile.TemporaryDirectory() as temp_dir:
            path = Path(temp_dir) / "noise.yaml"
            path.write_text("noise:\n  encoder:\n    dropout_prob: 1.5\n", encoding="utf-8")

            with self.assertRaisesRegex(ValueError, "dropout_prob"):
                load_noise_config(path)


class DifferentialSimulationNoiseTest(unittest.TestCase):
    def test_same_seed_makes_noisy_observation_reproducible(self) -> None:
        config = load_noise_config(REPO_ROOT / "examples" / "configs" / "noise" / "weak.yaml")
        first = DifferentialSimulation(route=build_default_route(), random_seed=13, noise_config=config)
        second = DifferentialSimulation(route=build_default_route(), random_seed=13, noise_config=config)

        first_observation = first.step_control_command(_command(17.5, 17.5))
        second_observation = second.step_control_command(_command(17.5, 17.5))

        np.testing.assert_allclose(first_observation.line_sensor.darkness, second_observation.line_sensor.darkness)
        self.assertAlmostEqual(first_observation.imu.yaw, second_observation.imu.yaw)
        self.assertAlmostEqual(
            first_observation.encoder.rear_left_measure_speed,
            second_observation.encoder.rear_left_measure_speed,
        )

    def test_different_seed_changes_noisy_initial_pose(self) -> None:
        config = load_noise_config(REPO_ROOT / "examples" / "configs" / "noise" / "weak.yaml")
        first = DifferentialSimulation(route=build_default_route(), random_seed=13, noise_config=config)
        second = DifferentialSimulation(route=build_default_route(), random_seed=14, noise_config=config)

        self.assertNotAlmostEqual(first.current_state().yaw, second.current_state().yaw)

    def test_line_sensor_noise_changes_darkness_and_clamps_range(self) -> None:
        route = build_default_route()
        clean = DifferentialSimulation(route=route)
        noisy = DifferentialSimulation(
            route=route,
            random_seed=3,
            noise_config=NoiseConfig(
                line_sensor=LineSensorNoiseConfig(enabled=True, darkness_std=0.5),
            ),
        )

        clean_darkness = clean.observe().line_sensor.darkness
        noisy_darkness = noisy.observe().line_sensor.darkness

        self.assertFalse(np.array_equal(clean_darkness, noisy_darkness))
        self.assertTrue(np.all(noisy_darkness >= 0.0))
        self.assertTrue(np.all(noisy_darkness <= 1.0))

    def test_imu_noise_changes_observation_not_true_pose(self) -> None:
        config = NoiseConfig(
            imu=ImuNoiseConfig(
                enabled=True,
                yaw_std_deg=0.0,
                yaw_rate_std_deg_s=0.0,
                yaw_bias_deg=10.0,
            )
        )
        simulation = DifferentialSimulation(route=build_default_route(), noise_config=config)

        true_yaw_deg = float(np.rad2deg(simulation.current_state().yaw) % 360.0)
        observation = simulation.observe()

        self.assertAlmostEqual(observation.imu.yaw, (true_yaw_deg + 10.0) % 360.0)
        self.assertAlmostEqual(float(np.rad2deg(simulation.current_state().yaw) % 360.0), true_yaw_deg)

    def test_actuator_noise_changes_trajectory(self) -> None:
        route = build_default_route()
        clean = DifferentialSimulation(route=route, random_seed=5)
        noisy = DifferentialSimulation(
            route=route,
            random_seed=5,
            noise_config=NoiseConfig(
                actuator=ActuatorNoiseConfig(
                    enabled=True,
                    left_gain_std=0.2,
                    right_gain_std=0.2,
                    command_std_pulse_per_tick=0.5,
                ),
            ),
        )

        clean.step_control_command(_command(17.5, 17.5))
        noisy.step_control_command(_command(17.5, 17.5))

        self.assertNotAlmostEqual(clean.current_state().x, noisy.current_state().x)


if __name__ == "__main__":
    unittest.main()
