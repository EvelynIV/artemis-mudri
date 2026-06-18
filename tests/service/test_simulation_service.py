import unittest
from math import pi
from pathlib import Path
from unittest.mock import patch

from artemis_mudri.api import JsonSimulationService
from artemis_mudri.api.json_codec import result_to_dict
from artemis_mudri.runtime import (
    FinishedResult,
    ObservationResult,
    SimulationEpisodeRunner,
    StartedResult,
    StartEpisode,
    WheelCommand,
)
from artemis_mudri.simulation.noise import load_noise_config
from artemis_mudri.track import build_default_route


REPO_ROOT = Path(__file__).resolve().parents[2]


class SimulationEpisodeRunnerTest(unittest.TestCase):
    def test_start_returns_started_and_initial_observation(self) -> None:
        runner = SimulationEpisodeRunner()

        response = runner.start(StartEpisode(control_period_s=0.01))
        runner.close()

        self.assertIsInstance(response, StartedResult)
        self.assertEqual(response.control_period_s, 0.01)
        self.assertEqual(response.observation.sequence_id, 0)
        self.assertEqual(len(response.observation.line_sensor.darkness), 8)

    def test_step_advances_time_and_returns_observation(self) -> None:
        runner = SimulationEpisodeRunner()
        runner.start(StartEpisode(max_time_s=0.03, control_period_s=0.01))

        response = runner.step(
            WheelCommand(
                sequence_id=0,
                rear_left_target_speed=7.0,
                rear_right_target_speed=7.0,
            )
        )
        runner.close()

        self.assertIsInstance(response, ObservationResult)
        observation = response.observation
        self.assertEqual(observation.sequence_id, 1)
        self.assertGreaterEqual(observation.sim_time_s, 0.01)
        self.assertGreater(observation.state.longitudinal_speed, 0.0)

    def test_stop_returns_finished_summary(self) -> None:
        runner = SimulationEpisodeRunner()
        runner.start(StartEpisode(control_period_s=0.01))

        response = runner.stop("test_stop")
        runner.close()

        self.assertIsInstance(response, FinishedResult)
        self.assertEqual(response.reason, "test_stop")
        self.assertGreaterEqual(response.summary.total_steps, 0)

    def test_step_returns_finished_on_time_limit(self) -> None:
        runner = SimulationEpisodeRunner()
        runner.start(StartEpisode(max_time_s=0.01, control_period_s=0.01))

        response = runner.step(
            WheelCommand(
                sequence_id=0,
                rear_left_target_speed=7.0,
                rear_right_target_speed=7.0,
            )
        )
        runner.close()

        self.assertIsInstance(response, FinishedResult)
        self.assertEqual(response.reason, "time_limit")

    def test_start_accepts_manual_initial_pose(self) -> None:
        runner = SimulationEpisodeRunner()

        response = runner.start(
            StartEpisode(
                control_period_s=0.01,
                initial_pose=(1.25, -0.25, pi),
            )
        )
        finished = runner.stop("test_stop")
        runner.close()

        observation = response.observation
        self.assertAlmostEqual(observation.imu.yaw, 180.0)
        self.assertAlmostEqual(observation.state.x, 1.25)
        self.assertAlmostEqual(observation.state.y, -0.25)
        self.assertAlmostEqual(observation.state.yaw, pi)
        final_x, final_y, final_yaw = finished.summary.final_pose
        self.assertAlmostEqual(final_x, 1.25)
        self.assertAlmostEqual(final_y, -0.25)
        self.assertAlmostEqual(final_yaw, pi)

    def test_start_accepts_initial_progress_index(self) -> None:
        runner = SimulationEpisodeRunner()
        route = build_default_route()
        initial_progress_index = 90
        point = route.path.points[initial_progress_index]
        yaw = route.path.headings[initial_progress_index]

        response = runner.start(
            StartEpisode(
                control_period_s=0.01,
                initial_pose=(float(point[0]), float(point[1]), float(yaw)),
                initial_progress_index=initial_progress_index,
            )
        )
        finished = runner.stop("test_stop")
        runner.close()

        events = response.observation.step_events
        self.assertEqual(len(events), 1)
        event = events[0]
        self.assertEqual(event.event_id, 1)
        self.assertEqual(event.namespace, "path")
        self.assertEqual(event.type, "checkpoint")
        self.assertEqual(event.severity, "info")
        self.assertEqual(event.labels["name"], route.path.events[0].name)
        self.assertEqual(event.metrics["path_index"], float(route.path.events[0].index))
        summary_event = finished.summary.events[0]
        self.assertEqual(summary_event.event_id, event.event_id)
        self.assertEqual(summary_event.labels["name"], route.path.events[0].name)

    def test_viewer_state_publisher_is_called_when_enabled(self) -> None:
        publisher = _FakeStatePublisher()
        runner = SimulationEpisodeRunner(state_publisher=publisher)

        runner.start(StartEpisode(max_time_s=0.03, control_period_s=0.01))
        runner.step(
            WheelCommand(
                sequence_id=0,
                rear_left_target_speed=7.0,
                rear_right_target_speed=7.0,
            )
        )
        runner.stop("test_stop")
        runner.close()

        self.assertEqual(publisher.sequence_ids, [0, 1])
        self.assertTrue(publisher.closed)

    def test_render_episode_reuses_simulation_between_starts(self) -> None:
        created: list[_FakeRenderEpisode] = []

        def create_episode(*args, **kwargs) -> _FakeRenderEpisode:
            del args, kwargs
            episode = _FakeRenderEpisode()
            created.append(episode)
            return episode

        runner = SimulationEpisodeRunner(default_render=True)
        with patch("artemis_mudri.runtime.session.DifferentialSimulation", side_effect=create_episode):
            first = runner._episode_from_start(
                StartEpisode(initial_progress_index=12),
            )
            second = runner._episode_from_start(
                StartEpisode(initial_progress_index=34),
            )
            runner.close()

        self.assertIs(first, second)
        self.assertEqual(len(created), 1)
        self.assertEqual(first.reset_progress_indices, [34])
        self.assertEqual(first.close_count, 1)

    def test_noise_config_is_accepted(self) -> None:
        runner = SimulationEpisodeRunner(
            noise_config=load_noise_config(REPO_ROOT / "examples" / "configs" / "noise" / "weak.yaml")
        )

        response = runner.start(StartEpisode(control_period_s=0.01))
        runner.close()

        self.assertIsInstance(response, StartedResult)

    def test_none_noise_config_matches_default_zero_initial_yaw(self) -> None:
        clean_runner = SimulationEpisodeRunner()
        none_runner = SimulationEpisodeRunner(
            noise_config=load_noise_config(REPO_ROOT / "examples" / "configs" / "noise" / "none.yaml")
        )

        clean_response = clean_runner.start(StartEpisode(control_period_s=0.01))
        none_response = none_runner.start(StartEpisode(control_period_s=0.01))
        clean_runner.close()
        none_runner.close()

        self.assertAlmostEqual(clean_response.observation.imu.yaw, 0.0)
        self.assertAlmostEqual(
            none_response.observation.imu.yaw,
            clean_response.observation.imu.yaw,
        )

    def test_result_codec_keeps_public_finished_shape(self) -> None:
        runner = SimulationEpisodeRunner()
        runner.start(StartEpisode(control_period_s=0.01))
        result = runner.stop("test_stop")
        runner.close()

        response = result_to_dict(result)

        self.assertEqual(response["type"], "finished")
        self.assertEqual(response["finished"]["reason"], "test_stop")
        self.assertEqual(response["finished"]["final_step_trace"]["reason"], "test_stop")
        self.assertFalse(response["finished"]["final_step_trace"]["terminated"])
        self.assertTrue(response["finished"]["final_step_trace"]["truncated"])


class JsonSimulationServiceTest(unittest.TestCase):
    def test_dispatcher_runs_start_step_stop(self) -> None:
        runner = SimulationEpisodeRunner()
        service = JsonSimulationService(runner)

        started = service.handle({"type": "start", "max_time_s": 0.03, "control_period_s": 0.01})
        observation = service.handle(
            {
                "type": "step",
                "sequence_id": 0,
                "rear_left_target_speed": 7.0,
                "rear_right_target_speed": 7.0,
            }
        )
        finished = service.handle({"type": "stop", "reason": "test_stop"})
        runner.close()

        self.assertEqual(started["type"], "started")
        self.assertEqual(observation["type"], "observation")
        self.assertEqual(finished["type"], "finished")

    def test_dispatcher_rejects_invalid_message_without_crashing(self) -> None:
        service = JsonSimulationService(SimulationEpisodeRunner())

        response = service.handle({"type": "unknown"})

        self.assertEqual(response["type"], "error")
        self.assertIn("Unsupported request type", response["error"])

    def test_dispatcher_accepts_json_initial_pose(self) -> None:
        service = JsonSimulationService(SimulationEpisodeRunner())

        response = service.handle(
            {
                "type": "start",
                "control_period_s": 0.01,
                "initial_pose": {"x_m": 1.25, "y_m": -0.25, "yaw_rad": pi},
            }
        )
        service.runner.close()

        self.assertAlmostEqual(response["observation"]["pose"]["x_m"], 1.25)
        self.assertAlmostEqual(response["observation"]["pose"]["y_m"], -0.25)
        self.assertAlmostEqual(response["observation"]["pose"]["yaw_rad"], pi)


class _FakeRenderEpisode:
    def __init__(self) -> None:
        self.reset_progress_indices: list[int] = []
        self.close_count = 0

    def viewer_is_running(self) -> bool:
        return True

    def reset(
        self,
        *,
        random_seed: int | None = None,
        initial_pose: tuple[float, float, float] | None = None,
        initial_progress_index: int = 0,
    ) -> None:
        del random_seed, initial_pose
        self.reset_progress_indices.append(initial_progress_index)

    def close_viewer(self) -> None:
        self.close_count += 1


class _FakeStatePublisher:
    def __init__(self) -> None:
        self.sequence_ids: list[int] = []
        self.closed = False

    def publish(self, simulation) -> None:
        self.sequence_ids.append(int(simulation.sequence_id))

    def close(self) -> None:
        self.closed = True


if __name__ == "__main__":
    unittest.main()
