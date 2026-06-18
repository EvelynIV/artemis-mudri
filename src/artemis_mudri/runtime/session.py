from __future__ import annotations
"""Transport-neutral simulation session runtime."""

import logging
import threading

from artemis_mudri.runtime.models import (
    FinishedResult,
    ObservationResult,
    StartedResult,
    StartEpisode,
    WheelCommand,
)
from artemis_mudri.simulation import (
    DEFAULT_SIMULATION_PRESET,
    DifferentialSimulation,
    MotorDriverConfig,
    SimulationPreset,
)
from artemis_mudri.simulation.noise import NoiseConfig
from artemis_mudri.simulation.state_publisher import SimulationStatePublisher
from artemis_mudri.vehicle import DifferentialMotorCommand

logger = logging.getLogger(__name__)


class SimulationEpisodeRunner:
    """Manage one active simulation episode."""

    def __init__(
        self,
        *,
        default_render: bool = False,
        motor_driver: MotorDriverConfig | None = None,
        noise_config: NoiseConfig | None = None,
        state_publisher: SimulationStatePublisher | None = None,
        preset: SimulationPreset = DEFAULT_SIMULATION_PRESET,
    ) -> None:
        self.default_render = default_render
        self.motor_driver = motor_driver or MotorDriverConfig()
        self.noise_config = noise_config
        self.state_publisher = state_publisher
        self.preset = preset
        self._lock = threading.RLock()
        self._render_episode: DifferentialSimulation | None = None
        self._episode: DifferentialSimulation | None = None
        self._time_budget_s = 0.0
        self._control_period_s = 0.0

    def start(self, request: StartEpisode) -> StartedResult:
        """Start a new episode and return its initial observation."""

        with self._lock:
            if self._episode is not None and not self.default_render:
                self._episode.close_viewer()

            route = self.preset.build_route()
            episode = self._episode_from_start(request)
            self._episode = episode
            self._control_period_s = float(request.control_period_s or episode.timestep_s)
            self._time_budget_s = float(request.max_time_s or route.time_limit_s)

            if self.default_render:
                episode.open_viewer()

            observation = episode.observe()
            self._publish_viewer_state(episode)
            logger.info(
                "Started simulation episode render=%s control_period=%s max_time=%s",
                self.default_render,
                self._control_period_s,
                self._time_budget_s,
            )
            return StartedResult(
                time_limit_s=self._time_budget_s,
                control_period_s=self._control_period_s,
                observation=observation,
            )

    def step(self, command: WheelCommand) -> ObservationResult | FinishedResult:
        """Apply one wheel command and return an observation or terminal result."""

        with self._lock:
            episode = self._require_episode()
            velocity = 0.5 * (command.rear_left_target_speed + command.rear_right_target_speed)
            turn = 0.5 * (command.rear_right_target_speed - command.rear_left_target_speed)
            observation = episode.step_control_command(
                DifferentialMotorCommand(
                    velocity=velocity,
                    turn=turn,
                    rear_left_target_speed=command.rear_left_target_speed,
                    rear_right_target_speed=command.rear_right_target_speed,
                ),
                dt=self._control_period_s,
            )
            self._publish_viewer_state(episode)

            if episode.data.time >= self._time_budget_s:
                return self._finish_locked("time_limit")
            if not episode.viewer_is_running():
                return self._finish_locked("viewer_closed")
            return ObservationResult(observation=observation)

    def stop(self, reason: str = "client_stopped") -> FinishedResult:
        """Stop the active episode and return its summary."""

        with self._lock:
            self._require_episode()
            return self._finish_locked(reason or "client_stopped")

    def close(self) -> None:
        """Release any viewer and publisher resources."""

        with self._lock:
            if self._episode is not None and not self.default_render:
                self._episode.close_viewer()
            self._episode = None
            if self._render_episode is not None:
                self._render_episode.close_viewer()
                self._render_episode = None
            if self.state_publisher is not None:
                self.state_publisher.close()

    def _episode_from_start(self, request: StartEpisode) -> DifferentialSimulation:
        route = self.preset.build_route()
        if not self.default_render:
            return DifferentialSimulation(
                route=route,
                route_resolution=self.preset.model_resolution,
                motor_driver=self.motor_driver,
                noise_config=self.noise_config,
                random_seed=request.random_seed,
                initial_pose=request.initial_pose,
                initial_progress_index=request.initial_progress_index,
            )

        if self._render_episode is not None and not self._render_episode.viewer_is_running():
            self._render_episode.close_viewer()
            self._render_episode = None
        if self._render_episode is None:
            self._render_episode = DifferentialSimulation(
                route=route,
                route_resolution=self.preset.model_resolution,
                motor_driver=self.motor_driver,
                noise_config=self.noise_config,
                random_seed=request.random_seed,
                initial_pose=request.initial_pose,
                initial_progress_index=request.initial_progress_index,
            )
        else:
            self._render_episode.reset(
                random_seed=request.random_seed,
                initial_pose=request.initial_pose,
                initial_progress_index=request.initial_progress_index,
            )
        return self._render_episode

    def _finish_locked(self, reason: str) -> FinishedResult:
        episode = self._require_episode()
        summary = episode.summary()
        if not self.default_render:
            episode.close_viewer()
        self._episode = None
        return FinishedResult(reason=reason, summary=summary)

    def _require_episode(self) -> DifferentialSimulation:
        if self._episode is None:
            raise RuntimeError("No active simulation episode. Send a start request first.")
        return self._episode

    def _publish_viewer_state(self, episode: DifferentialSimulation) -> None:
        if self.state_publisher is None:
            return
        self.state_publisher.publish(episode)
