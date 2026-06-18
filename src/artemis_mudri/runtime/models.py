from __future__ import annotations
"""Runtime request and result objects."""

from dataclasses import dataclass
from typing import TypeAlias

from artemis_mudri.simulation import SimulationObservation, SimulationSummary


@dataclass(frozen=True)
class StartEpisode:
    """Parameters required to start one simulation episode."""

    max_time_s: float | None = None
    control_period_s: float | None = None
    initial_pose: tuple[float, float, float] | None = None
    initial_progress_index: int = 0
    random_seed: int | None = None


@dataclass(frozen=True)
class WheelCommand:
    """One tick of target wheel speeds."""

    rear_left_target_speed: float
    rear_right_target_speed: float
    sequence_id: int | None = None


@dataclass(frozen=True)
class StartedResult:
    """Episode start result."""

    time_limit_s: float
    control_period_s: float
    observation: SimulationObservation


@dataclass(frozen=True)
class ObservationResult:
    """Regular step observation result."""

    observation: SimulationObservation


@dataclass(frozen=True)
class FinishedResult:
    """Terminal episode result."""

    reason: str
    summary: SimulationSummary


EpisodeResult: TypeAlias = StartedResult | ObservationResult | FinishedResult
