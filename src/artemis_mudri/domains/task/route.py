from __future__ import annotations
"""任务路线与控制段的数据结构。"""

from dataclasses import dataclass
from typing import Literal

from artemis_mudri.domains.geometry import FloatArray
from artemis_mudri.domains.track import ReferencePath

ControlMode = Literal["waypoint", "line_follow"]
PathSegmentType = Literal["line", "arc"]


@dataclass(frozen=True)
class ControlSegment:
    """单段控制目标。"""
    event_name: str
    mode: ControlMode
    target_position: FloatArray
    nominal_speed_mps: float
    arrival_tolerance_m: float
    search_turn_direction: float = 0.0


@dataclass(frozen=True)
class RoutePlan:
    """任务的完整路线与控制计划。"""
    task_id: str
    label: str
    description: str
    path: ReferencePath
    control_segments: tuple[ControlSegment, ...]
    time_limit_s: float

    @property
    def start_pose(self) -> tuple[float, float, float]:
        return self.path.start_pose
