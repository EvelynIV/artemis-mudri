from __future__ import annotations
"""仿真结果对象。"""

from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class SimulationEvent:
    """仿真过程中记录的关键事件。"""
    name: str
    timestamp_s: float
    path_index: int


@dataclass(frozen=True)
class SimulationSummary:
    """一次仿真的汇总统计结果。"""
    reached_goal: bool
    elapsed_time_s: float
    route_length_m: float
    max_cross_track_error_m: float
    rms_cross_track_error_m: float
    final_pose: tuple[float, float, float]
    events: tuple[SimulationEvent, ...]
    total_steps: int = 0

    def to_dict(self) -> dict[str, Any]:
        """将结果序列化为可写入 JSON 的字典。"""
        return {
            "reached_goal": self.reached_goal,
            "elapsed_time_s": self.elapsed_time_s,
            "route_length_m": self.route_length_m,
            "max_cross_track_error_m": self.max_cross_track_error_m,
            "rms_cross_track_error_m": self.rms_cross_track_error_m,
            "final_pose": self.final_pose,
            "events": [
                {
                    "name": event.name,
                    "timestamp_s": event.timestamp_s,
                    "path_index": event.path_index,
                }
                for event in self.events
            ],
            "total_steps": self.total_steps,
        }
