from __future__ import annotations
"""循线控制器基类与通用类型。"""

from abc import ABC, abstractmethod
from dataclasses import dataclass

from artemis_mudri.domains.task import ControlSegment
from artemis_mudri.domains.vehicle.sensing import LineSensorArrayReading


@dataclass(frozen=True)
class LineFollowControlOutput:
    """循线算法输出的车体线速度与角速度。"""
    linear_speed: float
    angular_speed: float


@dataclass(frozen=True)
class LineFollowExternalSignal:
    """预留给外接控制信号的修正量。"""
    linear_speed_scale: float = 1.0
    angular_speed_bias: float = 0.0


class BaseLineFollower(ABC):
    """所有循线控制器的统一基类。"""

    @abstractmethod
    def reset(self) -> None:
        """重置控制器内部状态。"""

    @abstractmethod
    def command(
        self,
        line_reading: LineSensorArrayReading,
        segment: ControlSegment,
        external_signal: LineFollowExternalSignal | None = None,
    ) -> LineFollowControlOutput:
        """根据线传感器读数输出循线控制。"""

    def _apply_external_signal(
        self,
        output: LineFollowControlOutput,
        external_signal: LineFollowExternalSignal | None,
    ) -> LineFollowControlOutput:
        """将外接控制信号叠加到控制输出。"""
        if external_signal is None:
            return output
        return LineFollowControlOutput(
            linear_speed=output.linear_speed * external_signal.linear_speed_scale,
            angular_speed=output.angular_speed + external_signal.angular_speed_bias,
        )
