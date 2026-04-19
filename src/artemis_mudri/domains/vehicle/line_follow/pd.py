from __future__ import annotations
"""PD 循线算法实现。"""

from dataclasses import dataclass

import numpy as np

from artemis_mudri.domains.task import ControlSegment
from artemis_mudri.domains.vehicle.line_follow.base import (
    BaseLineFollower,
    LineFollowControlOutput,
    LineFollowExternalSignal,
)
from artemis_mudri.domains.vehicle.sensing import LineSensorArrayReading


@dataclass(frozen=True)
class PdLineFollowerConfig:
    """PD 循线器参数。"""
    control_timestep_s: float = 0.01
    gain: float = 28.0
    derivative_gain: float = 2.4
    turn_limit_rps: float = 2.8
    min_speed_mps: float = 0.09
    reacquire_speed_mps: float = 0.08
    reacquire_turn_rate_rps: float = 1.6


class PdLineFollower(BaseLineFollower):
    """基于横向误差的 PD 循线器。"""

    def __init__(self, config: PdLineFollowerConfig | None = None) -> None:
        self.config = config or PdLineFollowerConfig()
        self.last_error_m = 0.0

    def reset(self) -> None:
        """重置 PD 内部状态。"""
        self.last_error_m = 0.0

    def command(
        self,
        line_reading: LineSensorArrayReading,
        segment: ControlSegment,
        external_signal: LineFollowExternalSignal | None = None,
    ) -> LineFollowControlOutput:
        """根据数字量线传感器读数输出循线控制。"""
        if line_reading.line_detected and line_reading.lateral_error_m is not None:
            error = line_reading.lateral_error_m
            derivative = (error - self.last_error_m) / self.config.control_timestep_s
            angular_speed = float(
                np.clip(
                    self.config.gain * error
                    + self.config.derivative_gain * derivative,
                    -self.config.turn_limit_rps,
                    self.config.turn_limit_rps,
                )
            )
            self.last_error_m = error
            linear_speed = segment.nominal_speed_mps / (1.0 + 0.18 * abs(angular_speed))
            linear_speed = max(self.config.min_speed_mps, linear_speed)
            output = LineFollowControlOutput(
                linear_speed=float(linear_speed),
                angular_speed=angular_speed,
            )
            return self._apply_external_signal(output, external_signal)

        self.last_error_m = 0.0
        output = LineFollowControlOutput(
            linear_speed=min(segment.nominal_speed_mps, self.config.reacquire_speed_mps),
            angular_speed=float(np.sign(segment.search_turn_direction)) * self.config.reacquire_turn_rate_rps,
        )
        return self._apply_external_signal(output, external_signal)
