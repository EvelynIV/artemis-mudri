from __future__ import annotations
"""LQR 循线算法实现。"""

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import solve_discrete_are

from artemis_mudri.domains.task import ControlSegment
from artemis_mudri.domains.vehicle.line_follow.base import (
    BaseLineFollower,
    LineFollowControlOutput,
    LineFollowExternalSignal,
)
from artemis_mudri.domains.vehicle.sensing import LineSensorArrayReading


@dataclass(frozen=True)
class LqrLineFollowerConfig:
    """基于横向误差和误差变化率估计的离散 LQR 参数。"""

    control_timestep_s: float = 0.01
    lateral_error_weight: float = 800.0
    heading_error_weight: float = 5.0
    control_weight: float = 1.0
    turn_limit_rps: float = 2.8
    min_speed_mps: float = 0.09
    speed_reduction_gain: float = 0.18
    min_effective_speed_mps: float = 0.08
    reacquire_speed_mps: float = 0.08
    reacquire_turn_rate_rps: float = 1.6
    speed_quantization_mps: float = 1e-3
    _gain_cache: dict[float, np.ndarray] = field(default_factory=dict, init=False, repr=False, compare=False)


class LqrLineFollower(BaseLineFollower):
    """使用简化横向模型的离散 LQR 循线器。"""

    def __init__(self, config: LqrLineFollowerConfig | None = None) -> None:
        self.config = config or LqrLineFollowerConfig()
        self.last_error_m = 0.0

    def reset(self) -> None:
        """重置 LQR 内部状态。"""
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
            error_rate = (error - self.last_error_m) / self.config.control_timestep_s
            effective_speed = max(segment.nominal_speed_mps, self.config.min_effective_speed_mps)
            heading_error = error_rate / effective_speed
            gain = self._gain_for_speed(effective_speed)
            state = np.array([error, heading_error], dtype=np.float64)
            angular_speed = float(
                np.clip(
                    gain @ state,
                    -self.config.turn_limit_rps,
                    self.config.turn_limit_rps,
                )
            )
            self.last_error_m = error
            linear_speed = segment.nominal_speed_mps / (
                1.0 + self.config.speed_reduction_gain * abs(angular_speed)
            )
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

    def _gain_for_speed(self, speed_mps: float) -> np.ndarray:
        """按当前标称速度离散化横向模型，并缓存 LQR 增益。"""
        key = round(speed_mps / self.config.speed_quantization_mps) * self.config.speed_quantization_mps
        if key in self.config._gain_cache:
            return self.config._gain_cache[key]

        dt = self.config.control_timestep_s
        a_matrix = np.array(
            [
                [1.0, speed_mps * dt],
                [0.0, 1.0],
            ],
            dtype=np.float64,
        )
        b_matrix = np.array(
            [
                [0.5 * speed_mps * dt * dt],
                [dt],
            ],
            dtype=np.float64,
        )
        q_matrix = np.diag(
            [
                self.config.lateral_error_weight,
                self.config.heading_error_weight,
            ]
        ).astype(np.float64)
        r_matrix = np.array([[self.config.control_weight]], dtype=np.float64)
        p_matrix = solve_discrete_are(a_matrix, b_matrix, q_matrix, r_matrix)
        gain = np.linalg.solve(
            r_matrix + b_matrix.T @ p_matrix @ b_matrix,
            b_matrix.T @ p_matrix @ a_matrix,
        ).reshape(-1)
        self.config._gain_cache[key] = gain
        return gain
