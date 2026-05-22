from __future__ import annotations
"""车辆执行器命令与编码器标定。"""

from dataclasses import dataclass


@dataclass(frozen=True)
class EncoderConfig:
    """C 端编码器与距离标定。"""

    wheel_diameter_cm: float = 6.5
    cm_to_pulse: float = 10.62
    control_tick_s: float = 0.02

    @property
    def wheel_radius_m(self) -> float:
        return self.wheel_diameter_cm / 200.0


@dataclass(frozen=True)
class DifferentialMotorCommand:
    """后双驱目标速度命令，单位为编码器脉冲/20ms。"""

    velocity: float
    turn: float
    rear_left_target_speed: float
    rear_right_target_speed: float


@dataclass(frozen=True)
class EncoderReading:
    """20ms tick 下的编码器读数。"""

    rear_left_pulses: float
    rear_right_pulses: float
    rear_left_total_pulses: float
    rear_right_total_pulses: float
    rear_left_measure_speed: float
    rear_right_measure_speed: float
    forward_distance_cm: float
