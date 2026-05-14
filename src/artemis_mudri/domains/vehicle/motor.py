from __future__ import annotations
"""对齐 artemis-m0 的电机速度环与编码器标定。"""

from dataclasses import dataclass
from enum import IntEnum

import numpy as np


class MotorMode(IntEnum):
    """对齐 C 端 pid_mode_t 的电机模式。"""

    STOP = 0
    SPEED = 1
    DIR = 2
    TRACK = 3
    DEG = 4


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
class IncrementalMotorPidConfig:
    """C 端增量式电机 PID 参数。"""

    kp: float = 20.0
    ki: float = 10.0
    kd: float = 3.0
    output_max: float = 999.0
    output_min: float = -999.0


@dataclass(frozen=True)
class MotorPidSnapshot:
    """单个电机速度环的调试快照。"""

    target_speed: float
    measure_speed: float
    output: float
    mode: MotorMode


class IncrementalMotorPid:
    """对齐 bsp_pid.c 的增量式速度 PID。"""

    def __init__(self, config: IncrementalMotorPidConfig | None = None) -> None:
        self.config = config or IncrementalMotorPidConfig()
        self.target_speed = 0.0
        self.measure_speed = 0.0
        self.error = 0.0
        self.last_error = 0.0
        self.second_last_error = 0.0
        self.output = 0.0
        self.mode = MotorMode.SPEED

    def reset(self) -> None:
        """重置 PID 状态。"""

        self.target_speed = 0.0
        self.measure_speed = 0.0
        self.error = 0.0
        self.last_error = 0.0
        self.second_last_error = 0.0
        self.output = 0.0
        self.mode = MotorMode.SPEED

    def compute(self, *, target_speed: float, measure_speed: float, mode: MotorMode) -> MotorPidSnapshot:
        """执行一次 C 风格增量 PID。"""

        self.target_speed = float(target_speed)
        self.measure_speed = float(measure_speed)
        self.mode = mode
        if mode == MotorMode.STOP:
            self.output = 0.0
            self.error = 0.0
            self.last_error = 0.0
            self.second_last_error = 0.0
            return self.snapshot()

        self.error = self.target_speed - self.measure_speed
        increment = (
            self.config.kp * (self.error - self.last_error)
            + self.config.ki * self.error
            + self.config.kd * (self.error - 2.0 * self.last_error + self.second_last_error)
        )
        self.output = float(np.clip(self.output + increment, self.config.output_min, self.config.output_max))
        self.second_last_error = self.last_error
        self.last_error = self.error
        return self.snapshot()

    def snapshot(self) -> MotorPidSnapshot:
        """返回当前 PID 状态。"""

        return MotorPidSnapshot(
            target_speed=self.target_speed,
            measure_speed=self.measure_speed,
            output=self.output,
            mode=self.mode,
        )


@dataclass(frozen=True)
class DifferentialMotorCommand:
    """后双驱目标速度命令。"""

    velocity: float
    turn: float
    rear_left_target_speed: float
    rear_right_target_speed: float
    rear_left_mode: MotorMode
    rear_right_mode: MotorMode


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


@dataclass(frozen=True)
class MotorDebugReading:
    """两侧电机速度环调试状态。"""

    rear_left: MotorPidSnapshot
    rear_right: MotorPidSnapshot
    rear_left_pwm: float
    rear_right_pwm: float
