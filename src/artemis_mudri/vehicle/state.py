from __future__ import annotations
"""车辆状态与传感器读数。"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray


@dataclass(frozen=True)
class VehicleState:
    """车辆在平面中的状态。"""

    x: float
    y: float
    yaw: float
    longitudinal_speed: float = 0.0
    lateral_speed: float = 0.0
    yaw_rate: float = 0.0
    slip_angle: float = 0.0
    steering_angle: float = 0.0

    @property
    def position(self) -> NDArray[np.float64]:
        return np.array([self.x, self.y], dtype=np.float64)

    @property
    def linear_speed(self) -> float:
        return float(np.hypot(self.longitudinal_speed, self.lateral_speed))

    @property
    def angular_speed(self) -> float:
        return self.yaw_rate


@dataclass(frozen=True)
class GyroReading:
    """陀螺仪读数。"""

    yaw: float
    yaw_rate: float
