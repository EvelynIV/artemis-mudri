from __future__ import annotations
"""车辆状态、命令与底盘模型。"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Literal

import numpy as np

from artemis_mudri.domains.geometry import FloatArray, wrap_angle

ChassisType = Literal["differential", "ackermann"]
DriftMode = Literal["off", "corner"]


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
    def position(self) -> FloatArray:
        return np.array([self.x, self.y], dtype=np.float64)

    @property
    def linear_speed(self) -> float:
        return float(np.hypot(self.longitudinal_speed, self.lateral_speed))

    @property
    def angular_speed(self) -> float:
        return self.yaw_rate


@dataclass(frozen=True)
class BodyMotionCommand:
    """控制器输出的目标车体速度。"""

    target_speed_mps: float
    target_yaw_rate_rps: float


@dataclass(frozen=True)
class DifferentialActuationCommand:
    """差速底盘执行命令。"""

    left_speed: float
    right_speed: float


@dataclass(frozen=True)
class AckermannActuationCommand:
    """阿克曼底盘执行命令。"""

    speed_mps: float
    steering_angle_rad: float


@dataclass(frozen=True)
class GyroReading:
    """陀螺仪读数。"""

    yaw: float
    yaw_rate: float


@dataclass(frozen=True)
class ChassisStepResult:
    """单步底盘推进结果。"""

    state: VehicleState
    actuation: DifferentialActuationCommand | AckermannActuationCommand


@dataclass(frozen=True)
class DifferentialDriveChassisConfig:
    """差速底盘参数。"""

    wheel_radius_m: float = 0.03
    wheel_track_m: float = 0.14


@dataclass(frozen=True)
class AckermannChassisConfig:
    """阿克曼底盘与漂移动力学参数。"""

    wheelbase_m: float = 0.11
    front_axle_to_cg_m: float = 0.055
    rear_axle_to_cg_m: float = 0.055
    max_steering_angle_rad: float = float(np.deg2rad(42.0))
    max_steering_rate_radps: float = float(np.deg2rad(300.0))
    steering_command_time_constant_s: float = 0.08
    mass_kg: float = 1.8
    yaw_inertia_kgm2: float = 0.035
    front_cornering_stiffness_nprad: float = 28.0
    rear_cornering_stiffness_nprad: float = 24.0
    front_tire_friction_coeff: float = 0.86
    rear_tire_friction_coeff: float = 0.76
    longitudinal_accel_limit_mps2: float = 12.0
    min_speed_for_steering_mps: float = 0.12
    drift_rear_grip_scale: float = 0.78
    drift_target_slip_angle_rad: float = float(np.deg2rad(3.0))
    max_lateral_speed_mps: float = 3.5
    max_yaw_rate_rps: float = 12.0


DEFAULT_DIFFERENTIAL_DRIVE_CHASSIS_CONFIG = DifferentialDriveChassisConfig()
DEFAULT_ACKERMANN_CHASSIS_CONFIG = AckermannChassisConfig()


def integrate_vehicle_state(
    state: VehicleState,
    linear_speed: float,
    angular_speed: float,
    dt: float,
) -> VehicleState:
    """用简单单轨近似推进一步车辆状态。"""

    yaw_mid = state.yaw + 0.5 * angular_speed * dt
    new_x = state.x + linear_speed * float(np.cos(yaw_mid)) * dt
    new_y = state.y + linear_speed * float(np.sin(yaw_mid)) * dt
    new_yaw = wrap_angle(state.yaw + angular_speed * dt)
    return VehicleState(
        x=new_x,
        y=new_y,
        yaw=new_yaw,
        longitudinal_speed=linear_speed,
        lateral_speed=0.0,
        yaw_rate=angular_speed,
        slip_angle=0.0,
        steering_angle=0.0,
    )


class BaseChassisModel(ABC):
    """底盘模型统一接口。"""

    def __init__(self) -> None:
        self._state = VehicleState(0.0, 0.0, 0.0)

    @property
    def state(self) -> VehicleState:
        return self._state

    def reset(self, pose: tuple[float, float, float]) -> None:
        self._state = VehicleState(*pose)

    @abstractmethod
    def step(
        self,
        command: BodyMotionCommand,
        dt: float,
        *,
        segment_mode: str | None = None,
        drift_mode: DriftMode = "off",
        gyro_reading: GyroReading | None = None,
    ) -> ChassisStepResult:
        """推进一步底盘状态。"""


class DifferentialDriveChassis(BaseChassisModel):
    """沿用原有简化运动学的差速底盘。"""

    def __init__(self, config: DifferentialDriveChassisConfig | None = None) -> None:
        super().__init__()
        self.config = config or DEFAULT_DIFFERENTIAL_DRIVE_CHASSIS_CONFIG

    def step(
        self,
        command: BodyMotionCommand,
        dt: float,
        *,
        segment_mode: str | None = None,
        drift_mode: DriftMode = "off",
        gyro_reading: GyroReading | None = None,
    ) -> ChassisStepResult:
        del segment_mode, drift_mode
        half_track = 0.5 * self.config.wheel_track_m
        yaw_rate = gyro_reading.yaw_rate if gyro_reading is not None else command.target_yaw_rate_rps
        next_state = integrate_vehicle_state(
            self._state,
            linear_speed=command.target_speed_mps,
            angular_speed=yaw_rate,
            dt=dt,
        )
        if gyro_reading is not None:
            next_state = VehicleState(
                x=next_state.x,
                y=next_state.y,
                yaw=gyro_reading.yaw,
                longitudinal_speed=next_state.longitudinal_speed,
                lateral_speed=0.0,
                yaw_rate=gyro_reading.yaw_rate,
                slip_angle=0.0,
                steering_angle=0.0,
            )
        self._state = next_state
        actuation = DifferentialActuationCommand(
            left_speed=float(
                (command.target_speed_mps - half_track * command.target_yaw_rate_rps) / self.config.wheel_radius_m
            ),
            right_speed=float(
                (command.target_speed_mps + half_track * command.target_yaw_rate_rps) / self.config.wheel_radius_m
            ),
        )
        return ChassisStepResult(state=self._state, actuation=actuation)


class AckermannChassisModel(BaseChassisModel):
    """带简化横向动力学的阿克曼底盘。"""

    def __init__(self, config: AckermannChassisConfig | None = None) -> None:
        super().__init__()
        self.config = config or DEFAULT_ACKERMANN_CHASSIS_CONFIG
        self._filtered_yaw_rate_command = 0.0

    def reset(self, pose: tuple[float, float, float]) -> None:
        super().reset(pose)
        self._filtered_yaw_rate_command = 0.0

    def step(
        self,
        command: BodyMotionCommand,
        dt: float,
        *,
        segment_mode: str | None = None,
        drift_mode: DriftMode = "off",
        gyro_reading: GyroReading | None = None,
    ) -> ChassisStepResult:
        del gyro_reading
        speed_ref = max(0.0, command.target_speed_mps)
        command_alpha = dt / max(dt + self.config.steering_command_time_constant_s, 1e-6)
        self._filtered_yaw_rate_command += (
            command.target_yaw_rate_rps - self._filtered_yaw_rate_command
        ) * command_alpha
        current_vx = self._state.longitudinal_speed
        accel = np.clip(
            (speed_ref - current_vx) / max(dt, 1e-6),
            -self.config.longitudinal_accel_limit_mps2,
            self.config.longitudinal_accel_limit_mps2,
        )
        next_vx = max(0.0, current_vx + float(accel) * dt)

        speed_for_steer = max(speed_ref, next_vx, self.config.min_speed_for_steering_mps)
        steering_target = float(
            np.clip(
                np.arctan(self.config.wheelbase_m * self._filtered_yaw_rate_command / speed_for_steer),
                -self.config.max_steering_angle_rad,
                self.config.max_steering_angle_rad,
            )
        )
        steering_step = float(
            np.clip(
                steering_target - self._state.steering_angle,
                -self.config.max_steering_rate_radps * dt,
                self.config.max_steering_rate_radps * dt,
            )
        )
        steering_angle = self._state.steering_angle + steering_step

        vx_effective = max(next_vx, self.config.min_speed_for_steering_mps)
        vy = self._state.lateral_speed
        yaw_rate = self._state.yaw_rate

        alpha_f = float(np.arctan2(vy + self.config.front_axle_to_cg_m * yaw_rate, vx_effective) - steering_angle)
        alpha_r = float(np.arctan2(vy - self.config.rear_axle_to_cg_m * yaw_rate, vx_effective))

        drift_active = drift_mode == "corner" and segment_mode == "line_follow" and next_vx > 0.05
        drift_scale = 0.0
        rear_slip_target = 0.0
        rear_cornering_stiffness = self.config.rear_cornering_stiffness_nprad
        if drift_active:
            drift_scale = 1.0
            rear_cornering_stiffness *= 1.0 - (1.0 - self.config.drift_rear_grip_scale) * drift_scale
            rear_slip_target = float(np.sign(self._filtered_yaw_rate_command)) * self.config.drift_target_slip_angle_rad
            rear_slip_target *= drift_scale

        lateral_front_force = self._saturated_lateral_force(
            slip_angle=alpha_f,
            cornering_stiffness=self.config.front_cornering_stiffness_nprad,
            friction_coeff=self.config.front_tire_friction_coeff,
            axle_load=self._front_axle_load(),
        )
        lateral_rear_force = self._saturated_lateral_force(
            slip_angle=alpha_r - rear_slip_target,
            cornering_stiffness=rear_cornering_stiffness,
            friction_coeff=self.config.rear_tire_friction_coeff,
            axle_load=self._rear_axle_load(),
        )

        vy_dot = (
            (lateral_front_force * np.cos(steering_angle) + lateral_rear_force) / self.config.mass_kg
            - next_vx * yaw_rate
        )
        yaw_accel = (
            self.config.front_axle_to_cg_m * lateral_front_force * np.cos(steering_angle)
            - self.config.rear_axle_to_cg_m * lateral_rear_force
        ) / self.config.yaw_inertia_kgm2

        next_vy = float(
            np.clip(vy + vy_dot * dt, -self.config.max_lateral_speed_mps, self.config.max_lateral_speed_mps)
        )
        next_yaw_rate = float(
            np.clip(yaw_rate + yaw_accel * dt, -self.config.max_yaw_rate_rps, self.config.max_yaw_rate_rps)
        )
        next_yaw = wrap_angle(self._state.yaw + next_yaw_rate * dt)

        cos_yaw = float(np.cos(next_yaw))
        sin_yaw = float(np.sin(next_yaw))
        world_vx = next_vx * cos_yaw - next_vy * sin_yaw
        world_vy = next_vx * sin_yaw + next_vy * cos_yaw
        next_x = self._state.x + world_vx * dt
        next_y = self._state.y + world_vy * dt
        slip_angle = float(np.arctan2(next_vy, max(abs(next_vx), self.config.min_speed_for_steering_mps)))

        self._state = VehicleState(
            x=next_x,
            y=next_y,
            yaw=next_yaw,
            longitudinal_speed=next_vx,
            lateral_speed=next_vy,
            yaw_rate=next_yaw_rate,
            slip_angle=slip_angle,
            steering_angle=steering_angle,
        )
        return ChassisStepResult(
            state=self._state,
            actuation=AckermannActuationCommand(
                speed_mps=next_vx,
                steering_angle_rad=steering_angle,
            ),
        )

    def _front_axle_load(self) -> float:
        return self.config.mass_kg * 9.81 * self.config.rear_axle_to_cg_m / self.config.wheelbase_m

    def _rear_axle_load(self) -> float:
        return self.config.mass_kg * 9.81 * self.config.front_axle_to_cg_m / self.config.wheelbase_m

    @staticmethod
    def _saturated_lateral_force(
        *,
        slip_angle: float,
        cornering_stiffness: float,
        friction_coeff: float,
        axle_load: float,
    ) -> float:
        linear_force = -cornering_stiffness * slip_angle
        max_force = friction_coeff * axle_load
        return float(np.clip(linear_force, -max_force, max_force))


DifferentialDriveOdometry = DifferentialDriveChassis


def create_chassis_model(chassis_type: ChassisType = "ackermann") -> BaseChassisModel:
    """根据底盘类型创建模型。"""

    if chassis_type == "differential":
        return DifferentialDriveChassis()
    if chassis_type == "ackermann":
        return AckermannChassisModel()
    raise ValueError(f"Unsupported chassis type: {chassis_type!r}")
