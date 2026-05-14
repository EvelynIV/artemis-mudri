from __future__ import annotations
"""MuJoCo 后端：负责模拟 artemis-m0 风格的后双驱小车硬件。"""

import time
from contextlib import AbstractContextManager
from dataclasses import dataclass, field
from typing import Any

import mujoco
import numpy as np

from artemis_mudri.domains.simulation import SimulationEvent, SimulationSummary
from artemis_mudri.domains.task import RoutePlan
from artemis_mudri.domains.vehicle import (
    DifferentialMotorCommand,
    EncoderConfig,
    EncoderReading,
    GyroReading,
    IncrementalMotorPid,
    IncrementalMotorPidConfig,
    LineSensorArray,
    LineSensorArrayReading,
    MotorDebugReading,
    MotorMode,
    VehicleState,
)
from artemis_mudri.domains.geometry import wrap_angle
from artemis_mudri.infrastructure.mujoco.xml import build_demo_model_xml


@dataclass(frozen=True)
class MotorDriverConfig:
    """电机驱动、编码器和简化动力学配置。"""

    encoder: EncoderConfig = field(default_factory=EncoderConfig)
    pid: IncrementalMotorPidConfig = field(default_factory=IncrementalMotorPidConfig)
    wheel_track_m: float = 0.14
    pid_output_to_pulse_per_tick: float = 0.03
    target_speed_feedforward: float = 0.75
    speed_response_alpha: float = 0.65
    max_pulse_speed_per_tick: float = 35.0


@dataclass(frozen=True)
class SimulationObservation:
    """服务端发给小车客户端的一帧观测。"""

    sequence_id: int
    sim_time_s: float
    line_sensor: LineSensorArrayReading
    imu: GyroReading
    encoder: EncoderReading
    motor_debug: MotorDebugReading
    active_segment_index: int
    completed_events: tuple[SimulationEvent, ...]
    reached_goal: bool


class DifferentialMuJoCoSimulation:
    """由外部速度目标命令驱动的后双驱小车仿真 episode。"""

    def __init__(
        self,
        route: RoutePlan,
        route_resolution: float = 0.03,
        motor_driver: MotorDriverConfig | None = None,
        random_seed: int | None = None,
        initial_yaw_noise_deg: float = 0.0,
    ) -> None:
        self.route = route
        self.motor_driver = motor_driver or MotorDriverConfig()
        self.sensor_array = LineSensorArray()
        self.model = mujoco.MjModel.from_xml_string(build_demo_model_xml(route, resolution=route_resolution))
        self.data = mujoco.MjData(self.model)
        self._joint_state_handles = self._resolve_joint_state_handles(("car_x", "car_y", "car_yaw"))
        self._led_site_ids = self._resolve_led_site_ids()
        self._led_off_rgba = np.array([0.16, 0.16, 0.16, 1.0], dtype=np.float32)
        self._led_on_rgba = np.array([0.0, 1.0, 0.2, 1.0], dtype=np.float32)
        self._led_on_delay_s = 0.03
        self._led_off_delay_s = 0.12
        self._led_brightness = np.zeros(len(self._led_site_ids), dtype=np.float32)
        self._viewer_context: AbstractContextManager[Any] | None = None
        self._viewer: Any | None = None
        self._rear_left_pid = IncrementalMotorPid(self.motor_driver.pid)
        self._rear_right_pid = IncrementalMotorPid(self.motor_driver.pid)
        self._last_command = DifferentialMotorCommand(
            velocity=0.0,
            turn=0.0,
            rear_left_target_speed=0.0,
            rear_right_target_speed=0.0,
            rear_left_mode=MotorMode.STOP,
            rear_right_mode=MotorMode.STOP,
        )
        self.reset(random_seed=random_seed, initial_yaw_noise_deg=initial_yaw_noise_deg)

    @property
    def timestep_s(self) -> float:
        """返回控制层默认 tick。"""

        return self.motor_driver.encoder.control_tick_s

    @property
    def reached_goal(self) -> bool:
        """判断是否已经完成所有路径事件。"""

        return len(self.logged_events) == len(self.route.path.events)

    @property
    def active_segment_index(self) -> int:
        """返回服务端根据路径进度估计的当前任务段。"""

        return min(len(self.logged_events), len(self.route.control_segments))

    def reset(self, *, random_seed: int | None = None, initial_yaw_noise_deg: float = 0.0) -> None:
        """重置仿真状态和统计量。"""

        mujoco.mj_resetData(self.model, self.data)
        rng = np.random.default_rng(0 if random_seed is None else random_seed)
        yaw_noise = float(rng.uniform(-initial_yaw_noise_deg, initial_yaw_noise_deg))
        start_x, start_y, _ = self.route.start_pose
        self.sequence_id = 0
        self.progress_index = 0
        self.cross_track_errors: list[float] = []
        self.logged_events: list[SimulationEvent] = []
        self._rear_left_speed = 0.0
        self._rear_right_speed = 0.0
        self._rear_left_total_pulses = 0.0
        self._rear_right_total_pulses = 0.0
        self._last_rear_left_pulses = 0.0
        self._last_rear_right_pulses = 0.0
        self._last_yaw_rate = 0.0
        self._rear_left_pid.reset()
        self._rear_right_pid.reset()
        self._led_brightness[:] = 0.0
        self._sync_mujoco_state(
            VehicleState(
                x=float(start_x),
                y=float(start_y),
                yaw=float(np.deg2rad(yaw_noise)),
            )
        )
        self._update_metrics(self.current_state().position)
        self._record_completed_events()
        self._update_sensor_leds((0,) * len(self._led_site_ids), dt=0.0)

    def open_viewer(self) -> None:
        """启动 MuJoCo 交互式查看器。"""

        if self._viewer is not None:
            return
        import mujoco.viewer

        self._viewer_context = mujoco.viewer.launch_passive(
            self.model,
            self.data,
            show_left_ui=False,
            show_right_ui=False,
        )
        self._viewer = self._viewer_context.__enter__()
        self._configure_viewer(self._viewer)

    def close_viewer(self) -> None:
        """关闭已启动的 MuJoCo viewer。"""

        if self._viewer_context is None:
            return
        self._viewer_context.__exit__(None, None, None)
        self._viewer_context = None
        self._viewer = None

    def viewer_is_running(self) -> bool:
        """返回 viewer 是否仍处于运行状态。"""

        return self._viewer is None or bool(self._viewer.is_running())

    def sync_viewer(self) -> None:
        """把当前状态同步到 viewer。"""

        if self._viewer is None:
            return
        self._viewer.sync()
        time.sleep(min(self.timestep_s, 0.02))

    def current_state(self) -> VehicleState:
        """返回底盘当前真值状态。"""

        x_addr, x_dof = self._joint_state_handles["car_x"]
        y_addr, y_dof = self._joint_state_handles["car_y"]
        yaw_addr, yaw_dof = self._joint_state_handles["car_yaw"]
        yaw = float(self.data.qpos[yaw_addr])
        cos_yaw = float(np.cos(yaw))
        sin_yaw = float(np.sin(yaw))
        world_vx = float(self.data.qvel[x_dof])
        world_vy = float(self.data.qvel[y_dof])
        longitudinal_speed = world_vx * cos_yaw + world_vy * sin_yaw
        lateral_speed = -world_vx * sin_yaw + world_vy * cos_yaw
        return VehicleState(
            x=float(self.data.qpos[x_addr]),
            y=float(self.data.qpos[y_addr]),
            yaw=yaw,
            longitudinal_speed=longitudinal_speed,
            lateral_speed=lateral_speed,
            yaw_rate=float(self.data.qvel[yaw_dof]),
        )

    def current_gyro_reading(self) -> GyroReading:
        """从底盘真值生成 MS901M 风格航向观测。"""

        state = self.current_state()
        return GyroReading(
            yaw=float(np.rad2deg(state.yaw) % 360.0),
            yaw_rate=float(np.rad2deg(state.yaw_rate)),
        )

    def observe(self) -> SimulationObservation:
        """采样当前传感器并返回一帧观测。"""

        state = self.current_state()
        line_sensor = self.sensor_array.sense_pose(state.x, state.y, state.yaw)
        self._update_sensor_leds(line_sensor.digital_values, dt=self.timestep_s)
        mujoco.mj_forward(self.model, self.data)
        self.sync_viewer()
        return SimulationObservation(
            sequence_id=self.sequence_id,
            sim_time_s=float(self.data.time),
            line_sensor=line_sensor,
            imu=self.current_gyro_reading(),
            encoder=self._encoder_reading(),
            motor_debug=self._motor_debug_reading(),
            active_segment_index=self.active_segment_index,
            completed_events=tuple(self.logged_events),
            reached_goal=self.reached_goal,
        )

    def step_control_command(
        self,
        command: DifferentialMotorCommand,
        dt: float | None = None,
    ) -> SimulationObservation:
        """按一帧固件速度目标命令推进仿真。"""

        step_dt = dt or self.timestep_s
        self._last_command = command
        left_snapshot = self._rear_left_pid.compute(
            target_speed=command.rear_left_target_speed,
            measure_speed=self._rear_left_speed,
            mode=command.rear_left_mode,
        )
        right_snapshot = self._rear_right_pid.compute(
            target_speed=command.rear_right_target_speed,
            measure_speed=self._rear_right_speed,
            mode=command.rear_right_mode,
        )
        self._rear_left_speed = self._next_motor_speed(
            current=self._rear_left_speed,
            target=command.rear_left_target_speed,
            pid_output=left_snapshot.output,
            mode=command.rear_left_mode,
        )
        self._rear_right_speed = self._next_motor_speed(
            current=self._rear_right_speed,
            target=command.rear_right_target_speed,
            pid_output=right_snapshot.output,
            mode=command.rear_right_mode,
        )

        scale = step_dt / self.motor_driver.encoder.control_tick_s
        self._last_rear_left_pulses = self._rear_left_speed * scale
        self._last_rear_right_pulses = self._rear_right_speed * scale
        self._rear_left_total_pulses += self._last_rear_left_pulses
        self._rear_right_total_pulses += self._last_rear_right_pulses

        left_mps = self._pulse_speed_to_mps(self._rear_left_speed)
        right_mps = self._pulse_speed_to_mps(self._rear_right_speed)
        linear_speed = 0.5 * (left_mps + right_mps)
        yaw_rate = (right_mps - left_mps) / self.motor_driver.wheel_track_m
        self._integrate_planar(linear_speed=linear_speed, yaw_rate=yaw_rate, dt=step_dt)
        self.sequence_id += 1
        self._update_metrics(self.current_state().position)
        self._record_completed_events()
        return self.observe()

    # 兼容旧测试名称，主路径使用 step_control_command。
    def step_motor_command(self, left_pwm: float, right_pwm: float, dt: float | None = None) -> SimulationObservation:
        """兼容旧接口：把归一化 PWM 粗略转换成速度目标。"""

        max_speed = self.motor_driver.max_pulse_speed_per_tick
        return self.step_control_command(
            DifferentialMotorCommand(
                velocity=0.0,
                turn=0.0,
                rear_left_target_speed=float(np.clip(left_pwm, -1.0, 1.0) * max_speed),
                rear_right_target_speed=float(np.clip(right_pwm, -1.0, 1.0) * max_speed),
                rear_left_mode=MotorMode.SPEED,
                rear_right_mode=MotorMode.SPEED,
            ),
            dt=dt,
        )

    def summary(self) -> SimulationSummary:
        """生成当前 episode 的汇总统计。"""

        state = self.current_state()
        errors = np.asarray(self.cross_track_errors, dtype=np.float64)
        return SimulationSummary(
            task_id=self.route.task_id,
            reached_goal=self.reached_goal,
            elapsed_time_s=float(self.data.time),
            route_length_m=self.route.path.total_length,
            max_cross_track_error_m=float(errors.max(initial=0.0)),
            rms_cross_track_error_m=float(np.sqrt(np.mean(errors * errors))) if len(errors) else 0.0,
            final_pose=(state.x, state.y, state.yaw),
            events=tuple(self.logged_events),
        )

    def _next_motor_speed(self, *, current: float, target: float, pid_output: float, mode: MotorMode) -> float:
        """将 PID 输出转换为下一 tick 的编码器测量速度。"""

        if mode == MotorMode.STOP:
            desired = 0.0
        else:
            desired_from_pid = pid_output * self.motor_driver.pid_output_to_pulse_per_tick
            desired = (
                self.motor_driver.target_speed_feedforward * target
                + (1.0 - self.motor_driver.target_speed_feedforward) * desired_from_pid
            )
        next_speed = current + (desired - current) * self.motor_driver.speed_response_alpha
        return float(np.clip(next_speed, -self.motor_driver.max_pulse_speed_per_tick, self.motor_driver.max_pulse_speed_per_tick))

    def _pulse_speed_to_mps(self, pulse_speed: float) -> float:
        """将脉冲/20ms 速度换算为 m/s。"""

        distance_cm_per_tick = pulse_speed / self.motor_driver.encoder.cm_to_pulse
        return float(distance_cm_per_tick / 100.0 / self.motor_driver.encoder.control_tick_s)

    def _integrate_planar(self, *, linear_speed: float, yaw_rate: float, dt: float) -> None:
        """用平面约束动力学推进 MuJoCo 状态。"""

        state = self.current_state()
        yaw_mid = state.yaw + 0.5 * yaw_rate * dt
        new_state = VehicleState(
            x=state.x + linear_speed * float(np.cos(yaw_mid)) * dt,
            y=state.y + linear_speed * float(np.sin(yaw_mid)) * dt,
            yaw=wrap_angle(state.yaw + yaw_rate * dt),
            longitudinal_speed=linear_speed,
            lateral_speed=0.0,
            yaw_rate=yaw_rate,
        )
        self._last_yaw_rate = yaw_rate
        self.data.time += dt
        self._sync_mujoco_state(new_state)

    def _encoder_reading(self) -> EncoderReading:
        """返回当前编码器读数。"""

        pulse_avg = 0.5 * (self._rear_left_total_pulses + self._rear_right_total_pulses)
        return EncoderReading(
            rear_left_pulses=self._last_rear_left_pulses,
            rear_right_pulses=self._last_rear_right_pulses,
            rear_left_total_pulses=self._rear_left_total_pulses,
            rear_right_total_pulses=self._rear_right_total_pulses,
            rear_left_measure_speed=self._rear_left_speed,
            rear_right_measure_speed=self._rear_right_speed,
            forward_distance_cm=pulse_avg / self.motor_driver.encoder.cm_to_pulse,
        )

    def _motor_debug_reading(self) -> MotorDebugReading:
        """返回电机速度环调试读数。"""

        return MotorDebugReading(
            rear_left=self._rear_left_pid.snapshot(),
            rear_right=self._rear_right_pid.snapshot(),
            rear_left_pwm=self._rear_left_pid.output,
            rear_right_pwm=self._rear_right_pid.output,
        )

    def _resolve_joint_state_handles(self, joint_names: tuple[str, ...]) -> dict[str, tuple[int, int]]:
        """解析关节 qpos/dof 地址。"""

        handles: dict[str, tuple[int, int]] = {}
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo joint not found: {joint_name}")
            handles[joint_name] = (
                int(self.model.jnt_qposadr[joint_id]),
                int(self.model.jnt_dofadr[joint_id]),
            )
        return handles

    def _resolve_led_site_ids(self) -> tuple[int, ...]:
        """解析 LED site 在 MuJoCo 模型中的索引。"""

        return tuple(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"led_{index}")
            for index in range(len(self.sensor_array.local_sensor_positions))
        )

    def _set_joint_state(self, joint_name: str, *, qpos: float, qvel: float) -> None:
        """写入单个关节的 qpos/qvel。"""

        qpos_addr, dof_addr = self._joint_state_handles[joint_name]
        self.data.qpos[qpos_addr] = qpos
        self.data.qvel[dof_addr] = qvel

    def _sync_mujoco_state(self, state: VehicleState) -> None:
        """将平面状态同步到 MuJoCo 数据。"""

        cos_yaw = float(np.cos(state.yaw))
        sin_yaw = float(np.sin(state.yaw))
        world_vx = state.longitudinal_speed * cos_yaw - state.lateral_speed * sin_yaw
        world_vy = state.longitudinal_speed * sin_yaw + state.lateral_speed * cos_yaw

        self._set_joint_state("car_x", qpos=state.x, qvel=world_vx)
        self._set_joint_state("car_y", qpos=state.y, qvel=world_vy)
        self._set_joint_state("car_yaw", qpos=state.yaw, qvel=state.yaw_rate)
        mujoco.mj_forward(self.model, self.data)

    def _update_metrics(self, position: np.ndarray) -> None:
        """更新路径进度和横向误差统计。"""

        self.progress_index = self.route.path.progress_index(
            position=position,
            start_index=self.progress_index,
            window=160,
        )
        nearest_point = self.route.path.points[self.progress_index]
        self.cross_track_errors.append(float(np.linalg.norm(nearest_point - position)))

    def _record_completed_events(self) -> None:
        """按路径进度记录已完成的任务事件。"""

        completed_names = {event.name for event in self.logged_events}
        for event in self.route.path.events:
            if event.name in completed_names:
                continue
            if self.progress_index < event.index:
                break
            self.logged_events.append(
                SimulationEvent(
                    name=event.name,
                    timestamp_s=float(self.data.time),
                    path_index=event.index,
                )
            )

    def _update_sensor_leds(self, digital_values: tuple[int, ...], dt: float) -> None:
        """根据数字量传感器结果更新带延迟的 LED 亮度。"""

        for index, (site_id, value) in enumerate(zip(self._led_site_ids, digital_values)):
            if site_id < 0:
                continue
            target = 1.0 if value else 0.0
            delay_s = self._led_on_delay_s if value else self._led_off_delay_s
            alpha = 1.0 if delay_s <= 0.0 else min(1.0, dt / delay_s)
            self._led_brightness[index] = float(
                self._led_brightness[index] + (target - self._led_brightness[index]) * alpha
            )
            brightness = self._led_brightness[index]
            self.model.site_rgba[site_id] = self._led_off_rgba + (
                self._led_on_rgba - self._led_off_rgba
            ) * brightness

    def _configure_viewer(self, viewer: Any) -> None:
        """设置 MuJoCo viewer 的俯视相机。"""

        viewer.cam.lookat[:] = [1.1, 0.6, 0.0]
        viewer.cam.distance = 2.8
        viewer.cam.azimuth = 0.0
        viewer.cam.elevation = -90.0
