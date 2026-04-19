from __future__ import annotations
"""MuJoCo 后端：负责运行教学演示仿真。"""

import time
from typing import Any

import mujoco
import numpy as np

from artemis_mudri.domains.simulation import SimulationEvent, SimulationSummary
from artemis_mudri.domains.task import RoutePlan
from artemis_mudri.domains.vehicle import (
    ChassisStepResult,
    ChassisType,
    ControllerStep,
    DriftMode,
    GyroReading,
    HybridLineFollowerController,
    LineSensorArray,
    VehicleState,
    create_chassis_model,
)
from artemis_mudri.infrastructure.mujoco.xml import build_demo_model_xml


class MuJoCoTeachingDemo:
    """将路线、控制器和 MuJoCo 模型组装成可运行演示。"""

    def __init__(
        self,
        route: RoutePlan,
        controller: HybridLineFollowerController | None = None,
        route_resolution: float = 0.03,
        chassis: ChassisType = "ackermann",
        drift_mode: DriftMode = "off",
    ) -> None:
        self.route = route
        self.controller = controller or HybridLineFollowerController()
        self.chassis_type = chassis
        self.drift_mode = drift_mode
        self.sensor_array = LineSensorArray()
        self.chassis_model = create_chassis_model(chassis)
        self.model = mujoco.MjModel.from_xml_string(build_demo_model_xml(route, resolution=route_resolution))
        self.data = mujoco.MjData(self.model)
        self._joint_state_handles = self._resolve_joint_state_handles(
            ("car_x", "car_y", "car_yaw", "front_left_steer", "front_right_steer")
        )
        self._led_site_ids = self._resolve_led_site_ids()
        self._led_off_rgba = np.array([0.16, 0.16, 0.16, 1.0], dtype=np.float32)
        self._led_on_rgba = np.array([0.0, 1.0, 0.2, 1.0], dtype=np.float32)
        self._led_on_delay_s = 0.03
        self._led_off_delay_s = 0.12
        self._final_hold_s = 2.0
        self._led_brightness = np.zeros(len(self._led_site_ids), dtype=np.float32)
        self._last_actuation: ChassisStepResult | None = None
        self.reset()

    def _resolve_joint_state_handles(self, joint_names: tuple[str, ...]) -> dict[str, tuple[int, int]]:
        """解析关节 qpos/dof 地址。"""

        handles: dict[str, tuple[int, int]] = {}
        for joint_name in joint_names:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            if joint_id < 0:
                continue
            handles[joint_name] = (
                int(self.model.jnt_qposadr[joint_id]),
                int(self.model.jnt_dofadr[joint_id]),
            )
        return handles

    def _resolve_led_site_ids(self) -> tuple[int, ...]:
        """解析 5 个 LED site 在 MuJoCo 模型中的索引。"""

        return tuple(
            mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, f"led_{index}")
            for index in range(len(self.sensor_array.local_sensor_positions))
        )

    def reset(self) -> None:
        """重置仿真状态、控制器和统计量。"""

        mujoco.mj_resetData(self.model, self.data)
        self.controller.reset(self.route.start_pose)
        self.chassis_model.reset(self.route.start_pose)
        self.progress_index = 0
        self.cross_track_errors: list[float] = []
        self.logged_events: list[SimulationEvent] = []
        self.path_event_index = {event.name: event.index for event in self.route.path.events}
        self._led_brightness[:] = 0.0
        self._sync_mujoco_state(self.chassis_model.state, steering_rate=0.0)
        self.data.ctrl[:] = 0.0
        self._update_sensor_leds((0, 0, 0, 0, 0), dt=0.0)
        self._last_actuation = None

    def current_state(self) -> VehicleState:
        """返回底盘模型当前真值状态。"""

        return self.chassis_model.state

    def current_gyro_reading(self) -> GyroReading:
        """从底盘真值生成陀螺仪观测。"""

        state = self.current_state()
        return GyroReading(yaw=state.yaw, yaw_rate=state.yaw_rate)

    def _set_joint_state(self, joint_name: str, *, qpos: float, qvel: float) -> None:
        """写入单个关节的 qpos/qvel。"""

        handle = self._joint_state_handles.get(joint_name)
        if handle is None:
            return
        qpos_addr, dof_addr = handle
        self.data.qpos[qpos_addr] = qpos
        self.data.qvel[dof_addr] = qvel

    def _sync_mujoco_state(self, state: VehicleState, steering_rate: float) -> None:
        """将领域层状态同步到 MuJoCo 数据。"""

        cos_yaw = float(np.cos(state.yaw))
        sin_yaw = float(np.sin(state.yaw))
        world_vx = state.longitudinal_speed * cos_yaw - state.lateral_speed * sin_yaw
        world_vy = state.longitudinal_speed * sin_yaw + state.lateral_speed * cos_yaw

        self._set_joint_state("car_x", qpos=state.x, qvel=world_vx)
        self._set_joint_state("car_y", qpos=state.y, qvel=world_vy)
        self._set_joint_state("car_yaw", qpos=state.yaw, qvel=state.yaw_rate)
        self._set_joint_state("front_left_steer", qpos=state.steering_angle, qvel=steering_rate)
        self._set_joint_state("front_right_steer", qpos=state.steering_angle, qvel=steering_rate)
        mujoco.mj_forward(self.model, self.data)

    def _apply_step_command(self, step_result: ControllerStep) -> None:
        """把控制器输出写入 MuJoCo 控制通道。"""

        yaw = self.current_state().yaw
        speed = step_result.command.target_speed_mps
        self.data.ctrl[0] = speed * np.cos(yaw)
        self.data.ctrl[1] = speed * np.sin(yaw)
        self.data.ctrl[2] = step_result.command.target_yaw_rate_rps

    def _advance_chassis_state(self, step_result: ControllerStep) -> ChassisStepResult:
        """用领域层底盘模型推进一帧仿真。"""

        dt = float(self.model.opt.timestep)
        previous_steering = self.chassis_model.state.steering_angle
        segment_mode = None
        if step_result.active_segment_index < len(self.route.control_segments):
            segment_mode = self.route.control_segments[step_result.active_segment_index].mode
        chassis_result = self.chassis_model.step(
            step_result.command,
            dt,
            segment_mode=segment_mode,
            drift_mode=self.drift_mode,
        )
        steering_rate = (chassis_result.state.steering_angle - previous_steering) / dt if dt > 0.0 else 0.0
        self.data.time += dt
        self._sync_mujoco_state(chassis_result.state, steering_rate=steering_rate)
        self._last_actuation = chassis_result
        return chassis_result

    def _record_completed_event(self, step_result: ControllerStep) -> None:
        """记录本步完成的任务事件。"""

        if step_result.completed_event is None:
            return
        self.logged_events.append(
            SimulationEvent(
                name=step_result.completed_event,
                timestamp_s=float(self.data.time),
                path_index=self.path_event_index[step_result.completed_event],
            )
        )

    def _update_metrics(self, position: np.ndarray) -> None:
        """更新路径进度和横向误差统计。"""

        self.progress_index = self.route.path.progress_index(
            position=position,
            start_index=self.progress_index,
            window=160,
        )
        nearest_point = self.route.path.points[self.progress_index]
        self.cross_track_errors.append(float(np.linalg.norm(nearest_point - position)))

    def _update_sensor_leds(self, digital_values: tuple[int, ...], dt: float) -> None:
        """根据 5 路数字量传感器结果更新带延迟的 LED 亮度。"""

        for index, (site_id, value) in enumerate(zip(self._led_site_ids, digital_values)):
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

    def run(self, max_time_s: float | None = None, render: bool = True) -> SimulationSummary:
        """运行仿真，可选打开 MuJoCo viewer。"""

        time_budget = max_time_s or self.route.time_limit_s
        if render:
            import mujoco.viewer

            with mujoco.viewer.launch_passive(
                self.model,
                self.data,
                show_left_ui=False,
                show_right_ui=False,
            ) as viewer:
                self._configure_viewer(viewer)
                return self._run_loop(time_budget, viewer=viewer)
        return self._run_loop(time_budget, viewer=None)

    def _run_loop(self, time_budget: float, viewer: Any) -> SimulationSummary:
        """主循环：采样、控制、推进、统计。"""

        settled_steps = 0
        while self.data.time < time_budget and (viewer is None or viewer.is_running()):
            true_state = self.current_state()
            gyro_reading = self.current_gyro_reading()
            sensor_reading = self.sensor_array.sense_pose(true_state.x, true_state.y, true_state.yaw)
            self._update_sensor_leds(sensor_reading.digital_values, dt=float(self.model.opt.timestep))
            step_result = self.controller.step(
                gyro_reading,
                sensor_reading,
                true_state.position,
                self.route,
            )
            self._apply_step_command(step_result)
            chassis_result = self._advance_chassis_state(step_result)
            self._update_metrics(chassis_result.state.position)
            self._record_completed_event(step_result)

            if viewer is not None:
                viewer.sync()
                time.sleep(self.model.opt.timestep)

            if step_result.goal_reached:
                settled_steps += 1
                if settled_steps >= 10:
                    break
            else:
                settled_steps = 0

        state = self.current_state()
        errors = np.asarray(self.cross_track_errors, dtype=np.float64)
        reached_goal = len(self.logged_events) == len(self.route.control_segments)
        if viewer is not None and reached_goal:
            hold_start = time.monotonic()
            while viewer.is_running() and (time.monotonic() - hold_start) < self._final_hold_s:
                viewer.sync()
                time.sleep(self.model.opt.timestep)

        return SimulationSummary(
            task_id=self.route.task_id,
            reached_goal=reached_goal,
            elapsed_time_s=float(self.data.time),
            route_length_m=self.route.path.total_length,
            max_cross_track_error_m=float(errors.max(initial=0.0)),
            rms_cross_track_error_m=float(np.sqrt(np.mean(errors * errors))) if len(errors) else 0.0,
            final_pose=(state.x, state.y, state.yaw),
            events=tuple(self.logged_events),
        )
