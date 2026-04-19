from __future__ import annotations
"""车辆高层控制器与段切换状态机。"""

from dataclasses import dataclass, field

import numpy as np

from artemis_mudri.domains.geometry import wrap_angle
from artemis_mudri.domains.task import ControlSegment, RoutePlan
from artemis_mudri.domains.vehicle.line_follow import (
    BaseLineFollower,
    LineFollowerConfig,
    PidLineFollowerConfig,
    create_line_follower,
)
from artemis_mudri.domains.vehicle.sensing import LineSensorArrayReading
from artemis_mudri.domains.vehicle.state import BodyMotionCommand, GyroReading


@dataclass(frozen=True)
class ControllerStep:
    """控制器单步输出与状态摘要。"""
    command: BodyMotionCommand
    active_segment_index: int
    completed_event: str | None
    goal_reached: bool
    line_detected: bool
    line_error_m: float | None
    distance_to_target_m: float | None


@dataclass(frozen=True)
class HybridLineFollowerConfig:
    """混合循迹控制器的参数集合。"""
    speed_scale: float = 1.0
    waypoint_heading_gain: float = 2.8
    waypoint_turn_limit_rps: float = 2.5
    line_follow: LineFollowerConfig = field(default_factory=PidLineFollowerConfig)
    open_loop_line_detect_frames: int = 2
    line_loss_exit_frames: int = 5


class HybridLineFollowerController:
    """结合开环定航向与数字量循线的状态机控制器。"""
    def __init__(self, config: HybridLineFollowerConfig | None = None) -> None:
        self.config = config or HybridLineFollowerConfig()
        self.line_follower: BaseLineFollower = create_line_follower(self.config.line_follow)
        self.segment_index = 0
        self.line_detect_streak = 0
        self.line_loss_streak = 0
        self.line_detected_in_segment = False
        self.current_heading_target: float | None = None
        self.current_heading_segment_index: int | None = None

    def reset(self, pose: tuple[float, float, float]) -> None:
        """重置控制器内部阶段状态。"""
        self.segment_index = 0
        self.line_follower.reset()
        self.line_detect_streak = 0
        self.line_loss_streak = 0
        self.line_detected_in_segment = False
        self.current_heading_target = None
        self.current_heading_segment_index = None

    def step(
        self,
        gyro_reading: GyroReading,
        line_reading: LineSensorArrayReading,
        current_position: np.ndarray,
        route: RoutePlan,
    ) -> ControllerStep:
        """根据陀螺仪和数字量传感器推进一个控制周期。"""
        completed_event = self._consume_segment_transitions(line_reading, current_position, route)
        if self.segment_index >= len(route.control_segments):
            return self._final_step(completed_event, line_reading)

        segment = route.control_segments[self.segment_index]
        if segment.mode == "waypoint":
            command = self._waypoint_command(gyro_reading, route)
        else:
            command = self._line_follow_command(line_reading, segment)

        return ControllerStep(
            command=command,
            active_segment_index=self.segment_index,
            completed_event=completed_event,
            goal_reached=False,
            line_detected=line_reading.line_detected,
            line_error_m=line_reading.lateral_error_m,
            distance_to_target_m=None,
        )

    def _consume_segment_transitions(
        self,
        line_reading: LineSensorArrayReading,
        current_position: np.ndarray,
        route: RoutePlan,
    ) -> str | None:
        """按当前段规则消费切段事件。"""
        completed_event = None
        while self.segment_index < len(route.control_segments):
            segment = route.control_segments[self.segment_index]
            if segment.mode == "waypoint":
                if line_reading.line_detected:
                    self.line_detect_streak += 1
                else:
                    self.line_detect_streak = 0
                if self.line_detect_streak < self.config.open_loop_line_detect_frames:
                    break
            else:
                if line_reading.line_detected:
                    self.line_detected_in_segment = True
                    self.line_loss_streak = 0
                else:
                    if not self.line_detected_in_segment:
                        break
                    self.line_loss_streak += 1
                if self.line_loss_streak < self.config.line_loss_exit_frames:
                    break
                distance_to_target = float(np.linalg.norm(segment.target_position - current_position))
                completion_tolerance = max(segment.arrival_tolerance_m, 0.12)
                if distance_to_target > completion_tolerance:
                    break

            completed_event = segment.event_name
            self.segment_index += 1
            self._reset_segment_state()
        return completed_event

    def _reset_segment_state(self) -> None:
        """切段后重置段内状态。"""
        self.line_follower.reset()
        self.line_detect_streak = 0
        self.line_loss_streak = 0
        self.line_detected_in_segment = False
        self.current_heading_target = None
        self.current_heading_segment_index = None

    def _final_step(
        self,
        completed_event: str | None,
        line_reading: LineSensorArrayReading,
    ) -> ControllerStep:
        """生成任务结束后的停车输出。"""
        return ControllerStep(
            command=BodyMotionCommand(0.0, 0.0),
            active_segment_index=self.segment_index,
            completed_event=completed_event,
            goal_reached=True,
            line_detected=line_reading.line_detected,
            line_error_m=line_reading.lateral_error_m,
            distance_to_target_m=None,
        )

    def _waypoint_command(
        self,
        gyro_reading: GyroReading,
        route: RoutePlan,
    ) -> BodyMotionCommand:
        """开环段：定航向前进，直到重新见线。"""
        target_heading = self._target_heading(route)
        heading_error = wrap_angle(target_heading - gyro_reading.yaw)
        angular_speed = float(
            np.clip(
                self.config.waypoint_heading_gain * heading_error,
                -self.config.waypoint_turn_limit_rps,
                self.config.waypoint_turn_limit_rps,
            )
        )
        segment = route.control_segments[self.segment_index]
        return BodyMotionCommand(
            target_speed_mps=float(self.config.speed_scale * segment.nominal_speed_mps),
            target_yaw_rate_rps=float(self.config.speed_scale * angular_speed),
        )

    def _target_heading(self, route: RoutePlan) -> float:
        """计算当前开环段的目标航向。"""
        if self.current_heading_segment_index == self.segment_index and self.current_heading_target is not None:
            return self.current_heading_target
        target_position = route.control_segments[self.segment_index].target_position
        if self.segment_index == 0:
            start_x, start_y, _ = route.start_pose
            start_position = np.array([start_x, start_y], dtype=np.float64)
        else:
            start_position = route.control_segments[self.segment_index - 1].target_position
        delta = target_position - start_position
        self.current_heading_target = float(np.arctan2(delta[1], delta[0]))
        self.current_heading_segment_index = self.segment_index
        return self.current_heading_target

    def _line_follow_command(
        self,
        line_reading: LineSensorArrayReading,
        segment: ControlSegment,
    ) -> BodyMotionCommand:
        """循线段：调用独立的循线算法子包。"""
        output = self.line_follower.command(line_reading, segment)
        return BodyMotionCommand(
            target_speed_mps=float(self.config.speed_scale * output.linear_speed),
            target_yaw_rate_rps=float(self.config.speed_scale * output.angular_speed),
        )
