import unittest

import numpy as np

from artemis_mudri.domains.task import ControlSegment, build_route_plan
from artemis_mudri.domains.vehicle import (
    AckermannChassisModel,
    BodyMotionCommand,
    DifferentialDriveChassis,
    GyroReading,
    HybridLineFollowerController,
    LineSensorArrayReading,
    LqrLineFollower,
    LqrLineFollowerConfig,
    create_line_follower,
)


class HybridLineFollowerControllerTest(unittest.TestCase):
    def setUp(self) -> None:
        self.controller = HybridLineFollowerController()

    def test_waypoint_segment_turns_toward_target(self) -> None:
        route = build_route_plan("1")
        self.controller.reset(route.start_pose)
        current_position = np.array([0.8, 0.85], dtype=np.float64)
        empty_reading = LineSensorArrayReading(
            local_sensor_positions=np.zeros((5, 2), dtype=np.float64),
            world_sensor_positions=np.zeros((5, 2), dtype=np.float64),
            darkness=np.zeros(5, dtype=np.float64),
            digital_values=(0, 0, 0, 0, 0),
            line_detected=False,
            lateral_error_m=None,
        )
        step = self.controller.step(
            GyroReading(yaw=-0.3, yaw_rate=0.0),
            empty_reading,
            current_position,
            route,
        )
        self.assertGreater(step.command.target_yaw_rate_rps, 0.0)
        self.assertGreater(step.command.target_speed_mps, 0.0)

    def test_line_follow_segment_turns_toward_detected_line(self) -> None:
        route = build_route_plan("2")
        self.controller.reset(route.start_pose)
        self.controller.segment_index = 1
        line_reading = LineSensorArrayReading(
            local_sensor_positions=np.zeros((5, 2), dtype=np.float64),
            world_sensor_positions=np.zeros((5, 2), dtype=np.float64),
            darkness=np.array([0.9, 0.8, 0.2, 0.0, 0.0], dtype=np.float64),
            digital_values=(1, 1, 0, 0, 0),
            line_detected=True,
            lateral_error_m=-0.03,
        )
        step = self.controller.step(
            GyroReading(yaw=0.0, yaw_rate=0.0),
            line_reading,
            np.array([1.58, 0.95], dtype=np.float64),
            route,
        )
        self.assertLess(step.command.target_yaw_rate_rps, 0.0)
        self.assertGreater(step.command.target_speed_mps, 0.0)


class LqrLineFollowerTest(unittest.TestCase):
    def test_factory_builds_lqr_controller(self) -> None:
        follower = create_line_follower(LqrLineFollowerConfig())
        self.assertIsInstance(follower, LqrLineFollower)

    def test_lqr_turns_in_the_same_direction_as_line_error(self) -> None:
        follower = LqrLineFollower(LqrLineFollowerConfig())
        segment = ControlSegment(
            event_name="B",
            mode="line_follow",
            target_position=np.array([1.6, 1.0], dtype=np.float64),
            nominal_speed_mps=0.18,
            arrival_tolerance_m=0.05,
            search_turn_direction=1.0,
        )
        reading = LineSensorArrayReading(
            local_sensor_positions=np.zeros((5, 2), dtype=np.float64),
            world_sensor_positions=np.zeros((5, 2), dtype=np.float64),
            darkness=np.array([0.0, 0.0, 0.2, 0.8, 0.9], dtype=np.float64),
            digital_values=(0, 0, 0, 1, 1),
            line_detected=True,
            lateral_error_m=0.03,
        )
        output = follower.command(reading, segment)
        self.assertGreater(output.angular_speed, 0.0)
        self.assertLessEqual(output.linear_speed, segment.nominal_speed_mps)
        self.assertGreaterEqual(output.linear_speed, follower.config.min_speed_mps)


class ChassisModelTest(unittest.TestCase):
    def test_ackermann_positive_yaw_demand_maps_to_positive_steer(self) -> None:
        chassis = AckermannChassisModel()
        chassis.reset((0.0, 0.0, 0.0))
        result = chassis.step(
            BodyMotionCommand(target_speed_mps=0.4, target_yaw_rate_rps=0.8),
            0.01,
            segment_mode="line_follow",
            drift_mode="off",
        )
        self.assertGreater(result.actuation.steering_angle_rad, 0.0)

    def test_drift_adds_lateral_speed_on_corner_segments(self) -> None:
        chassis = AckermannChassisModel()
        chassis.reset((0.0, 0.0, 0.0))
        state = None
        for _ in range(60):
            state = chassis.step(
                BodyMotionCommand(target_speed_mps=0.6, target_yaw_rate_rps=1.2),
                0.01,
                segment_mode="line_follow",
                drift_mode="corner",
            ).state
        assert state is not None
        self.assertGreater(abs(state.lateral_speed), 0.02)
        self.assertGreater(abs(state.slip_angle), 0.02)

    def test_drift_mode_stays_off_for_waypoint_segments(self) -> None:
        chassis = AckermannChassisModel()
        chassis.reset((0.0, 0.0, 0.0))
        state = None
        for _ in range(60):
            state = chassis.step(
                BodyMotionCommand(target_speed_mps=0.6, target_yaw_rate_rps=1.2),
                0.01,
                segment_mode="waypoint",
                drift_mode="corner",
            ).state
        assert state is not None
        self.assertLess(abs(state.slip_angle), 0.12)

    def test_differential_chassis_preserves_turn_direction(self) -> None:
        chassis = DifferentialDriveChassis()
        chassis.reset((0.0, 0.0, 0.0))
        result = chassis.step(
            BodyMotionCommand(target_speed_mps=0.3, target_yaw_rate_rps=-0.7),
            0.01,
        )
        self.assertLess(result.state.yaw_rate, 0.0)
        self.assertGreater(result.actuation.left_speed, result.actuation.right_speed)


if __name__ == "__main__":
    unittest.main()
