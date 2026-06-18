from __future__ import annotations
"""JSON wire codec for simulation requests and responses."""

from typing import Any

from artemis_mudri.runtime.models import (
    EpisodeResult,
    FinishedResult,
    ObservationResult,
    StartedResult,
    StartEpisode,
    WheelCommand,
)
from artemis_mudri.simulation import SimulationEvent, SimulationObservation, SimulationSummary

JsonObject = dict[str, Any]


def parse_start_request(message: JsonObject) -> StartEpisode:
    """Parse a start JSON request into a runtime request."""

    return StartEpisode(
        max_time_s=_optional_float(message, "max_time_s"),
        control_period_s=_optional_float(message, "control_period_s"),
        initial_pose=_initial_pose(message.get("initial_pose")),
        initial_progress_index=int(message.get("initial_progress_index", 0)),
        random_seed=_optional_int(message, "random_seed"),
    )


def parse_step_request(message: JsonObject) -> WheelCommand:
    """Parse a step JSON request into a runtime command."""

    return WheelCommand(
        sequence_id=_optional_int(message, "sequence_id"),
        rear_left_target_speed=float(message["rear_left_target_speed"]),
        rear_right_target_speed=float(message["rear_right_target_speed"]),
    )


def result_to_dict(result: EpisodeResult) -> JsonObject:
    """Serialize a runtime result to the public JSON wire shape."""

    if isinstance(result, StartedResult):
        return {
            "type": "started",
            "started": {
                "time_limit_s": result.time_limit_s,
                "control_period_s": result.control_period_s,
            },
            "observation": observation_to_dict(result.observation),
        }
    if isinstance(result, ObservationResult):
        return {"type": "observation", "observation": observation_to_dict(result.observation)}
    if isinstance(result, FinishedResult):
        return {
            "type": "finished",
            "finished": {
                "reason": result.reason,
                "summary": summary_to_dict(result.summary),
                "final_step_trace": final_step_trace(result.summary, result.reason),
            },
        }
    raise TypeError(f"Unsupported result type: {type(result).__name__}")


def error_response(message: str) -> JsonObject:
    """Build a public error response."""

    return {"type": "error", "error": message}


def observation_to_dict(observation: SimulationObservation) -> JsonObject:
    """Serialize a simulation observation to JSON-friendly dictionaries."""

    state = observation.state
    line_sensor = observation.line_sensor
    encoder = observation.encoder
    return {
        "sequence_id": int(observation.sequence_id),
        "sim_time_s": float(observation.sim_time_s),
        "line_sensor_darkness": [float(value) for value in line_sensor.darkness],
        "line_sensor": {
            "darkness": [float(value) for value in line_sensor.darkness],
            "digital": [bool(value) for value in line_sensor.digital_values],
            "line_detected": bool(line_sensor.line_detected),
            "error": float(line_sensor.error),
            "lateral_error_m": (
                None if line_sensor.lateral_error_m is None else float(line_sensor.lateral_error_m)
            ),
            "error_weights": [int(value) for value in line_sensor.error_weights],
            "local_sensor_positions": line_sensor.local_sensor_positions.astype(float).tolist(),
            "world_sensor_positions": line_sensor.world_sensor_positions.astype(float).tolist(),
        },
        "imu": {
            "yaw_deg": float(observation.imu.yaw),
            "yaw_rate_deg_s": float(observation.imu.yaw_rate),
        },
        "encoder": {
            "rear_left_delta_ticks": float(encoder.rear_left_pulses),
            "rear_right_delta_ticks": float(encoder.rear_right_pulses),
            "rear_left_total_ticks": float(encoder.rear_left_total_pulses),
            "rear_right_total_ticks": float(encoder.rear_right_total_pulses),
            "rear_left_measure_speed": float(encoder.rear_left_measure_speed),
            "rear_right_measure_speed": float(encoder.rear_right_measure_speed),
            "forward_distance_cm": float(encoder.forward_distance_cm),
        },
        "path_progress": {
            "active_segment_index": int(observation.active_segment_index),
            "reached_goal": bool(observation.reached_goal),
            "progress_index": int(observation.progress_index),
            "progress_m": float(observation.progress_m),
            "remaining_distance_m": float(observation.remaining_distance_m),
        },
        "kinematics": {
            "longitudinal_velocity_m_s": float(state.longitudinal_speed),
            "lateral_velocity_m_s": float(state.lateral_speed),
            "yaw_rate_rad_s": float(state.yaw_rate),
        },
        "oracle": {
            "cross_track_error_m": float(observation.cross_track_error_m),
            "heading_error_rad": float(observation.heading_error_rad),
            "progress_index": int(observation.progress_index),
            "progress_m": float(observation.progress_m),
            "remaining_distance_m": float(observation.remaining_distance_m),
        },
        "step_trace": {
            "step_id": int(observation.sequence_id),
            "events": [event_to_dict(event) for event in observation.step_events],
            "terminated": False,
            "truncated": False,
            "reason": "",
        },
        "pose": {
            "x_m": float(state.x),
            "y_m": float(state.y),
            "yaw_rad": float(state.yaw),
        },
    }


def summary_to_dict(summary: SimulationSummary) -> JsonObject:
    """Serialize a simulation summary to JSON-friendly dictionaries."""

    final_x, final_y, final_yaw = summary.final_pose
    return {
        "reached_goal": bool(summary.reached_goal),
        "elapsed_time_s": float(summary.elapsed_time_s),
        "route_length_m": float(summary.route_length_m),
        "max_cross_track_error_m": float(summary.max_cross_track_error_m),
        "rms_cross_track_error_m": float(summary.rms_cross_track_error_m),
        "final_pose": {
            "x_m": float(final_x),
            "y_m": float(final_y),
            "yaw_rad": float(final_yaw),
        },
        "events": [event_to_dict(event) for event in summary.events],
        "total_steps": int(summary.total_steps),
    }


def event_to_dict(event: SimulationEvent) -> JsonObject:
    """Serialize a simulation event to JSON-friendly dictionaries."""

    x_m, y_m, yaw_rad = event.pose
    return {
        "event_id": int(event.event_id),
        "step_id": int(event.step_id),
        "namespace": event.namespace,
        "type": event.type,
        "severity": event.severity,
        "pose": {
            "x_m": float(x_m),
            "y_m": float(y_m),
            "yaw_rad": float(yaw_rad),
        },
        "metrics": {key: float(value) for key, value in event.metrics.items()},
        "labels": dict(event.labels),
    }


def final_step_trace(summary: SimulationSummary, reason: str) -> JsonObject:
    """Build the final trace block attached to a finished response."""

    terminated = reason == "goal_reached"
    return {
        "step_id": int(summary.total_steps),
        "events": [event_to_dict(event) for event in summary.events],
        "terminated": terminated,
        "truncated": not terminated,
        "reason": reason,
    }


def _optional_float(message: JsonObject, key: str) -> float | None:
    value = message.get(key)
    if value is None:
        return None
    return float(value)


def _optional_int(message: JsonObject, key: str) -> int | None:
    value = message.get(key)
    if value is None:
        return None
    return int(value)


def _initial_pose(value: Any) -> tuple[float, float, float] | None:
    if value is None:
        return None
    if not isinstance(value, dict):
        raise ValueError("initial_pose must be an object with x_m, y_m and yaw_rad.")
    return (float(value["x_m"]), float(value["y_m"]), float(value["yaw_rad"]))
