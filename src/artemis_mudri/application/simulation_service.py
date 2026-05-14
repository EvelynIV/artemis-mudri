from __future__ import annotations
"""gRPC 仿真服务应用层。"""

import logging
from concurrent import futures
from collections.abc import Iterator

import grpc

from artemis_mudri.domains.simulation import SimulationSummary
from artemis_mudri.domains.task import build_route_plan
from artemis_mudri.infrastructure.mujoco.backend import (
    DifferentialMuJoCoSimulation,
    DifferentialMotorCommand,
    MotorDriverConfig,
    SimulationObservation,
)
from artemis_mudri.domains.vehicle import MotorMode
from artemis_mudri.simulation.v1 import vehicle_simulation_pb2 as pb2
from artemis_mudri.simulation.v1 import vehicle_simulation_pb2_grpc as pb2_grpc

logger = logging.getLogger(__name__)


class VehicleSimulationService(pb2_grpc.VehicleSimulationServiceServicer):
    """差速小车仿真流式服务。"""

    def __init__(
        self,
        default_task_id: str = "1",
        default_render: bool = False,
        motor_driver: MotorDriverConfig | None = None,
    ) -> None:
        self.default_task_id = default_task_id
        self.default_render = default_render
        self.motor_driver = motor_driver or MotorDriverConfig()

    def StreamEpisode(
        self,
        request_iterator: Iterator[pb2.ClientMessage],
        context: grpc.ServicerContext,
    ) -> Iterator[pb2.ServerMessage]:
        """按 start/observation/command 的节奏运行一次 episode。"""

        del context
        try:
            first_message = next(request_iterator)
        except StopIteration:
            yield _error_message("StreamEpisode requires an initial StartEpisodeRequest.")
            return

        if first_message.WhichOneof("payload") != "start":
            yield _error_message("First client message must be StartEpisodeRequest.")
            return

        start = first_message.start
        task_id = start.task_id or self.default_task_id
        route = build_route_plan(task_id)
        episode = DifferentialMuJoCoSimulation(
            route=route,
            motor_driver=self.motor_driver,
            random_seed=start.random_seed if start.HasField("random_seed") else None,
            initial_yaw_noise_deg=start.initial_yaw_noise_deg if start.initial_yaw_noise_deg else 5.0,
        )
        control_period_s = start.control_period_s or episode.timestep_s
        time_budget_s = start.max_time_s or route.time_limit_s
        render = self.default_render or start.render
        reason = "client_disconnected"

        try:
            if render:
                episode.open_viewer()
            logger.info(
                "Started simulation episode task=%s render=%s control_period=%s max_time=%s",
                task_id,
                render,
                control_period_s,
                time_budget_s,
            )
            yield pb2.ServerMessage(
                started=pb2.EpisodeStarted(
                    task_id=route.task_id,
                    label=route.label,
                    description=route.description,
                    time_limit_s=route.time_limit_s,
                    control_period_s=control_period_s,
                )
            )

            yield _observation_message(episode.observe(), route_task_id=route.task_id)

            while episode.data.time < time_budget_s and episode.viewer_is_running():
                try:
                    client_message = next(request_iterator)
                except StopIteration:
                    break

                payload = client_message.WhichOneof("payload")
                if payload == "stop":
                    reason = client_message.stop.reason or "client_stopped"
                    break
                if payload != "control_command":
                    yield _error_message(f"Unsupported client message during episode: {payload!r}.")
                    return

                control_command = client_message.control_command
                observation = episode.step_control_command(
                    DifferentialMotorCommand(
                        velocity=control_command.velocity,
                        turn=control_command.turn,
                        rear_left_target_speed=control_command.rear_left_target_speed,
                        rear_right_target_speed=control_command.rear_right_target_speed,
                        rear_left_mode=_mode_from_proto(control_command.rear_left_mode),
                        rear_right_mode=_mode_from_proto(control_command.rear_right_mode),
                    ),
                    dt=control_period_s,
                )

                if episode.data.time >= time_budget_s:
                    reason = "time_limit"
                    break
                if not episode.viewer_is_running():
                    reason = "viewer_closed"
                    break

                yield _observation_message(observation, route_task_id=route.task_id)

            yield _finished_message(episode.summary(), reason)
        except Exception as exc:  # pragma: no cover - converted to protocol error for clients.
            logger.exception("Simulation episode failed")
            yield _error_message(str(exc))
        finally:
            episode.close_viewer()


def create_grpc_server(
    default_task_id: str = "1",
    default_render: bool = False,
    max_workers: int = 4,
) -> grpc.Server:
    """创建已注册 VehicleSimulationService 的 gRPC server。"""

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    pb2_grpc.add_VehicleSimulationServiceServicer_to_server(
        VehicleSimulationService(
            default_task_id=default_task_id,
            default_render=default_render,
        ),
        server,
    )
    return server


def serve(
    host: str = "127.0.0.1",
    port: int = 50051,
    default_task_id: str = "1",
    render: bool = False,
    max_workers: int = 4,
) -> None:
    """启动阻塞式 gRPC 仿真服务。"""

    address = f"{host}:{port}"
    server = create_grpc_server(
        default_task_id=default_task_id,
        default_render=render,
        max_workers=max_workers,
    )
    bound_port = server.add_insecure_port(address)
    if bound_port == 0:
        raise RuntimeError(f"Failed to bind gRPC server to {address}")
    server.start()
    logger.info("Vehicle simulation service listening on %s", address)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        logger.info("Stopping vehicle simulation service")
        server.stop(grace=1.0)


def _observation_message(observation: SimulationObservation, route_task_id: str) -> pb2.ServerMessage:
    line_sensor = observation.line_sensor
    line_sensor_frame = pb2.LineSensorFrame(
        digital_values=list(line_sensor.digital_values),
        darkness=[float(value) for value in line_sensor.darkness],
        line_detected=line_sensor.line_detected,
        weights=list(line_sensor.error_weights),
        error=line_sensor.error,
    )
    if line_sensor.lateral_error_m is not None:
        line_sensor_frame.lateral_error_m = line_sensor.lateral_error_m

    return pb2.ServerMessage(
        observation=pb2.ObservationFrame(
            sequence_id=observation.sequence_id,
            sim_time_s=observation.sim_time_s,
            line_sensor=line_sensor_frame,
            imu=pb2.ImuFrame(
                yaw_deg=observation.imu.yaw,
                yaw_rate_deg_s=observation.imu.yaw_rate,
            ),
            task_progress=pb2.TaskProgressFrame(
                task_id=route_task_id,
                active_segment_index=observation.active_segment_index,
                completed_event_count=len(observation.completed_events),
                completed_events=[event.name for event in observation.completed_events],
                reached_goal=observation.reached_goal,
            ),
            encoder=pb2.EncoderFrame(
                rear_left_pulses=observation.encoder.rear_left_pulses,
                rear_right_pulses=observation.encoder.rear_right_pulses,
                rear_left_total_pulses=observation.encoder.rear_left_total_pulses,
                rear_right_total_pulses=observation.encoder.rear_right_total_pulses,
                rear_left_measure_speed=observation.encoder.rear_left_measure_speed,
                rear_right_measure_speed=observation.encoder.rear_right_measure_speed,
                forward_distance_cm=observation.encoder.forward_distance_cm,
            ),
            motor_debug=pb2.MotorDebugFrame(
                rear_left_target_speed=observation.motor_debug.rear_left.target_speed,
                rear_right_target_speed=observation.motor_debug.rear_right.target_speed,
                rear_left_pid_output=observation.motor_debug.rear_left.output,
                rear_right_pid_output=observation.motor_debug.rear_right.output,
                rear_left_pwm=observation.motor_debug.rear_left_pwm,
                rear_right_pwm=observation.motor_debug.rear_right_pwm,
                rear_left_mode=_mode_to_proto(observation.motor_debug.rear_left.mode),
                rear_right_mode=_mode_to_proto(observation.motor_debug.rear_right.mode),
            ),
        )
    )


def _finished_message(summary: SimulationSummary, reason: str) -> pb2.ServerMessage:
    final_x, final_y, final_yaw = summary.final_pose
    return pb2.ServerMessage(
        finished=pb2.EpisodeFinished(
            summary=pb2.SimulationSummary(
                task_id=summary.task_id,
                reached_goal=summary.reached_goal,
                elapsed_time_s=summary.elapsed_time_s,
                route_length_m=summary.route_length_m,
                max_cross_track_error_m=summary.max_cross_track_error_m,
                rms_cross_track_error_m=summary.rms_cross_track_error_m,
                final_pose=pb2.Pose2D(
                    x_m=final_x,
                    y_m=final_y,
                    yaw_rad=final_yaw,
                ),
                events=[
                    pb2.SimulationEvent(
                        name=event.name,
                        timestamp_s=event.timestamp_s,
                        path_index=event.path_index,
                    )
                    for event in summary.events
                ],
            ),
            reason=reason,
        )
    )


def _error_message(message: str) -> pb2.ServerMessage:
    return pb2.ServerMessage(error=pb2.SimulationError(message=message))


def _mode_from_proto(value: int) -> MotorMode:
    try:
        return MotorMode(int(value))
    except ValueError:
        return MotorMode.STOP


def _mode_to_proto(mode: MotorMode) -> int:
    return int(mode)
