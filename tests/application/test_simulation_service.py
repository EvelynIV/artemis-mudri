import unittest

from artemis_mudri.application.simulation_service import VehicleSimulationService
from artemis_mudri.simulation.v1 import vehicle_simulation_pb2 as pb2


class VehicleSimulationServiceTest(unittest.TestCase):
    def test_proto_message_round_trip_preserves_control_command(self) -> None:
        message = pb2.ClientMessage(
            control_command=pb2.VehicleControlCommand(
                sequence_id=42,
                velocity=7.0,
                turn=-0.25,
                rear_left_target_speed=6.75,
                rear_right_target_speed=7.25,
                rear_left_mode=pb2.MOTOR_MODE_DEG,
                rear_right_mode=pb2.MOTOR_MODE_DEG,
            )
        )
        parsed = pb2.ClientMessage.FromString(message.SerializeToString())
        self.assertEqual(parsed.WhichOneof("payload"), "control_command")
        self.assertEqual(parsed.control_command.sequence_id, 42)
        self.assertAlmostEqual(parsed.control_command.velocity, 7.0)
        self.assertEqual(parsed.control_command.rear_left_mode, pb2.MOTOR_MODE_DEG)

    def test_stream_episode_starts_observes_and_finishes_on_stop(self) -> None:
        service = VehicleSimulationService()
        requests = iter(
            [
                pb2.ClientMessage(start=pb2.StartEpisodeRequest(task_id="1", control_period_s=0.01)),
                pb2.ClientMessage(stop=pb2.StopEpisodeRequest(reason="test_stop")),
            ]
        )
        responses = list(service.StreamEpisode(requests, None))
        self.assertEqual([response.WhichOneof("payload") for response in responses], ["started", "observation", "finished"])
        self.assertEqual(responses[-1].finished.reason, "test_stop")

    def test_stream_episode_motor_command_advances_time(self) -> None:
        service = VehicleSimulationService()
        requests = iter(
            [
                pb2.ClientMessage(
                    start=pb2.StartEpisodeRequest(task_id="1", max_time_s=0.01, control_period_s=0.01)
                ),
                pb2.ClientMessage(
                    control_command=pb2.VehicleControlCommand(
                        sequence_id=0,
                        velocity=7.0,
                        rear_left_target_speed=7.0,
                        rear_right_target_speed=7.0,
                        rear_left_mode=pb2.MOTOR_MODE_DEG,
                        rear_right_mode=pb2.MOTOR_MODE_DEG,
                    )
                ),
            ]
        )
        responses = list(service.StreamEpisode(requests, None))
        self.assertEqual(responses[-1].WhichOneof("payload"), "finished")
        self.assertGreaterEqual(responses[-1].finished.summary.elapsed_time_s, 0.01)


if __name__ == "__main__":
    unittest.main()
