import json
import math
import unittest
from pathlib import Path

from artemis_mudri.domains.task import TaskCatalog, available_tasks, build_route_plan
from artemis_mudri.domains.track import ANCHORS


class RoutePlanTest(unittest.TestCase):
    def test_available_tasks_reflect_task_directory(self) -> None:
        self.assertEqual(available_tasks(), ("1", "2", "3"))

    def test_anchor_coordinates_match_problem_geometry(self) -> None:
        self.assertTupleEqual(tuple(ANCHORS["A"]), (0.6, 1.0))
        self.assertTupleEqual(tuple(ANCHORS["B"]), (1.6, 1.0))
        self.assertTupleEqual(tuple(ANCHORS["C"]), (1.6, 0.2))
        self.assertTupleEqual(tuple(ANCHORS["D"]), (0.6, 0.2))

    def test_task2_path_length_matches_expected_segments(self) -> None:
        route = build_route_plan("2")
        expected = 1.0 + math.pi * 0.4 + 1.0 + math.pi * 0.4
        self.assertAlmostEqual(route.path.total_length, expected, delta=0.03)

    def test_task3_event_order_matches_statement(self) -> None:
        route = build_route_plan("3")
        event_names = [event.name for event in route.path.events]
        self.assertEqual(event_names, ["C", "B", "D", "A"])

    def test_task2_control_modes_match_hardware_story(self) -> None:
        route = build_route_plan("2")
        control_modes = [segment.mode for segment in route.control_segments]
        self.assertEqual(control_modes, ["waypoint", "line_follow", "waypoint", "line_follow"])

    def test_task_json_has_required_schema_fields(self) -> None:
        tasks_dir = Path(__file__).resolve().parents[2] / "assets" / "tasks"
        catalog = TaskCatalog(tasks_dir=tasks_dir)
        for task_id in catalog.available_tasks():
            payload = json.loads((tasks_dir / f"task{task_id}.json").read_text(encoding="utf-8"))
            self.assertTrue(
                {"task_id", "label", "description", "time_limit_s", "path_segments", "control_segments"}
                <= payload.keys()
            )
            self.assertTrue(payload["path_segments"])
            self.assertTrue(payload["control_segments"])


if __name__ == "__main__":
    unittest.main()
