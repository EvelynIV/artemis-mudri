from __future__ import annotations
"""任务目录加载与路线构建。"""

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

from artemis_mudri.domains.geometry import FloatArray, sample_arc, sample_line
from artemis_mudri.domains.task.route import ControlSegment, RoutePlan
from artemis_mudri.domains.track import ARC_RADIUS_M, anchor, build_reference_path, center_point


@dataclass(frozen=True)
class TaskCatalog:
    """从任务 JSON 目录读取比赛任务。"""
    tasks_dir: Path | None = None

    def available_tasks(self) -> tuple[str, ...]:
        """返回当前可用任务编号。"""
        task_ids = sorted(path.name.removeprefix("task").removesuffix(".json") for path in self._task_paths())
        return tuple(task_ids)

    def build_route_plan(self, task_id: str, resolution: float = 0.02) -> RoutePlan:
        """读取任务配置并生成路线计划。"""
        normalized = task_id.lower().replace("task", "")
        payload = self._load_task_payload(normalized)
        self._validate_task_payload(payload, normalized)
        segments = [
            (
                str(segment["event_name"]),
                self._build_path_segment(segment, resolution),
            )
            for segment in payload["path_segments"]
        ]
        return RoutePlan(
            task_id=str(payload["task_id"]),
            label=str(payload["label"]),
            description=str(payload["description"]),
            path=build_reference_path(f"task{normalized}", segments),
            control_segments=tuple(self._build_control_segment(segment) for segment in payload["control_segments"]),
            time_limit_s=float(payload["time_limit_s"]),
        )

    def _task_paths(self) -> list[Path]:
        """列出任务目录中的 JSON 文件。"""
        base_dir = self._tasks_dir()
        return sorted(
            (entry for entry in base_dir.iterdir() if entry.name.startswith("task") and entry.name.endswith(".json")),
            key=lambda entry: entry.name,
        )

    def _tasks_dir(self) -> Path:
        """解析任务目录路径。"""
        if self.tasks_dir is not None:
            return self.tasks_dir
        return Path(__file__).resolve().parents[4] / "assets" / "tasks"

    def _load_task_payload(self, normalized_task_id: str) -> dict[str, Any]:
        """读取单个任务 JSON。"""
        task_path = self._tasks_dir() / f"task{normalized_task_id}.json"
        if not task_path.exists():
            raise ValueError(f"Unknown task id: {normalized_task_id!r}")
        return json.loads(task_path.read_text(encoding="utf-8"))

    def _validate_task_payload(self, payload: dict[str, Any], task_id: str) -> None:
        """校验任务 JSON 的必要字段。"""
        required_top_level = {
            "task_id",
            "label",
            "description",
            "time_limit_s",
            "path_segments",
            "control_segments",
        }
        missing = required_top_level - payload.keys()
        if missing:
            fields = ", ".join(sorted(missing))
            raise ValueError(f"Task {task_id} is missing required fields: {fields}")
        if not payload["path_segments"]:
            raise ValueError(f"Task {task_id} must declare at least one path segment")
        if not payload["control_segments"]:
            raise ValueError(f"Task {task_id} must declare at least one control segment")

    def _build_path_segment(self, payload: dict[str, Any], resolution: float) -> FloatArray:
        """将任务段描述转换为离散路径点。"""
        segment_type = payload["type"]
        if segment_type == "line":
            return sample_line(
                anchor(str(payload["start_anchor"])),
                anchor(str(payload["end_anchor"])),
                resolution,
            )
        if segment_type == "arc":
            return sample_arc(
                center=center_point(str(payload["center"])),
                radius=float(payload.get("radius_m", ARC_RADIUS_M)),
                start_angle_deg=float(payload["start_angle_deg"]),
                end_angle_deg=float(payload["end_angle_deg"]),
                resolution=resolution,
            )
        raise ValueError(f"Unsupported path segment type: {segment_type!r}")

    def _build_control_segment(self, payload: dict[str, Any]) -> ControlSegment:
        """将控制段 JSON 转成领域对象。"""
        return ControlSegment(
            event_name=str(payload["event_name"]),
            mode=str(payload["mode"]),
            target_position=anchor(str(payload["target_anchor"])),
            nominal_speed_mps=float(payload["nominal_speed_mps"]),
            arrival_tolerance_m=float(payload["arrival_tolerance_m"]),
            search_turn_direction=float(payload.get("search_turn_direction", 0.0)),
        )


DEFAULT_TASK_CATALOG = TaskCatalog()


def available_tasks() -> tuple[str, ...]:
    """返回默认任务目录中的任务编号。"""
    return DEFAULT_TASK_CATALOG.available_tasks()


def build_route_plan(task_id: str, resolution: float = 0.02) -> RoutePlan:
    """使用默认任务目录构建路线计划。"""
    return DEFAULT_TASK_CATALOG.build_route_plan(task_id, resolution=resolution)
