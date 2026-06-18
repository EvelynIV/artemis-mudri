from __future__ import annotations
"""参考路径及其事件索引定义。"""

from dataclasses import dataclass

import numpy as np
from numpy.typing import NDArray

FloatArray = NDArray[np.float64]


@dataclass(frozen=True)
class PathEvent:
    """路径上的关键事件点。"""
    name: str
    index: int
    position: FloatArray


@dataclass(frozen=True)
class ReferencePath:
    """离散参考路径及其派生几何量。"""
    name: str
    points: FloatArray
    headings: FloatArray
    arc_length: FloatArray
    events: tuple[PathEvent, ...]

    @property
    def total_length(self) -> float:
        return float(self.arc_length[-1])

    @property
    def start_pose(self) -> tuple[float, float, float]:
        return (
            float(self.points[0, 0]),
            float(self.points[0, 1]),
            float(self.headings[0]),
        )

    def remaining_distance(self, index: int) -> float:
        """计算从指定索引到终点的剩余距离。"""
        return max(0.0, self.total_length - float(self.arc_length[index]))

    def progress_index(
        self,
        position: FloatArray,
        start_index: int = 0,
        window: int = 120,
    ) -> int:
        """在局部窗口内寻找最接近当前位置的路径索引。"""
        upper = min(len(self.points), max(start_index + 1, start_index + window))
        search_points = self.points[start_index:upper]
        distances = np.linalg.norm(search_points - position[None, :], axis=1)
        return start_index + int(np.argmin(distances))

    def target_index(self, progress_index: int, lookahead: float) -> int:
        """根据前视距离找到目标路径索引。"""
        target_s = min(self.total_length, float(self.arc_length[progress_index]) + lookahead)
        target_index = int(np.searchsorted(self.arc_length, target_s, side="left"))
        return min(target_index, len(self.points) - 1)


def append_segment(existing: list[FloatArray], segment: FloatArray) -> FloatArray:
    """将新片段拼接到已有路径，避免重复首点。"""
    if existing:
        segment = segment[1:]
    existing.append(segment)
    return segment


def build_reference_path(
    name: str,
    segments: list[tuple[str, FloatArray]],
) -> ReferencePath:
    """将多个路径片段拼接成一条完整参考路径。"""
    stitched: list[FloatArray] = []
    events: list[PathEvent] = []
    current_index = -1
    for event_name, segment in segments:
        used_segment = append_segment(stitched, segment)
        current_index += len(used_segment)
        events.append(
            PathEvent(
                name=event_name,
                index=current_index,
                position=used_segment[-1].astype(np.float64, copy=True),
            )
        )

    points = np.vstack(stitched).astype(np.float64)
    deltas = np.diff(points, axis=0)
    segment_lengths = np.linalg.norm(deltas, axis=1)
    arc_length = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    headings = np.arctan2(deltas[:, 1], deltas[:, 0])
    headings = np.concatenate((headings, headings[-1:]))
    return ReferencePath(
        name=name,
        points=points,
        headings=headings.astype(np.float64),
        arc_length=arc_length.astype(np.float64),
        events=tuple(events),
    )
