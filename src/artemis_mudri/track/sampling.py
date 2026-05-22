from __future__ import annotations
"""线段与圆弧采样工具。"""

from math import ceil

import numpy as np

from artemis_mudri.track.path import FloatArray


def sample_line(start: FloatArray, end: FloatArray, resolution: float) -> FloatArray:
    """按给定分辨率对线段均匀采样。"""
    delta = end - start
    distance = float(np.linalg.norm(delta))
    samples = max(2, ceil(distance / resolution) + 1)
    return np.linspace(start, end, samples)


def sample_arc(
    center: FloatArray,
    radius: float,
    start_angle_deg: float,
    end_angle_deg: float,
    resolution: float,
) -> FloatArray:
    """按给定分辨率对圆弧均匀采样。"""
    span_rad = np.deg2rad(end_angle_deg - start_angle_deg)
    arc_length = abs(span_rad) * radius
    samples = max(2, ceil(arc_length / resolution) + 1)
    angles = np.deg2rad(np.linspace(start_angle_deg, end_angle_deg, samples))
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)
    return np.column_stack((x, y))
