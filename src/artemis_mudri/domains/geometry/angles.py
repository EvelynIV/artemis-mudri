from __future__ import annotations
"""角度处理工具。"""

import numpy as np


def wrap_angle(angle: float) -> float:
    """将角度归一化到 [-pi, pi]。"""
    return float(np.arctan2(np.sin(angle), np.cos(angle)))
