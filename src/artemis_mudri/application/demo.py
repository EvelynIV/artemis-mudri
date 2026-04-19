from __future__ import annotations
"""应用层：组装任务、仿真与结果输出。"""

import json
from pathlib import Path
from typing import Literal

from artemis_mudri.domains.simulation import SimulationSummary
from artemis_mudri.domains.task import RoutePlan, build_route_plan
from artemis_mudri.domains.vehicle import (
    ChassisType,
    DriftMode,
    HybridLineFollowerConfig,
    HybridLineFollowerController,
    LqrLineFollowerConfig,
    PdLineFollowerConfig,
    PidLineFollowerConfig,
)
from artemis_mudri.infrastructure.mujoco.backend import MuJoCoTeachingDemo

LineFollowControllerName = Literal["pid", "pd", "lqr"]


def _build_controller(
    controller_name: LineFollowControllerName,
    speed_scale: float = 1.0,
) -> HybridLineFollowerController:
    """根据名称构造循线控制器。"""
    if controller_name == "pid":
        line_follow_config = PidLineFollowerConfig()
    elif controller_name == "pd":
        line_follow_config = PdLineFollowerConfig()
    else:
        line_follow_config = LqrLineFollowerConfig()
    return HybridLineFollowerController(
        HybridLineFollowerConfig(
            line_follow=line_follow_config,
            speed_scale=speed_scale,
        )
    )


def create_demo(
    task_id: str,
    route_resolution: float = 0.03,
    line_follow_controller: LineFollowControllerName = "pid",
    chassis: ChassisType = "ackermann",
    drift_mode: DriftMode = "off",
    speed_scale: float = 1.0,
) -> tuple[RoutePlan, MuJoCoTeachingDemo]:
    """根据任务编号创建路线与 MuJoCo 演示对象。"""
    route = build_route_plan(task_id)
    demo = MuJoCoTeachingDemo(
        route=route,
        controller=_build_controller(line_follow_controller, speed_scale=speed_scale),
        route_resolution=route_resolution,
        chassis=chassis,
        drift_mode=drift_mode,
    )
    return route, demo


def run_demo(
    task_id: str,
    max_time_s: float | None = None,
    render: bool = True,
    line_follow_controller: LineFollowControllerName = "pid",
    chassis: ChassisType = "ackermann",
    drift_mode: DriftMode = "off",
    speed_scale: float = 1.0,
) -> tuple[RoutePlan, SimulationSummary]:
    """运行一次完整演示，并返回路线与汇总结果。"""
    route, demo = create_demo(
        task_id,
        line_follow_controller=line_follow_controller,
        chassis=chassis,
        drift_mode=drift_mode,
        speed_scale=speed_scale,
    )
    return route, demo.run(max_time_s=max_time_s, render=render)


def write_summary_json(summary: SimulationSummary, output_path: str | Path) -> None:
    """将仿真汇总结果写入 JSON 文件。"""
    path = Path(output_path)
    path.write_text(
        json.dumps(summary.to_dict(), indent=2),
        encoding="utf-8",
    )
