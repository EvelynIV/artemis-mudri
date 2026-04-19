from __future__ import annotations
"""命令行接口：负责参数解析与结果展示。"""

from enum import Enum
from typing import Annotated, Literal

import typer

from artemis_mudri.application.demo import run_demo, write_summary_json
from artemis_mudri.domains.task import available_tasks

app = typer.Typer(
    add_completion=False,
    help="Run the MuJoCo teaching demo for the NUEDC automatic car task.",
)
LineFollowControllerName = Literal["pid", "pd", "lqr"]
ChassisName = Literal["differential", "ackermann"]
DriftModeName = Literal["off", "corner"]


class LineFollowControllerOption(str, Enum):
    """命令行可选的循线控制器类型。"""

    PID = "pid"
    PD = "pd"
    LQR = "lqr"


class ChassisOption(str, Enum):
    """命令行可选的底盘类型。"""

    DIFFERENTIAL = "differential"
    ACKERMANN = "ackermann"


class DriftModeOption(str, Enum):
    """命令行可选的漂移模式。"""

    OFF = "off"
    CORNER = "corner"


def _format_summary(summary, route) -> str:
    """将仿真结果整理成终端可读文本。"""
    lines = [
        f"{route.label}: {route.description}",
        f"Route length: {summary.route_length_m:.3f} m",
        f"Elapsed time: {summary.elapsed_time_s:.3f} s",
        f"Reached goal: {summary.reached_goal}",
        f"Max cross-track error: {summary.max_cross_track_error_m:.4f} m",
        f"RMS cross-track error: {summary.rms_cross_track_error_m:.4f} m",
        "Events:",
    ]
    if summary.events:
        lines.extend(
            f"  - {event.name} at {event.timestamp_s:.3f} s (path index {event.path_index})"
            for event in summary.events
        )
    else:
        lines.append("  - none")
    return "\n".join(lines)


def _validate_task(task: str) -> str:
    """校验任务编号是否合法。"""
    valid_tasks = available_tasks()
    if task not in valid_tasks:
        choices = ", ".join(valid_tasks)
        raise typer.BadParameter(f"Task must be one of: {choices}.")
    return task


@app.command()
def run_demo_command(
    task: Annotated[
        str,
        typer.Option(
            help="Competition requirement to replay.",
            metavar="TASK_ID",
            show_default=True,
            envvar="ARTEMIS_TASK",
            show_envvar=True,
        ),
    ] = "1",
    max_time: Annotated[
        float | None,
        typer.Option(
            help="Override the task time budget in seconds.",
            envvar="ARTEMIS_MAX_TIME_S",
            show_envvar=True,
        ),
    ] = None,
    render: Annotated[
        bool,
        typer.Option(
            help="Launch the MuJoCo interactive viewer.",
            show_default=True,
            envvar="ARTEMIS_RENDER",
            show_envvar=True,
        ),
    ] = True,
    summary_json: Annotated[
        str | None,
        typer.Option(
            help="Optional path for writing the run summary as JSON.",
            envvar="ARTEMIS_SUMMARY_JSON",
            show_envvar=True,
        ),
    ] = None,
    line_follow_controller: Annotated[
        LineFollowControllerOption,
        typer.Option(
            "--line-follow-controller",
            help="Line following controller implementation to use.",
            envvar="ARTEMIS_LINE_FOLLOW_CONTROLLER",
            show_envvar=True,
        ),
    ] = LineFollowControllerOption.PID,
    chassis: Annotated[
        ChassisOption,
        typer.Option(
            "--chassis",
            help="Vehicle chassis model to use.",
            envvar="ARTEMIS_CHASSIS",
            show_envvar=True,
        ),
    ] = ChassisOption.ACKERMANN,
    drift_mode: Annotated[
        DriftModeOption,
        typer.Option(
            "--drift-mode",
            help="Drift mode for the Ackermann chassis.",
            envvar="ARTEMIS_DRIFT_MODE",
            show_envvar=True,
        ),
    ] = DriftModeOption.OFF,
    speed_scale: Annotated[
        float,
        typer.Option(
            "--speed-scale",
            help="Global multiplier applied to segment target speeds.",
            envvar="ARTEMIS_SPEED_SCALE",
            show_envvar=True,
        ),
    ] = 1.0,
) -> None:
    """CLI 命令：运行指定任务的教学演示。"""
    validated_task = _validate_task(task)
    route, summary = run_demo(
        validated_task,
        max_time_s=max_time,
        render=render,
        line_follow_controller=line_follow_controller.value,
        chassis=chassis.value,
        drift_mode=drift_mode.value,
        speed_scale=speed_scale,
    )
    typer.echo(_format_summary(summary, route))
    if summary_json:
        write_summary_json(summary, summary_json)
        typer.echo(f"Summary written to: {summary_json}")


def main() -> None:
    """CLI 主入口。"""
    app()


if __name__ == "__main__":
    main()
