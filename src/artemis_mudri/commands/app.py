"""标准命令行入口：启动仿真服务。"""

from __future__ import annotations

import logging
from typing import Annotated

import typer

from artemis_mudri.application.simulation_service import serve as serve_simulation
from artemis_mudri.domains.task import available_tasks

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
)
logger = logging.getLogger(__name__)

app = typer.Typer(
    add_completion=False,
    help="启动自动行驶小车 MuJoCo 仿真 gRPC 服务。",
)


@app.callback()
def root() -> None:
    """自动行驶小车仿真服务。"""


def _validate_task(task: str) -> str:
    """校验任务编号是否合法。"""
    valid_tasks = available_tasks()
    if task not in valid_tasks:
        choices = ", ".join(valid_tasks)
        raise typer.BadParameter(f"Task must be one of: {choices}.")
    return task


@app.command("serve")
def serve_command(
    task: Annotated[
        str,
        typer.Option(
            "--task",
            help="客户端未指定任务时使用的默认任务编号。",
            metavar="TASK_ID",
            show_default=True,
            envvar="ARTEMIS_TASK",
            show_envvar=True,
        ),
    ] = "1",
    host: Annotated[
        str,
        typer.Option(
            "--host",
            help="gRPC 服务监听地址。",
            envvar="ARTEMIS_SIM_HOST",
            show_envvar=True,
        ),
    ] = "127.0.0.1",
    port: Annotated[
        int,
        typer.Option(
            "--port",
            help="gRPC 服务监听端口。",
            envvar="ARTEMIS_SIM_PORT",
            show_envvar=True,
        ),
    ] = 50051,
    render: Annotated[
        bool,
        typer.Option(
            "--render/--no-render",
            help="是否在 episode 中启动 MuJoCo viewer。",
            envvar="ARTEMIS_RENDER",
            show_envvar=True,
        ),
    ] = False,
    max_workers: Annotated[
        int,
        typer.Option(
            "--max-workers",
            help="gRPC server 线程池大小。",
            envvar="ARTEMIS_SIM_MAX_WORKERS",
            show_envvar=True,
        ),
    ] = 4,
) -> None:
    """启动仿真服务，等待小车客户端连接。"""

    validated_task = _validate_task(task)
    logger.info(
        "Starting simulation service host=%s port=%s default_task=%s render=%s",
        host,
        port,
        validated_task,
        render,
    )
    serve_simulation(
        host=host,
        port=port,
        default_task_id=validated_task,
        render=render,
        max_workers=max_workers,
    )


def main() -> None:
    """CLI 主入口。"""
    app()


if __name__ == "__main__":
    app()
