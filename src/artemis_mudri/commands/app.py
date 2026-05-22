"""标准命令行入口：启动仿真服务。"""

from __future__ import annotations

import logging
from concurrent import futures
from pathlib import Path
from typing import Annotated

import grpc
import typer

from artemis_mudri.protos.simulation.v1 import vehicle_simulation_pb2_grpc as pb2_grpc
from artemis_mudri.simulation.noise import NoiseConfig, enabled_noise_modules, load_noise_config
from artemis_mudri.simulation.state_publisher import ZmqSimulationStatePublisher
from artemis_mudri.servicer.servicer import VehicleSimulationService

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


def create_grpc_server(
    default_render: bool = False,
    max_workers: int = 4,
    noise_config: NoiseConfig | None = None,
    viewer_state_bind: str | None = None,
) -> grpc.Server:
    """创建已注册 VehicleSimulationService 的 gRPC server。"""

    server = grpc.server(futures.ThreadPoolExecutor(max_workers=max_workers))
    state_publisher = None
    if viewer_state_bind is not None:
        state_publisher = ZmqSimulationStatePublisher(viewer_state_bind)
        state_publisher.start()
    service = VehicleSimulationService(
        default_render=default_render,
        noise_config=noise_config,
        state_publisher=state_publisher,
    )
    pb2_grpc.add_VehicleSimulationServiceServicer_to_server(
        service,
        server,
    )
    setattr(server, "_artemis_vehicle_simulation_service", service)
    return server


def serve_simulation(
    host: str = "127.0.0.1",
    port: int = 50051,
    render: bool = False,
    max_workers: int = 4,
    noise_config_path: Path | None = None,
    viewer_state_bind: str | None = None,
) -> None:
    """启动阻塞式 gRPC 仿真服务。"""

    address = f"{host}:{port}"
    noise_config = load_noise_config(noise_config_path) if noise_config_path is not None else None
    if noise_config is not None:
        logger.info(
            "Loaded noise config preset=%s path=%s enabled=%s",
            noise_config.preset,
            noise_config_path,
            ",".join(enabled_noise_modules(noise_config)) or "none",
        )
    server = create_grpc_server(
        default_render=render,
        max_workers=max_workers,
        noise_config=noise_config,
        viewer_state_bind=viewer_state_bind,
    )
    bound_port = server.add_insecure_port(address)
    if bound_port == 0:
        raise RuntimeError(f"Failed to bind gRPC server to {address}")
    server.start()
    logger.info("Vehicle simulation service listening on %s", address)
    if viewer_state_bind is not None:
        logger.info("Remote viewer state publisher enabled at %s", viewer_state_bind)
    try:
        server.wait_for_termination()
    except KeyboardInterrupt:
        logger.info("Stopping vehicle simulation service")
        server.stop(grace=1.0)
    finally:
        service = getattr(server, "_artemis_vehicle_simulation_service", None)
        if service is not None:
            service.close()


@app.command("serve")
def serve_command(
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
    noise_config: Annotated[
        Path | None,
        typer.Option(
            "--noise-config",
            help="噪声 YAML 配置路径；不传时保持兼容默认仿真。",
            envvar="ARTEMIS_NOISE_CONFIG",
            show_envvar=True,
        ),
    ] = None,
    viewer_state_bind: Annotated[
        str | None,
        typer.Option(
            "--viewer-state-bind",
            help="可选 ZeroMQ PUB 监听地址，用于发布 MuJoCo qpos/qvel 给远程 viewer。",
            envvar="ARTEMIS_VIEWER_STATE_BIND",
            show_envvar=True,
        ),
    ] = None,
) -> None:
    """启动仿真服务，等待小车客户端连接。"""

    logger.info(
        "Starting simulation service host=%s port=%s render=%s",
        host,
        port,
        render,
    )
    try:
        serve_simulation(
            host=host,
            port=port,
            render=render,
            max_workers=max_workers,
            noise_config_path=noise_config,
            viewer_state_bind=viewer_state_bind,
        )
    except (FileNotFoundError, ValueError, TypeError) as exc:
        raise typer.BadParameter(str(exc), param_hint="--noise-config") from exc


def main() -> None:
    """CLI 主入口。"""
    app()


if __name__ == "__main__":
    app()
