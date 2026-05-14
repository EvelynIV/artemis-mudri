# Artemis MuJoCo 仿真服务

本项目提供自动行驶小车题目的 MuJoCo 仿真服务。主包只保留赛道、任务、硬件观测、电机速度环、平面车辆推进、仿真统计和 gRPC 服务；小车固件控制器与客户端逻辑位于 sibling 项目 `../artemis-vicon`。

## 功能内容

- 根据题目描述重建赛场几何与 task0-4 任务路线。
- 通过 gRPC 双向流发送 8 路循迹、MS901M 风格 Yaw、编码器和电机 PID 调试帧。
- 服务端接收 artemis-m0 风格的 `Velocity`、`Turn`、后左/后右目标速度和电机模式。
- 服务端按 20ms 控制 tick 模拟 C 端增量式电机速度 PID、编码器累计和后双驱平面车辆运动。
- 支持无界面仿真和可选 MuJoCo 交互式查看器。
- 提供 proto 生成脚本、主包服务测试和 MuJoCo 差速后端测试。

## 项目结构

- `artemis_mudri.commands.app`：标准 Typer CLI 入口，用于启动仿真服务。
- `artemis_mudri.application.simulation_service`：gRPC 服务应用层。
- `artemis_mudri.domains.geometry`：几何类型与采样工具。
- `artemis_mudri.domains.track`：赛场布局与参考路径建模。
- `artemis_mudri.domains.task`：任务目录加载与路线规划。
- `artemis_mudri.domains.vehicle`：8 路传感器、车辆状态、电机 PID 和编码器标定。
- `artemis_mudri.infrastructure.mujoco`：MuJoCo 配置、XML 生成和仿真后端。
- `artemis_mudri.simulation.v1`：由 proto 生成的 gRPC/Protobuf 代码。
- `protos`：仿真通信协议定义。

小车客户端示例和控制器已拆分到 sibling 项目 `../artemis-vicon`，本仓库不再包含客户端 examples。

## 安装

项目使用 Poetry 管理依赖和虚拟环境：

```bash
poetry install
```

## 生成 Proto 代码

```bash
bash scripts/run_grpcio_tools.sh
```

脚本会使用 Poetry 环境运行 `grpcio-tools`，并生成 `*_pb2.py`、`*_pb2.pyi`、`*_pb2_grpc.py` 和 `*_pb2_grpc.pyi`。

## 运行

启动仿真服务：

```bash
poetry run python -m artemis_mudri.commands.app serve --host 127.0.0.1 --port 50051 --task 1 --no-render
```

然后在 `../artemis-vicon` 中启动客户端，例如：

```bash
poetry run python -m artemis_vicon.commands.app --target 127.0.0.1:50051 --task 1 --seed 7
```

## 配置

仿真服务常用环境变量：

- `ARTEMIS_TASK`：默认任务编号。
- `ARTEMIS_SIM_HOST`：gRPC 服务监听地址。
- `ARTEMIS_SIM_PORT`：gRPC 服务监听端口。
- `ARTEMIS_RENDER`：是否启动 MuJoCo viewer。
- `ARTEMIS_SIM_MAX_WORKERS`：gRPC server 线程池大小。

客户端可在 `StartEpisodeRequest` 中传入控制周期、初始航向随机种子和初始航向扰动范围；未指定时按 20ms 控制 tick 与 ±5° 初始 yaw 扰动运行。

## 测试

```bash
poetry run python -m unittest discover -s tests -v
```
