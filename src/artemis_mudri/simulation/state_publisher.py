from __future__ import annotations
"""MuJoCo 状态发布工具。"""

import logging
import threading
from typing import Any, Protocol

from artemis_mudri.simulation.episode import DifferentialSimulation

logger = logging.getLogger(__name__)


class SimulationStatePublisher(Protocol):
    """发布 MuJoCo 仿真状态的最小接口。"""

    def publish(self, simulation: DifferentialSimulation) -> None:
        """发布一帧仿真状态。"""

    def close(self) -> None:
        """释放发布器资源。"""


def build_state_message(simulation: DifferentialSimulation) -> dict[str, Any]:
    """构造可通过网络发送的一帧 MuJoCo 状态。"""

    return {
        "time": float(simulation.data.time),
        "qpos": simulation.data.qpos.copy(),
        "qvel": simulation.data.qvel.copy(),
        "sequence_id": int(simulation.sequence_id),
    }


class ZmqSimulationStatePublisher:
    """通过 ZeroMQ PUB socket 发布 MuJoCo qpos/qvel。"""

    def __init__(self, bind: str) -> None:
        self.bind = bind
        self._lock = threading.RLock()
        self._socket: Any | None = None

    def start(self) -> None:
        """绑定 PUB socket。"""

        with self._lock:
            if self._socket is not None:
                return
            import zmq

            ctx = zmq.Context.instance()
            socket = ctx.socket(zmq.PUB)
            socket.bind(self.bind)
            self._socket = socket
            logger.info("MuJoCo state publisher listening on %s", self.bind)

    def publish(self, simulation: DifferentialSimulation) -> None:
        """发布一帧 MuJoCo 状态。"""

        with self._lock:
            if self._socket is None:
                self.start()
            if self._socket is None:
                raise RuntimeError("MuJoCo state publisher socket is not initialized")
            self._socket.send_pyobj(build_state_message(simulation))

    def close(self) -> None:
        """关闭 PUB socket。"""

        with self._lock:
            if self._socket is None:
                return
            self._socket.close(linger=0)
            self._socket = None
