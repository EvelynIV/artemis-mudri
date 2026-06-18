from __future__ import annotations
"""ZeroMQ JSON request/reply transport for simulation episodes."""

import logging
from collections.abc import Callable
from typing import Any

logger = logging.getLogger(__name__)

JsonObject = dict[str, Any]
JsonHandler = Callable[[Any], JsonObject]


class ZmqJsonSimulationServer:
    """Blocking ZMQ REP server for one JSON handler."""

    def __init__(self, *, bind: str, handler: JsonHandler, close_handler: Callable[[], None]) -> None:
        self.bind = bind
        self.handler = handler
        self.close_handler = close_handler
        self._socket: Any | None = None

    def serve_forever(self) -> None:
        """Bind the REP socket and serve JSON request/reply traffic."""

        import zmq

        context = zmq.Context.instance()
        socket = context.socket(zmq.REP)
        socket.bind(self.bind)
        self._socket = socket
        logger.info("ZMQ JSON simulation service listening on %s", self.bind)
        while True:
            request = socket.recv_json()
            socket.send_json(self.handler(request))

    def close(self) -> None:
        """Close the REP socket and service resources."""

        if self._socket is not None:
            self._socket.close(linger=0)
            self._socket = None
        self.close_handler()
