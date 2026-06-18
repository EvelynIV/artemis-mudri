from __future__ import annotations
"""JSON API service for simulation sessions."""

import logging
from typing import Any

from artemis_mudri.api.json_codec import (
    JsonObject,
    error_response,
    parse_start_request,
    parse_step_request,
    result_to_dict,
)
from artemis_mudri.runtime import SimulationEpisodeRunner

logger = logging.getLogger(__name__)


class JsonSimulationService:
    """Validate JSON requests and dispatch them to the runtime."""

    def __init__(self, runner: SimulationEpisodeRunner) -> None:
        self.runner = runner

    def handle(self, message: Any) -> JsonObject:
        """Handle one JSON request and return one JSON response."""

        if not isinstance(message, dict):
            return error_response("Request must be a JSON object.")
        message_type = message.get("type")
        try:
            if message_type == "start":
                return result_to_dict(self.runner.start(parse_start_request(message)))
            if message_type == "step":
                return result_to_dict(self.runner.step(parse_step_request(message)))
            if message_type == "stop":
                return result_to_dict(self.runner.stop(str(message.get("reason") or "client_stopped")))
            return error_response(f"Unsupported request type: {message_type!r}.")
        except KeyError as exc:
            return error_response(f"Missing required field: {exc.args[0]}.")
        except (TypeError, ValueError, RuntimeError) as exc:
            return error_response(str(exc))
        except Exception as exc:  # pragma: no cover - defensive service boundary.
            logger.exception("Simulation request failed")
            return error_response(str(exc))
