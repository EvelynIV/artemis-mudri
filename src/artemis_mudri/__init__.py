"""Teaching-oriented MuJoCo demo for the NUEDC automatic car task."""

from artemis_mudri.domains.simulation import SimulationSummary
from artemis_mudri.domains.task import available_tasks, build_route_plan
from artemis_mudri.infrastructure.mujoco.backend import MuJoCoTeachingDemo

__all__ = [
    "MuJoCoTeachingDemo",
    "SimulationSummary",
    "available_tasks",
    "build_route_plan",
]
