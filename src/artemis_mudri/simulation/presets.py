from __future__ import annotations
"""仿真与渲染共用的预定义场景。"""

from dataclasses import dataclass

from artemis_mudri.simulation.episode import DifferentialSimulation, MotorDriverConfig
from artemis_mudri.simulation.noise import NoiseConfig
from artemis_mudri.track import RoutePlan, build_default_route


@dataclass(frozen=True)
class SimulationPreset:
    """服务端和渲染客户端共享的固定场景预设。"""

    name: str
    route_resolution: float = 0.02
    model_resolution: float = 0.03

    def build_route(self) -> RoutePlan:
        """构建该预设对应的参考路线。"""

        return build_default_route(resolution=self.route_resolution)

    def build_simulation(
        self,
        *,
        motor_driver: MotorDriverConfig | None = None,
        noise_config: NoiseConfig | None = None,
        random_seed: int | None = None,
        initial_pose: tuple[float, float, float] | None = None,
        initial_progress_index: int = 0,
    ) -> DifferentialSimulation:
        """构建该预设对应的 MuJoCo 仿真实例。"""

        return DifferentialSimulation(
            route=self.build_route(),
            route_resolution=self.model_resolution,
            motor_driver=motor_driver,
            noise_config=noise_config,
            random_seed=random_seed,
            initial_pose=initial_pose,
            initial_progress_index=initial_progress_index,
        )


DEFAULT_SIMULATION_PRESET = SimulationPreset(name="default")
