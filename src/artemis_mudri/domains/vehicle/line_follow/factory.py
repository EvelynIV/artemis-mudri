from __future__ import annotations
"""循线控制器工厂。"""

from artemis_mudri.domains.vehicle.line_follow.base import BaseLineFollower
from artemis_mudri.domains.vehicle.line_follow.lqr import LqrLineFollower, LqrLineFollowerConfig
from artemis_mudri.domains.vehicle.line_follow.pd import PdLineFollower, PdLineFollowerConfig
from artemis_mudri.domains.vehicle.line_follow.pid import PidLineFollower, PidLineFollowerConfig

LineFollowerConfig = PidLineFollowerConfig | PdLineFollowerConfig | LqrLineFollowerConfig


def create_line_follower(config: LineFollowerConfig | None = None) -> BaseLineFollower:
    """根据配置实例创建具体的循线控制器。"""
    if config is None:
        return PidLineFollower()
    if isinstance(config, PidLineFollowerConfig):
        return PidLineFollower(config)
    if isinstance(config, PdLineFollowerConfig):
        return PdLineFollower(config)
    if isinstance(config, LqrLineFollowerConfig):
        return LqrLineFollower(config)
    raise TypeError(f"Unsupported line follower config: {type(config)!r}")
