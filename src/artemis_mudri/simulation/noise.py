from __future__ import annotations
"""仿真噪声配置与 YAML 加载。"""

from dataclasses import dataclass, field, fields, is_dataclass
from pathlib import Path
from typing import Any, get_type_hints

from omegaconf import DictConfig, OmegaConf


@dataclass
class InitialPoseNoiseConfig:
    """初始位姿扰动配置。"""

    enabled: bool = False
    x_std_m: float = 0.0
    y_std_m: float = 0.0
    yaw_uniform_deg: float = 0.0


@dataclass
class LineSensorNoiseConfig:
    """巡线传感器噪声配置。"""

    enabled: bool = False
    darkness_std: float = 0.0
    threshold_std: float = 0.0
    dropout_prob: float = 0.0
    false_positive_prob: float = 0.0


@dataclass
class ImuNoiseConfig:
    """IMU 噪声配置。"""

    enabled: bool = False
    yaw_std_deg: float = 0.0
    yaw_rate_std_deg_s: float = 0.0
    yaw_bias_deg: float = 0.0
    yaw_rate_bias_deg_s: float = 0.0
    yaw_bias_random_walk_std_deg_per_sqrt_s: float = 0.0


@dataclass
class EncoderNoiseConfig:
    """编码器噪声配置。"""

    enabled: bool = False
    pulse_std: float = 0.0
    speed_std: float = 0.0
    dropout_prob: float = 0.0
    quantize: bool = False


@dataclass
class ActuatorNoiseConfig:
    """执行器噪声配置。"""

    enabled: bool = False
    left_gain_std: float = 0.0
    right_gain_std: float = 0.0
    command_std_pulse_per_tick: float = 0.0
    speed_response_alpha_std: float = 0.0


@dataclass
class NoiseConfig:
    """一次 episode 使用的完整噪声配置。"""

    preset: str = "custom"
    seed: int | None = None
    initial_pose: InitialPoseNoiseConfig = field(default_factory=InitialPoseNoiseConfig)
    line_sensor: LineSensorNoiseConfig = field(default_factory=LineSensorNoiseConfig)
    imu: ImuNoiseConfig = field(default_factory=ImuNoiseConfig)
    encoder: EncoderNoiseConfig = field(default_factory=EncoderNoiseConfig)
    actuator: ActuatorNoiseConfig = field(default_factory=ActuatorNoiseConfig)


DEFAULT_NOISE_CONFIG = NoiseConfig()


def load_noise_config(path: Path | None) -> NoiseConfig:
    """从 YAML 文件加载噪声配置；未传路径时返回默认关闭配置。"""

    if path is None:
        return DEFAULT_NOISE_CONFIG
    if not path.exists():
        raise FileNotFoundError(f"Noise config file not found: {path}")
    raw = OmegaConf.load(path)
    if not isinstance(raw, DictConfig):
        raise ValueError(f"Noise config must be a mapping: {path}")
    for key in raw.keys():
        if key != "noise":
            raise ValueError(f"Unknown noise config key: {key}")
    if "noise" not in raw:
        raise ValueError(f"Noise config must contain top-level 'noise': {path}")
    payload = raw.get("noise")
    if payload is None:
        raise ValueError(f"Noise config must contain top-level 'noise': {path}")
    _validate_unknown_keys(OmegaConf.to_container(payload, resolve=True), NoiseConfig, "noise")
    base = OmegaConf.structured(NoiseConfig)
    OmegaConf.set_readonly(base, False)
    merged = OmegaConf.merge(base, payload)
    config = OmegaConf.to_object(merged)
    if not isinstance(config, NoiseConfig):
        raise TypeError(f"Invalid noise config loaded from {path}")
    validate_noise_config(config)
    return config


def validate_noise_config(config: NoiseConfig) -> None:
    """校验噪声配置取值范围。"""

    _validate_non_negative(
        config.initial_pose,
        "x_std_m",
        "y_std_m",
        "yaw_uniform_deg",
    )
    _validate_non_negative(
        config.line_sensor,
        "darkness_std",
        "threshold_std",
    )
    _validate_probabilities(config.line_sensor, "dropout_prob", "false_positive_prob")
    _validate_non_negative(
        config.imu,
        "yaw_std_deg",
        "yaw_rate_std_deg_s",
        "yaw_bias_random_walk_std_deg_per_sqrt_s",
    )
    _validate_non_negative(config.encoder, "pulse_std", "speed_std")
    _validate_probabilities(config.encoder, "dropout_prob")
    _validate_non_negative(
        config.actuator,
        "left_gain_std",
        "right_gain_std",
        "command_std_pulse_per_tick",
        "speed_response_alpha_std",
    )


def enabled_noise_modules(config: NoiseConfig) -> tuple[str, ...]:
    """返回当前启用的噪声模块名称。"""

    modules = []
    for name in ("initial_pose", "line_sensor", "imu", "encoder", "actuator"):
        module = getattr(config, name)
        if bool(getattr(module, "enabled")):
            modules.append(name)
    return tuple(modules)


def _validate_unknown_keys(value: Any, schema: type[Any], path: str) -> None:
    if not isinstance(value, dict):
        raise ValueError(f"{path} must be a mapping")
    type_hints = get_type_hints(schema)
    allowed = {field.name: type_hints[field.name] for field in fields(schema)}
    for key, child in value.items():
        if key not in allowed:
            raise ValueError(f"Unknown noise config key: {path}.{key}")
        child_schema = allowed[key]
        if isinstance(child_schema, str):
            continue
        if is_dataclass(child_schema) and isinstance(child, dict):
            _validate_unknown_keys(child, child_schema, f"{path}.{key}")


def _validate_non_negative(config: object, *names: str) -> None:
    for name in names:
        value = float(getattr(config, name))
        if value < 0.0:
            raise ValueError(f"{type(config).__name__}.{name} must be non-negative")


def _validate_probabilities(config: object, *names: str) -> None:
    for name in names:
        value = float(getattr(config, name))
        if value < 0.0 or value > 1.0:
            raise ValueError(f"{type(config).__name__}.{name} must be in [0, 1]")
