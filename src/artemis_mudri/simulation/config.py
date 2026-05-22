from __future__ import annotations
"""MuJoCo 世界与车体的静态配置。"""

from dataclasses import dataclass

from artemis_mudri.vehicle import LineSensorArrayConfig


@dataclass(frozen=True)
class VisualHeadlight:
    """头灯光照参数。"""

    ambient: tuple[float, float, float]
    diffuse: tuple[float, float, float]
    specular: tuple[float, float, float]


@dataclass(frozen=True)
class VisualConfig:
    """视觉表现相关配置。"""

    headlight: VisualHeadlight
    haze_rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class DefaultGeomConfig:
    """默认几何体材质配置。"""

    friction: tuple[float, float, float]
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class ActuatorConfig:
    """执行器定义。"""

    name: str
    joint: str
    kv: float


@dataclass(frozen=True)
class BoundaryConfig:
    """场地边界几何定义。"""

    name: str
    fromto: tuple[float, float, float, float, float, float]
    radius: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class AnchorSiteStyle:
    """锚点可视化样式。"""

    size: float
    z: float
    fallback_rgba: tuple[float, float, float, float]
    rgba_by_name: dict[str, tuple[float, float, float, float]]


@dataclass(frozen=True)
class SensorSiteStyle:
    """传感器可视化样式。"""

    z: float
    size: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class LedSiteStyle:
    """LED 可视化样式。"""

    forward_offset_m: float
    z: float
    size: float
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class BodyGeomConfig:
    """车体几何配置。"""

    name: str
    geom_type: str
    size: tuple[float, ...]
    rgba: tuple[float, float, float, float]
    contype: int = 0
    conaffinity: int = 0
    euler_deg: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class SiteConfig:
    """MuJoCo site 配置。"""

    name: str
    pos: tuple[float, float, float]
    size: float
    site_type: str
    rgba: tuple[float, float, float, float]


@dataclass(frozen=True)
class WheelBodyConfig:
    """单个车轮刚体配置。"""

    name: str
    pos: tuple[float, float, float]
    geom: BodyGeomConfig
    steer_joint_name: str | None = None
    steer_joint_axis: tuple[float, float, float] | None = None


@dataclass(frozen=True)
class CarBodyConfig:
    """车体整体配置。"""

    body_name: str
    pos: tuple[float, float, float]
    joints: tuple[tuple[str, str, tuple[float, float, float]], ...]
    chassis: BodyGeomConfig
    nose_site: SiteConfig
    wheels: tuple[WheelBodyConfig, ...]
    sensor_site_style: SensorSiteStyle
    led_site_style: LedSiteStyle


@dataclass(frozen=True)
class WorldConfig:
    """完整 MuJoCo 场景配置。"""

    model_name: str
    compiler_angle: str
    compiler_coordinate: str
    timestep_s: float
    gravity: tuple[float, float, float]
    integrator: str
    joint_damping: float
    default_geom: DefaultGeomConfig
    default_velocity_kv: float
    visual: VisualConfig
    ground_rgba: tuple[float, float, float, float]
    field_rgba: tuple[float, float, float, float]
    field_thickness: float
    track_rgba: tuple[float, float, float, float]
    track_z: float
    route_rgba: tuple[float, float, float, float]
    route_z: float
    route_line_radius: float
    boundaries: tuple[BoundaryConfig, ...]
    anchor_site_style: AnchorSiteStyle
    actuators: tuple[ActuatorConfig, ...]
    car: CarBodyConfig
    sensor_array: LineSensorArrayConfig


DEFAULT_WORLD_CONFIG = WorldConfig(
    model_name="artemis_mudri_demo",
    compiler_angle="degree",
    compiler_coordinate="local",
    timestep_s=0.01,
    gravity=(0.0, 0.0, -9.81),
    integrator="RK4",
    joint_damping=0.8,
    default_geom=DefaultGeomConfig(
        friction=(0.8, 0.1, 0.1),
        rgba=(0.88, 0.90, 0.92, 1.0),
    ),
    default_velocity_kv=80.0,
    visual=VisualConfig(
        headlight=VisualHeadlight(
            ambient=(0.45, 0.45, 0.45),
            diffuse=(0.65, 0.65, 0.65),
            specular=(0.15, 0.15, 0.15),
        ),
        haze_rgba=(1.0, 1.0, 1.0, 1.0),
    ),
    ground_rgba=(0.98, 0.98, 0.98, 1.0),
    field_rgba=(1.0, 1.0, 1.0, 1.0),
    field_thickness=0.001,
    track_rgba=(0.05, 0.05, 0.05, 1.0),
    track_z=0.001,
    route_rgba=(0.12, 0.38, 0.80, 0.65),
    route_z=0.006,
    route_line_radius=0.004,
    boundaries=(
        BoundaryConfig(
            name="boundary_top",
            fromto=(0.0, 0.0, 0.002, 2.2, 0.0, 0.002),
            radius=0.004,
            rgba=(0.55, 0.55, 0.55, 1.0),
        ),
        BoundaryConfig(
            name="boundary_bottom",
            fromto=(0.0, 1.2, 0.002, 2.2, 1.2, 0.002),
            radius=0.004,
            rgba=(0.55, 0.55, 0.55, 1.0),
        ),
        BoundaryConfig(
            name="boundary_left",
            fromto=(0.0, 0.0, 0.002, 0.0, 1.2, 0.002),
            radius=0.004,
            rgba=(0.55, 0.55, 0.55, 1.0),
        ),
        BoundaryConfig(
            name="boundary_right",
            fromto=(2.2, 0.0, 0.002, 2.2, 1.2, 0.002),
            radius=0.004,
            rgba=(0.55, 0.55, 0.55, 1.0),
        ),
    ),
    anchor_site_style=AnchorSiteStyle(
        size=0.02,
        z=0.01,
        fallback_rgba=(0.5, 0.5, 0.5, 1.0),
        rgba_by_name={
            "A": (0.87, 0.20, 0.20, 1.0),
            "B": (0.18, 0.56, 0.20, 1.0),
            "C": (0.16, 0.38, 0.76, 1.0),
            "D": (0.88, 0.62, 0.10, 1.0),
        },
    ),
    actuators=(
        ActuatorConfig(name="x_drive", joint="car_x", kv=120.0),
        ActuatorConfig(name="y_drive", joint="car_y", kv=120.0),
        ActuatorConfig(name="yaw_drive", joint="car_yaw", kv=45.0),
    ),
    car=CarBodyConfig(
        body_name="car",
        pos=(0.0, 0.0, 0.05),
        joints=(
            ("car_x", "slide", (1.0, 0.0, 0.0)),
            ("car_y", "slide", (0.0, 1.0, 0.0)),
            ("car_yaw", "hinge", (0.0, 0.0, 1.0)),
        ),
        chassis=BodyGeomConfig(
            name="chassis",
            geom_type="box",
            size=(0.09, 0.06, 0.025),
            rgba=(0.22, 0.25, 0.31, 1.0),
        ),
        nose_site=SiteConfig(
            name="car_nose",
            pos=(0.09, 0.0, 0.01),
            size=0.014,
            site_type="sphere",
            rgba=(0.90, 0.25, 0.25, 1.0),
        ),
        wheels=(
            WheelBodyConfig(
                name="front_left_wheel",
                pos=(0.055, 0.07, -0.005),
                steer_joint_name="front_left_steer",
                steer_joint_axis=(0.0, 0.0, 1.0),
                geom=BodyGeomConfig(
                    name="front_left_wheel_geom",
                    geom_type="cylinder",
                    size=(0.03, 0.014),
                    rgba=(0.10, 0.10, 0.10, 1.0),
                    euler_deg=(90.0, 0.0, 0.0),
                ),
            ),
            WheelBodyConfig(
                name="front_right_wheel",
                pos=(0.055, -0.07, -0.005),
                steer_joint_name="front_right_steer",
                steer_joint_axis=(0.0, 0.0, 1.0),
                geom=BodyGeomConfig(
                    name="front_right_wheel_geom",
                    geom_type="cylinder",
                    size=(0.03, 0.014),
                    rgba=(0.10, 0.10, 0.10, 1.0),
                    euler_deg=(90.0, 0.0, 0.0),
                ),
            ),
            WheelBodyConfig(
                name="rear_left_wheel",
                pos=(-0.055, 0.07, -0.005),
                geom=BodyGeomConfig(
                    name="rear_left_wheel_geom",
                    geom_type="cylinder",
                    size=(0.03, 0.014),
                    rgba=(0.10, 0.10, 0.10, 1.0),
                    euler_deg=(90.0, 0.0, 0.0),
                ),
            ),
            WheelBodyConfig(
                name="rear_right_wheel",
                pos=(-0.055, -0.07, -0.005),
                geom=BodyGeomConfig(
                    name="rear_right_wheel_geom",
                    geom_type="cylinder",
                    size=(0.03, 0.014),
                    rgba=(0.10, 0.10, 0.10, 1.0),
                    euler_deg=(90.0, 0.0, 0.0),
                ),
            ),
        ),
        sensor_site_style=SensorSiteStyle(
            z=-0.015,
            size=0.005,
            rgba=(0.95, 0.65, 0.20, 1.0),
        ),
        led_site_style=LedSiteStyle(
            forward_offset_m=0.035,
            z=0.035,
            size=0.006,
            rgba=(0.16, 0.16, 0.16, 1.0),
        ),
    ),
    sensor_array=LineSensorArrayConfig(),
)
