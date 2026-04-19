from __future__ import annotations
"""MuJoCo XML 渲染工具。"""

from xml.sax.saxutils import escape

from artemis_mudri.domains.task import RoutePlan
from artemis_mudri.domains.track import ANCHORS, ARC_LINE_WIDTH_M, FIELD_HEIGHT_M, FIELD_WIDTH_M, OFFICIAL_ARCS
from artemis_mudri.infrastructure.mujoco.config import (
    DEFAULT_WORLD_CONFIG,
    ActuatorConfig,
    BodyGeomConfig,
    BoundaryConfig,
    SiteConfig,
    WheelBodyConfig,
    WorldConfig,
)


def _capsule_chain_xml(
    name_prefix: str,
    points,
    radius: float,
    rgba: tuple[float, float, float, float],
    z: float,
) -> str:
    """将离散路径点渲染为 capsule 链条。"""

    rgba_text = " ".join(f"{value:.3f}" for value in rgba)
    geoms: list[str] = []
    for index, (start, end) in enumerate(zip(points[:-1], points[1:])):
        fromto = (
            f"{start[0]:.4f} {start[1]:.4f} {z:.4f} "
            f"{end[0]:.4f} {end[1]:.4f} {z:.4f}"
        )
        geoms.append(
            f'<geom name="{escape(name_prefix)}_{index}" type="capsule" '
            f'fromto="{fromto}" size="{radius:.4f}" '
            f'rgba="{rgba_text}" contype="0" conaffinity="0"/>'
        )
    return "\n".join(geoms)


def _format_floats(values: tuple[float, ...], precision: int = 3) -> str:
    """按固定精度格式化浮点元组。"""

    return " ".join(f"{value:.{precision}f}" for value in values)


def _site_xml(site: SiteConfig) -> str:
    """渲染单个 site 节点。"""

    return (
        f'<site name="{escape(site.name)}" pos="{_format_floats(site.pos, precision=4)}" '
        f'size="{site.size:.4f}" type="{escape(site.site_type)}" '
        f'rgba="{_format_floats(site.rgba)}"/>'
    )


def _geom_xml(geom: BodyGeomConfig) -> str:
    """渲染单个 geom 节点。"""

    attributes = [
        f'name="{escape(geom.name)}"',
        f'type="{escape(geom.geom_type)}"',
        f'size="{_format_floats(geom.size, precision=3)}"',
        f'rgba="{_format_floats(geom.rgba)}"',
        f'contype="{geom.contype}"',
        f'conaffinity="{geom.conaffinity}"',
    ]
    if geom.euler_deg is not None:
        attributes.append(f'euler="{_format_floats(geom.euler_deg, precision=0)}"')
    return f"<geom {' '.join(attributes)}/>"


def _wheel_body_xml(wheel: WheelBodyConfig) -> str:
    """渲染车轮 body。"""

    lines = [f'<body name="{escape(wheel.name)}" pos="{_format_floats(wheel.pos, precision=3)}">']
    if wheel.steer_joint_name is not None and wheel.steer_joint_axis is not None:
        lines.append(
            "        "
            f'<joint name="{escape(wheel.steer_joint_name)}" type="hinge" '
            f'axis="{_format_floats(wheel.steer_joint_axis, precision=0)}"/>'
        )
    lines.append(f"        {_geom_xml(wheel.geom)}")
    lines.append("      </body>")
    return "\n".join(lines)


def _actuator_xml(actuator: ActuatorConfig) -> str:
    """渲染执行器配置。"""

    return (
        f'<velocity name="{escape(actuator.name)}" joint="{escape(actuator.joint)}" '
        f'kv="{actuator.kv:.0f}"/>'
    )


def _boundary_xml(boundary: BoundaryConfig) -> str:
    """渲染边界几何体。"""

    return (
        f'<geom name="{escape(boundary.name)}" type="capsule" '
        f'fromto="{_format_floats(boundary.fromto, precision=3)}" '
        f'size="{boundary.radius:.3f}" rgba="{_format_floats(boundary.rgba)}" '
        f'contype="0" conaffinity="0"/>'
    )


def _anchor_sites_xml(world: WorldConfig) -> str:
    """渲染所有锚点 site。"""

    sites: list[str] = []
    for name, position in ANCHORS.items():
        rgba = world.anchor_site_style.rgba_by_name.get(name, world.anchor_site_style.fallback_rgba)
        sites.append(
            _site_xml(
                SiteConfig(
                    name=f"anchor_{name}",
                    pos=(float(position[0]), float(position[1]), world.anchor_site_style.z),
                    size=world.anchor_site_style.size,
                    site_type="sphere",
                    rgba=rgba,
                )
            )
        )
    return "\n".join(sites)


def _sensor_sites_xml(world: WorldConfig) -> str:
    """渲染车头传感器 site。"""

    sites: list[str] = []
    for index, lateral_offset in enumerate(world.sensor_array.lateral_offsets_m):
        sites.append(
            _site_xml(
                SiteConfig(
                    name=f"sensor_{index}",
                    pos=(world.sensor_array.forward_offset_m, lateral_offset, world.car.sensor_site_style.z),
                    size=world.car.sensor_site_style.size,
                    site_type="sphere",
                    rgba=world.car.sensor_site_style.rgba,
                )
            )
        )
    return "\n".join(sites)


def _led_sites_xml(world: WorldConfig) -> str:
    """渲染 5 个 LED site。"""

    sites: list[str] = []
    for index, lateral_offset in enumerate(world.sensor_array.lateral_offsets_m):
        sites.append(
            _site_xml(
                SiteConfig(
                    name=f"led_{index}",
                    pos=(
                        world.car.led_site_style.forward_offset_m,
                        lateral_offset,
                        world.car.led_site_style.z,
                    ),
                    size=world.car.led_site_style.size,
                    site_type="sphere",
                    rgba=world.car.led_site_style.rgba,
                )
            )
        )
    return "\n".join(sites)


def build_demo_model_xml(
    route: RoutePlan,
    resolution: float = 0.03,
    world: WorldConfig = DEFAULT_WORLD_CONFIG,
) -> str:
    """生成完整 MuJoCo 教学演示 XML。"""

    del route
    official_track = "\n".join(
        _capsule_chain_xml(
            arc.name,
            arc.sample(resolution),
            radius=0.5 * ARC_LINE_WIDTH_M,
            rgba=world.track_rgba,
            z=world.track_z,
        )
        for arc in OFFICIAL_ARCS
    )
    anchor_sites = _anchor_sites_xml(world)
    sensor_sites = _sensor_sites_xml(world)
    led_sites = _led_sites_xml(world)
    wheel_bodies = "\n      ".join(_wheel_body_xml(wheel) for wheel in world.car.wheels)
    boundaries = "\n    ".join(_boundary_xml(boundary) for boundary in world.boundaries)
    actuators = "\n    ".join(_actuator_xml(actuator) for actuator in world.actuators)
    joints = "\n      ".join(
        f'<joint name="{escape(name)}" type="{joint_type}" axis="{_format_floats(axis, precision=0)}"/>'
        for name, joint_type, axis in world.car.joints
    )

    return f"""
<mujoco model="{escape(world.model_name)}">
  <compiler angle="{escape(world.compiler_angle)}" coordinate="{escape(world.compiler_coordinate)}"/>
  <option timestep="{world.timestep_s:.2f}" gravity="{_format_floats(world.gravity, precision=2)}" integrator="{escape(world.integrator)}"/>
  <visual>
    <headlight ambient="{_format_floats(world.visual.headlight.ambient)}" diffuse="{_format_floats(world.visual.headlight.diffuse)}" specular="{_format_floats(world.visual.headlight.specular)}"/>
    <rgba haze="{_format_floats(world.visual.haze_rgba)}"/>
  </visual>
  <default>
    <joint damping="{world.joint_damping:.1f}"/>
    <geom friction="{_format_floats(world.default_geom.friction, precision=1)}" rgba="{_format_floats(world.default_geom.rgba)}"/>
    <velocity kv="{world.default_velocity_kv:.0f}"/>
  </default>
  <worldbody>
    <geom name="ground" type="plane" pos="{FIELD_WIDTH_M / 2:.3f} {FIELD_HEIGHT_M / 2:.3f} 0" size="3 3 0.1" rgba="{_format_floats(world.ground_rgba)}"/>
    <geom name="field" type="box" pos="{FIELD_WIDTH_M / 2:.3f} {FIELD_HEIGHT_M / 2:.3f} {world.field_thickness:.3f}" size="{FIELD_WIDTH_M / 2:.3f} {FIELD_HEIGHT_M / 2:.3f} {world.field_thickness:.3f}" rgba="{_format_floats(world.field_rgba)}" contype="0" conaffinity="0"/>
    {boundaries}
    {official_track}
    {anchor_sites}
    <body name="{escape(world.car.body_name)}" pos="{_format_floats(world.car.pos, precision=3)}">
      {joints}
      {_geom_xml(world.car.chassis)}
      {_site_xml(world.car.nose_site)}
      {sensor_sites}
      {led_sites}
      {wheel_bodies}
    </body>
  </worldbody>
  <actuator>
    {actuators}
  </actuator>
</mujoco>
""".strip()
