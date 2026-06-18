from __future__ import annotations
"""MuJoCo 小车模型渲染。"""

from xml.sax.saxutils import escape

from artemis_mudri.simulation.config import ActuatorConfig, SiteConfig, WheelBodyConfig, WorldConfig
from artemis_mudri.simulation.xml_common import format_floats, geom_xml, site_xml


def _wheel_body_xml(wheel: WheelBodyConfig) -> str:
    """渲染车轮 body。"""

    lines = [f'<body name="{escape(wheel.name)}" pos="{format_floats(wheel.pos, precision=3)}">']
    if wheel.steer_joint_name is not None and wheel.steer_joint_axis is not None:
        lines.append(
            "        "
            f'<joint name="{escape(wheel.steer_joint_name)}" type="hinge" '
            f'axis="{format_floats(wheel.steer_joint_axis, precision=0)}"/>'
        )
    lines.append(f"        {geom_xml(wheel.geom)}")
    lines.append("      </body>")
    return "\n".join(lines)


def _actuator_xml(actuator: ActuatorConfig) -> str:
    """渲染执行器配置。"""

    return (
        f'<velocity name="{escape(actuator.name)}" joint="{escape(actuator.joint)}" '
        f'kv="{actuator.kv:.0f}"/>'
    )


def _sensor_sites_xml(world: WorldConfig) -> str:
    """渲染车头传感器 site。"""

    sites: list[str] = []
    for index, lateral_offset in enumerate(world.sensor_array.lateral_offsets_m):
        sites.append(
            site_xml(
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
            site_xml(
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


def build_car_worldbody_xml(world: WorldConfig) -> str:
    """生成小车 body 片段。"""

    sensor_sites = _sensor_sites_xml(world)
    led_sites = _led_sites_xml(world)
    wheel_bodies = "\n      ".join(_wheel_body_xml(wheel) for wheel in world.car.wheels)
    joints = "\n      ".join(
        f'<joint name="{escape(name)}" type="{joint_type}" axis="{format_floats(axis, precision=0)}"/>'
        for name, joint_type, axis in world.car.joints
    )

    return f"""
    <body name="{escape(world.car.body_name)}" pos="{format_floats(world.car.pos, precision=3)}">
      {joints}
      {geom_xml(world.car.chassis)}
      {site_xml(world.car.nose_site)}
      {sensor_sites}
      {led_sites}
      {wheel_bodies}
    </body>
""".strip()


def build_car_actuator_xml(world: WorldConfig) -> str:
    """生成小车执行器片段。"""

    return "\n    ".join(_actuator_xml(actuator) for actuator in world.actuators)
