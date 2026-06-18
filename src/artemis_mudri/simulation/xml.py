from __future__ import annotations
"""MuJoCo XML 渲染工具。"""

from xml.sax.saxutils import escape

from artemis_mudri.simulation.car_model import build_car_actuator_xml, build_car_worldbody_xml
from artemis_mudri.simulation.config import DEFAULT_WORLD_CONFIG, WorldConfig
from artemis_mudri.simulation.track_model import build_track_worldbody_xml
from artemis_mudri.simulation.xml_common import format_floats
from artemis_mudri.track import RoutePlan


def build_demo_model_xml(
    route: RoutePlan,
    resolution: float = 0.03,
    world: WorldConfig = DEFAULT_WORLD_CONFIG,
) -> str:
    """生成完整 MuJoCo 教学演示 XML。"""

    track_worldbody = build_track_worldbody_xml(route=route, world=world, resolution=resolution)
    car_worldbody = build_car_worldbody_xml(world)
    car_actuators = build_car_actuator_xml(world)

    return f"""
<mujoco model="{escape(world.model_name)}">
  <compiler angle="{escape(world.compiler_angle)}" coordinate="{escape(world.compiler_coordinate)}"/>
  <option timestep="{world.timestep_s:.2f}" gravity="{format_floats(world.gravity, precision=2)}" integrator="{escape(world.integrator)}"/>
  <visual>
    <headlight ambient="{format_floats(world.visual.headlight.ambient)}" diffuse="{format_floats(world.visual.headlight.diffuse)}" specular="{format_floats(world.visual.headlight.specular)}"/>
    <rgba haze="{format_floats(world.visual.haze_rgba)}"/>
  </visual>
  <default>
    <joint damping="{world.joint_damping:.1f}"/>
    <geom friction="{format_floats(world.default_geom.friction, precision=1)}" rgba="{format_floats(world.default_geom.rgba)}"/>
    <velocity kv="{world.default_velocity_kv:.0f}"/>
  </default>
  <worldbody>
    {track_worldbody}
    {car_worldbody}
  </worldbody>
  <actuator>
    {car_actuators}
  </actuator>
</mujoco>
""".strip()
