from __future__ import annotations
"""MuJoCo XML 渲染的基础格式化工具。"""

from xml.sax.saxutils import escape

from artemis_mudri.simulation.config import BodyGeomConfig, SiteConfig


def format_floats(values: tuple[float, ...], precision: int = 3) -> str:
    """按固定精度格式化浮点元组。"""

    return " ".join(f"{value:.{precision}f}" for value in values)


def site_xml(site: SiteConfig) -> str:
    """渲染单个 site 节点。"""

    return (
        f'<site name="{escape(site.name)}" pos="{format_floats(site.pos, precision=4)}" '
        f'size="{site.size:.4f}" type="{escape(site.site_type)}" '
        f'rgba="{format_floats(site.rgba)}"/>'
    )


def geom_xml(geom: BodyGeomConfig) -> str:
    """渲染单个 geom 节点。"""

    attributes = [
        f'name="{escape(geom.name)}"',
        f'type="{escape(geom.geom_type)}"',
        f'size="{format_floats(geom.size, precision=3)}"',
        f'rgba="{format_floats(geom.rgba)}"',
        f'contype="{geom.contype}"',
        f'conaffinity="{geom.conaffinity}"',
    ]
    if geom.euler_deg is not None:
        attributes.append(f'euler="{format_floats(geom.euler_deg, precision=0)}"')
    return f"<geom {' '.join(attributes)}/>"
