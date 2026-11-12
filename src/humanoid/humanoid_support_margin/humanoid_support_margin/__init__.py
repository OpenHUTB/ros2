"""人形机器人支撑多边形与倾倒裕度核算（只依赖标准库）."""

from .core import (
    G,
    HumanoidSupport,
    convex_hull,
    foot_load_fractions,
    point_in_polygon,
    support_margin,
    tipping_force_n,
    tipping_impulse_ns,
)

__all__ = ['HumanoidSupport', 'convex_hull', 'point_in_polygon', 'support_margin',
           'tipping_force_n', 'tipping_impulse_ns', 'foot_load_fractions', 'G']
