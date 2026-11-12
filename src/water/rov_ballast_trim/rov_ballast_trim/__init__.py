"""ROV 浮力-压载配平与初稳性核算（只依赖标准库）."""

from .core import (
    ADDED_MASS_FACTOR,
    DEFAULT_COMPONENTS,
    GM_MIN,
    GM_RECOMMENDED,
    G,
    NET_BUOYANCY_TOL,
    RHO_NOMINAL,
    TRIM_LIMIT_DEG,
    Component,
    RovTrim,
    build_components,
    cog,
    gm_level,
    roll_period_s,
    trim_angle_deg,
    water_density,
)

__all__ = ['RovTrim', 'Component', 'build_components', 'cog', 'gm_level',
           'roll_period_s', 'trim_angle_deg', 'water_density',
           'DEFAULT_COMPONENTS', 'G', 'RHO_NOMINAL', 'GM_MIN', 'GM_RECOMMENDED',
           'TRIM_LIMIT_DEG', 'NET_BUOYANCY_TOL', 'ADDED_MASS_FACTOR']
