"""USV 弗劳德缩比换算与拖曳功率估算（只依赖标准库）."""

from .core import (
    G,
    NU_SEA,
    RHO_SEA,
    UsvFroude,
    friction_coef_itfc,
    friction_resistance,
    froude_number,
    model_speed_for_froude,
    scale_power,
    scale_resistance,
)

__all__ = ['UsvFroude', 'froude_number', 'model_speed_for_froude',
           'scale_resistance', 'scale_power', 'friction_coef_itfc',
           'friction_resistance', 'G', 'RHO_SEA', 'NU_SEA']
