# -*- coding: utf-8 -*-
"""人形上下台阶：ZMP 预瞄控制 + 解析 IK。

子模块
------
config   常量
scene    参数化台阶场景 + 平面双足模型
zmp      LIPM 与预瞄控制（纯 numpy + scipy 求 Riccati）
gait     落脚点 -> ZMP 参考 -> CoM -> 关节角
sim      MuJoCo 播放与接触/穿透测量
"""

from humanoid_stair_zmp_preview.config import VERSION

__version__ = VERSION

__all__ = ["VERSION", "__version__"]