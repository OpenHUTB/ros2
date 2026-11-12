# -*- coding: utf-8 -*-
"""人形机器人神经网络行走控制：CPG 基线 + 四条学习路线的可行性分析。

本模块交付的重点是**诚实的实验矩阵**：手工 CPG 能走，而 ARS / 残差 / 行为克隆 /
DAgger 四种学习路线在本模型与本算力预算下都无法复现它。
"""

from humanoid_ars_phase_walk.config import VERSION

__version__ = VERSION

__all__ = ["VERSION", "__version__"]