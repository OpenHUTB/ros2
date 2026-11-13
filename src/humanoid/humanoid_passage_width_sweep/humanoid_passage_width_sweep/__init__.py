# -*- coding: utf-8 -*-
"""人形足迹下的局部避障算法对比：窄通道宽度扫描。

子模块
------
config       常量
scene        参数化窄通道场景生成
planners     TEB / MPPI / 质点基线
sim          MuJoCo 封装
experiments  宽度扫描与对比实验
metrics      指标提取
figures      曲线与动图
"""

from humanoid_passage_width_sweep.config import VERSION

__version__ = VERSION

__all__ = ["VERSION", "__version__"]
