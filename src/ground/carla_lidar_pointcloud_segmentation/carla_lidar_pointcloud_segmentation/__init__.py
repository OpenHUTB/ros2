"""激光雷达点云语义分割：用 CARLA 的语义激光雷达当监督信号训练普通点云的逐点分类器。

模块划分（除 ``app``/``node`` 外都不 import carla，可离线单测）：

* :mod:`labels`   语义标签映射与类别权重
* :mod:`features` 逐点特征工程、体素下采样、标准化
* :mod:`metrics`  混淆矩阵、IoU/F1、分距离评价
* :mod:`model`    纯 NumPy 的多层感知机

``app`` 负责在 CARLA 里采集"普通点云 + 语义真值"成对数据；``node`` 是 ROS 2 节点。
"""

from . import control, features, labels, metrics, model, pipeline  # noqa: F401

__all__ = ['control', 'features', 'labels', 'metrics', 'model', 'pipeline']
__version__ = '0.1.0'