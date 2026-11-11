# -*- coding: utf-8 -*-
"""model_io.py —— 权重目录定位（源码运行与 ros2 run 安装后运行都要能找得到）。

评审意见里最常见的一类问题是"源码能跑、install 之后找不到文件"。这里统一
用 ament 的 share 目录，并在 import 失败（比如在纯 python 环境里跑 main.py）
时回退到源码目录，两条路径都能工作。
"""
import os


def _pkg_root():
    return os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def models_dir():
    """权重目录：优先使用 install 后的 share/uav_camera_tracking/models。"""
    try:
        from ament_index_python.packages import get_package_share_directory
        share = os.path.join(get_package_share_directory('uav_camera_tracking'),
                             'models')
        if os.path.isdir(share):
            return share
    except Exception:          # ament 不可用（未 source 环境 / 纯 python 运行）
        pass
    return os.path.join(_pkg_root(), 'models')


def model_path(name, override=''):
    """返回某个权重文件的绝对路径，供节点参数默认值使用。"""
    if override:
        return override
    return os.path.join(models_dir(), name)
