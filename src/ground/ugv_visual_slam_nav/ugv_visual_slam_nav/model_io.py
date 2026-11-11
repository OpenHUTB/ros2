"""模型权重读写。

节点启动时按 ament share 目录找 models/*.npz，因此 colcon install 之后
不需要关心 CWD 在哪；没有 ROS 环境时退回到源码树里的 models/。
"""

import os

import numpy as np


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return get_package_share_directory('ugv_visual_slam_nav')
    except Exception:
        return None


def default_model_dir():
    s = share_dir()
    if s:
        cand = os.path.join(s, 'models')
        if os.path.isdir(cand):
            return cand
    return os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                        'models')


def save_net(net, path):
    np.savez(path, **net.state())
    return path


def load_net(path):
    from .cnn import CNN

    with np.load(path) as data:
        h, w = int(data['shape'][0]), int(data['shape'][1])
        net = CNN(h, w)
        net.load_state({k: np.array(data[k]) for k in data.files})
    return net
