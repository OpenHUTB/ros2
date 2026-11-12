"""神经网络策略与特征抽取（节点、训练、评测、可视化共用）。

之前 `NNPilot` 在 nodes/gap_nav_node.py、scripts/evaluate.py、scripts/main.py
三处各有一份，`features()` 在 train.py 与 evaluate.py 各有一份：
都是逐字重复，改一处就要改四处。这里统一成单一实现。
"""

import os

import numpy as np

from nn import MLP


def features(sim, obs):
    """观测 -> 网络输入：91 束雷达 + 目标方位 + 目标距离 + 当前速度（94 维）。"""
    return np.concatenate([
        obs["scan"],
        [sim.goal_bearing(), min(sim.goal_distance(), 20.0), obs["speed"]],
    ]).astype(np.float64)


class NNPilot(object):
    """加载训练好的 MLP，输出平滑后的 (v, w)。"""

    def __init__(self, model_dir, smooth=0.5):
        self.net = MLP.load(os.path.join(model_dir, "gap_planner.npz"))
        z = np.load(os.path.join(model_dir, "gap_norm.npz"))
        self.xm, self.xs, self.ym, self.ys = z["xm"], z["xs"], z["ym"], z["ys"]
        self.smooth = float(smooth)
        self.v = None
        self.w = None

    def reset(self):
        self.v = None
        self.w = None

    def act(self, feat):
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        v = float(np.clip(yn[0] * self.ys[0] + self.ym[0], -0.3, 1.0))
        w = float(np.clip(yn[1] * self.ys[1] + self.ym[1], -1.2, 1.2))
        a = self.smooth
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)
        return self.v, self.w
