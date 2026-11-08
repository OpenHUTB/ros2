"""神经网络策略封装（分层结构）。

网络输入：船体系 21x21 局部占据栅格 + 目标相对位姿
网络输出：船体系下的局部目标点 (gx, gy)，单位归一化为 5 m
底层控制器：把局部目标点转成 (v, w)，并做限幅、平滑与安全限速

之所以让网络输出"目标点"而不是直接输出 (v, w)：局部栅格只能反映
近处是否可通行，无法唯一确定绝对速度，直接回归动作会因信息缺口
而退化为"朝目标直冲"。输出目标点则与人驾驶时的决策层级一致。
"""

import os

import numpy as np

from nn import MLP

CROP = 21
TARGET_SCALE = 5.0      # 网络输出 1.0 对应 5 m


class NNPolicy(object):
    def __init__(self, model_dir):
        self.net = MLP.load(os.path.join(model_dir, "planner_mlp.npz"))
        z = np.load(os.path.join(model_dir, "norm_stats.npz"))
        self.xm, self.xs = z["xm"], z["xs"]
        self.ym, self.ys = z["ym"], z["ys"]
        self.v = None
        self.w = None

    def local_target(self, feat):
        xn = (feat[None, :] - self.xm) / self.xs
        yn = self.net.predict(xn)[0]
        gx = yn[0] * self.ys[0] + self.ym[0]
        gy = yn[1] * self.ys[1] + self.ym[1]
        return float(gx * TARGET_SCALE), float(gy * TARGET_SCALE)

    def act(self, feat, scan=None, v_max=1.15):
        gx, gy = self.local_target(feat)
        dist = float(np.hypot(gx, gy))
        alpha = float(np.arctan2(gy, gx))

        # 与纯跟踪一致的几何控制律
        w = float(np.clip(0.9 * alpha, -1.0, 1.0))
        v = v_max * max(0.0, 1.0 - abs(alpha) / 1.1) * min(1.0, 0.35 + dist / 4.0)

        # 一阶低通平滑，抑制逐帧抖动引起的附加回转
        a = 0.45
        self.v = a * v + (1 - a) * (self.v if self.v is not None else v)
        self.w = a * w + (1 - a) * (self.w if self.w is not None else w)

        # 安全约束：前向最小测距越近，限速越低
        if scan is not None:
            n = scan.shape[0]
            front = scan[int(0.30 * n):int(0.70 * n)]
            d_min = float(np.min(front))
            if d_min < 2.2:
                self.v = min(self.v, max(0.0, 0.45 * (d_min - 0.9))
                             )
        return self.v, self.w

    def reset(self):
        self.v = None
        self.w = None
