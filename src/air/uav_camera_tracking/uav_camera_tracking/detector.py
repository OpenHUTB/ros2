# -*- coding: utf-8 -*-
"""detector.py —— 感知网络：下采样图像 -> 16x16 目标置信度热图 -> 质心。

设计要点（也是相对"直接回归坐标"的改进）：
1. 输出空间热图而不是坐标：网络只需学会"哪个格子更像目标"，不必学习
   "位置->坐标"的查表映射，样本效率高一个量级；
2. 用软标签(以真值为中心的高斯)做交叉熵，等价于可微的软最大化定位；
3. 另设一个标量头做可见性判断(遮挡/出视野)，不可见样本只监督该头，
   避免占位标签污染定位项。

注意：不再回归目标像素宽度。地面目标的图像纵坐标已唯一决定距离，
另设宽度头反而会引入偏差(实测其输出长期偏大)，把距离环带偏。

监督信号来自仿真器的几何投影真值——这在仿真里是免费的，也是"仿真中训练、
真机上微调"这一常见做法的起点。
"""
import numpy as np

from . import params as P
from .camera import Camera
from .nn import MLP


class DetectorNet:
    """轻量感知网络（MLP 主干 + 空间热图检测头）。"""

    HEAT_SIGMA = 0.8      # 软标签高斯宽度(格子数)

    def __init__(self, seed=0):
        self.cam = Camera()
        self.net = MLP([P.DS * P.DS * 3, 96, P.DS * P.DS + 1], seed=seed)
        self.history = {'loss': [], 'val_loss': []}

    # ---------- 数据 ----------
    def features(self, img):
        """感知网络输入(16x16)。"""
        return self.cam.downsample(img, P.DS).ravel()

    def features_e2e(self, img):
        """端到端网络输入(24x24)：没有显式检测头，需要更高空间分辨率。"""
        return self.cam.downsample(img, P.E2E_DS).ravel()

    def build_dataset(self, n=3000, seed=0):
        """随机采位姿渲染图像，标签取几何投影真值。"""
        cam, rng = self.cam, np.random.default_rng(seed)
        X = np.zeros((n, P.DS * P.DS * 3), dtype=np.float32)
        Y = np.zeros((n, 4), dtype=np.float32)
        for i in range(n):
            tx = rng.uniform(-10, 10)
            ty = rng.uniform(-10, 10)
            cam_pos = np.array([
                tx + rng.uniform(-12, 4),
                ty + rng.uniform(-8, 8),
                rng.uniform(4.5, 16.0),
            ])
            yaw = rng.uniform(-0.9, 0.9)
            walls = ()
            if rng.random() < 0.30:      # 30% 样本带立柱遮挡，学会"看不见"
                walls = tuple(
                    (tx + rng.uniform(1.5, 4.0) *
                     np.sign(cam_pos[0] if abs(cam_pos[0]) > 0.5 else 1.0),
                     ty + rng.uniform(-3, 3), 0.28, 0.22, 1.2)
                    for _ in range(int(rng.integers(1, 3))))
            distr = tuple((tx + rng.uniform(-6, 6), ty + rng.uniform(-6, 6))
                          for _ in range(int(rng.integers(0, 3))))
            img = cam.render(cam_pos, yaw, (tx, ty), distr, walls, rng)
            obs = cam.observe(cam_pos, yaw, (tx, ty), walls)
            X[i] = self.features(img)
            if obs['visible']:
                Y[i] = [obs['u'] / P.IMG_W, obs['v'] / P.IMG_H,
                        obs['width_px'] / P.IMG_W, 1.0]
            else:
                Y[i] = [0.5, 0.5, 0.0, 0.0]
        return X, Y

    # ---------- 标签 ----------
    def _heat_target(self, u_norm, v_norm):
        """归一化坐标 -> 16x16 高斯软标签（行优先展平）。"""
        ds = P.DS
        cx = u_norm * ds - 0.5
        cy = v_norm * ds - 0.5
        idx = np.arange(ds)
        gx, gy = np.meshgrid(idx, idx)
        d2 = (gx - cx) ** 2 + (gy - cy) ** 2
        q = np.exp(-d2 / (2.0 * self.HEAT_SIGMA ** 2)).reshape(-1)
        s = q.sum()
        return q / s if s > 1e-9 else np.full(ds * ds, 1.0 / (ds * ds))

    # ---------- 训练 ----------
    def fit(self, X, Y, epochs=60, batch=64, lr=2e-3, val_split=0.15,
            seed=0, verbose=False):
        rng = np.random.default_rng(seed)
        X = np.asarray(X, dtype=float)
        Y = np.asarray(Y, dtype=float)
        n = X.shape[0]
        idx = rng.permutation(n)
        n_val = max(1, int(n * val_split))
        va, tr = idx[:n_val], idx[n_val:]
        Xtr, Ytr = X[tr], Y[tr]
        self.net.mu = Xtr.mean(axis=0)
        self.net.sd = Xtr.std(axis=0) + 1e-6
        Xtr_n, Xva_n = self.net._norm(Xtr), self.net._norm(X[va])

        Qtr = np.stack([self._heat_target(u, v) for u, v in Ytr[:, :2]])
        Qva = np.stack([self._heat_target(u, v) for u, v in Y[va, :2]])

        self.net.lr = lr
        nh = P.DS * P.DS
        m = Xtr_n.shape[0]
        self.history = {'loss': [], 'val_loss': []}
        for ep in range(epochs):
            order = rng.permutation(m)
            for s in range(0, m, batch):
                bi = order[s:s + batch]
                xb, yb, qb = Xtr_n[bi], Ytr[bi], Qtr[bi]
                acts, zs = self.net._forward(xb)
                out = acts[-1]
                # 热图头：softmax 交叉熵
                z = out[:, :nh] - out[:, :nh].max(axis=1, keepdims=True)
                p = np.exp(z)
                p /= p.sum(axis=1, keepdims=True)
                grad = np.zeros_like(out)
                grad[:, :nh] = (p - qb) / out.shape[0]
                # 可见性头：BCE
                vis = yb[:, 3]
                pv = 1.0 / (1.0 + np.exp(-out[:, nh]))
                grad[:, nh] = (pv - vis) / out.shape[0]
                gw, gb = self.net._backward(acts, zs, grad)
                self.net._adam(gw, gb)
            self.history['loss'].append(float(self._loss(Xtr_n, Ytr, Qtr)))
            self.history['val_loss'].append(float(self._loss(Xva_n, Y[va], Qva)))
            if verbose and (ep % 10 == 0 or ep == epochs - 1):
                print('  epoch %3d  loss %.4f  val %.4f'
                      % (ep, self.history['loss'][-1],
                         self.history['val_loss'][-1]))
        return self.history

    def _loss(self, X, Y, Q):
        out = self.net._forward(X)[0][-1]
        nh = P.DS * P.DS
        z = out[:, :nh] - out[:, :nh].max(axis=1, keepdims=True)
        p = np.exp(z)
        p /= p.sum(axis=1, keepdims=True)
        ce = float(-np.mean(np.sum(Q * np.log(p + 1e-9), axis=1)))
        vis = Y[:, 3]
        pv = np.clip(1.0 / (1.0 + np.exp(-out[:, nh])), 1e-6, 1 - 1e-6)
        bce = float(-np.mean(vis * np.log(pv) + (1 - vis) * np.log(1 - pv)))
        return ce + bce

    # ---------- 推理 ----------
    def detect(self, img):
        out = self.net.predict(self.features(img))[0]
        nh = P.DS * P.DS
        z = out[:nh] - out[:nh].max()
        p = np.exp(z)
        p /= p.sum()
        idx = np.arange(P.DS)
        gx, gy = np.meshgrid(idx, idx)
        u = float((p * (gx.reshape(-1) + 0.5)).sum() / P.DS * P.IMG_W)
        v = float((p * (gy.reshape(-1) + 0.5)).sum() / P.DS * P.IMG_H)
        vis = float(1.0 / (1.0 + np.exp(-out[nh])))
        return {'u': u, 'v': v, 'vis': vis}

    # ---------- 存取 ----------
    def save(self, path):
        self.net.save(path)

    @classmethod
    def load(cls, path):
        obj = cls()
        obj.net = MLP.load(path)
        return obj
