#!/usr/bin/env python3
"""
任务 4 —— 端到端视觉伺服网络 (纯 NumPy 实现)
=============================================
输入: 前视相机 RGB 图像 -> 灰度 84x84 -> 归一化
输出: 归一化控制量 [vx, vy, wz]
      vx = nominal_speed * (out0 + 1) / 2
      vy = 0.35 * out1
      wz = 0.60 * out2

三种网络架构 (均支持前向推理与反向传播训练):
  - nature_cnn : 16@5x5/s2 -> 32@3x3/s2 -> 2x2 avgpool -> FC128 -> FC3
  - small_cnn  :  8@5x5/s2 -> 16@3x3/s2 -> 2x2 avgpool -> FC64  -> FC3
  - mlp        : flatten(84x84) -> FC128 -> FC3

训练脚本: train/train_task4.py
"""

import math
import os
import time

import numpy as np

IMG_SIZE = 84
ACTION_DIM = 3
NOMINAL_SPEED = 0.45
VY_SCALE = 0.35
WZ_SCALE = 0.60

ARCHITECTURES = {
    'small_cnn':  {'channels': (8, 16), 'fc': 64},
    'nature_cnn': {'channels': (16, 32), 'fc': 128},
    'mlp':        None,
}

PIPE_Y = 3.2          # 管线横向中心线
PIPE_Z = -1.45        # 巡检高度


def expert_label_ref(y, yaw):
    """几何专家标签：横向纠偏 + 航向回正 + 跟随巡航。

    返回归一化动作 [out0, out1, out2]，与 VisuomotorController 的输出映射一致。
    """
    dy = PIPE_Y - y
    vx = NOMINAL_SPEED * (1.0 - 0.55 * min(1.0, abs(dy) / 0.6))
    vx *= (1.0 - 0.30 * min(1.0, abs(yaw) / 0.28))
    vx = float(np.clip(vx, 0.0, NOMINAL_SPEED))
    vy = float(np.clip(1.2 * dy, -VY_SCALE, VY_SCALE))
    wz = float(np.clip(-1.6 * yaw, -WZ_SCALE, WZ_SCALE))
    return np.array([
        2.0 * vx / NOMINAL_SPEED - 1.0,
        vy / VY_SCALE,
        wz / WZ_SCALE,
    ], dtype=np.float64)


# ──────────────────────────────────────────────────────────────
# 图像预处理
# ──────────────────────────────────────────────────────────────

def rgb_to_gray_resize(image, size=IMG_SIZE):
    """RGB/RGBA -> 灰度 + 最近邻缩放到 size x size"""
    img = np.asarray(image)
    if img.ndim == 2:
        gray = img.astype(np.float32)
    else:
        rgb = img[..., :3].astype(np.float32)
        gray = 0.299 * rgb[..., 0] + 0.587 * rgb[..., 1] + 0.114 * rgb[..., 2]
    h, w = gray.shape
    if h != size or w != size:
        yi = np.linspace(0, h - 1, size).astype(np.int32)
        xi = np.linspace(0, w - 1, size).astype(np.int32)
        gray = gray[yi][:, xi]
    return gray.astype(np.float32)


# ──────────────────────────────────────────────────────────────
# 基础层
# ──────────────────────────────────────────────────────────────

def _im2col(x, kh, kw, stride):
    """(N,C,H,W) -> (N*OH*OW, C*kh*kw)"""
    n, c, h, w = x.shape
    oh = (h - kh) // stride + 1
    ow = (w - kw) // stride + 1
    cols = np.empty((n, oh, ow, c, kh, kw), dtype=x.dtype)
    for i in range(kh):
        for j in range(kw):
            cols[:, :, :, :, i, j] = x[:, :, i:i + oh * stride:stride,
                                       j:j + ow * stride:stride].transpose(0, 2, 3, 1)
    return cols.reshape(n * oh * ow, c * kh * kw), oh, ow


def _col2im(dcols, x_shape, kh, kw, stride, oh, ow):
    n, c, h, w = x_shape
    d = dcols.reshape(n, oh, ow, c, kh, kw)
    dx = np.zeros(x_shape, dtype=dcols.dtype)
    for i in range(kh):
        for j in range(kw):
            dx[:, :, i:i + oh * stride:stride, j:j + ow * stride:stride] += \
                d[:, :, :, :, i, j].transpose(0, 3, 1, 2)
    return dx


def _avgpool2(x, k=2, stride=2):
    n, c, h, w = x.shape
    oh = (h - k) // stride + 1
    ow = (w - k) // stride + 1
    out = np.zeros((n, c, oh, ow), dtype=x.dtype)
    for i in range(k):
        for j in range(k):
            out += x[:, :, i:i + oh * stride:stride, j:j + ow * stride:stride]
    return out / (k * k), (n, c, h, w), oh, ow


def _avgpool2_back(dout, x_shape, k=2, stride=2):
    n, c, h, w = x_shape
    oh, ow = dout.shape[2], dout.shape[3]
    dx = np.zeros(x_shape, dtype=dout.dtype)
    share = dout / (k * k)
    for i in range(k):
        for j in range(k):
            dx[:, :, i:i + oh * stride:stride, j:j + ow * stride:stride] += share
    return dx


# ──────────────────────────────────────────────────────────────
# 卷积网络
# ──────────────────────────────────────────────────────────────

class NumpyConvNet:
    def __init__(self, architecture='small_cnn', seed=0, weights=None):
        if architecture not in ARCHITECTURES:
            raise ValueError(f"未知架构: {architecture}")
        self.architecture = architecture
        self.rng = np.random.default_rng(seed)
        self.arch_mlp = ARCHITECTURES[architecture] is None

        if self.arch_mlp:
            self.fc_sizes = [IMG_SIZE * IMG_SIZE, 128, ACTION_DIM]
            self.W = [self._he(n_in, n_out) for n_in, n_out in
                      zip(self.fc_sizes[:-1], self.fc_sizes[1:])]
            # 输出层小初始化：避免 tanh 一开始就饱和 (饱和区梯度 ~0)
            self.W[-1] = self.rng.normal(0.0, 0.01, self.W[-1].shape)
            self.b = [np.zeros(n, dtype=np.float64) for n in self.fc_sizes[1:]]
        else:
            cfg = ARCHITECTURES[architecture]
            self.channels = cfg['channels']
            self.fc_hidden = cfg['fc']
            c1, c2 = self.channels
            # conv1 5x5 s2: 84 -> 40; conv2 3x3 s2: 40 -> 19; pool2: 19 -> 9
            self.flat_dim = c2 * 9 * 9
            self.c1w = self._he(1 * 5 * 5, c1).reshape(1 * 5 * 5, c1)
            self.c1b = np.zeros(c1)
            self.c2w = self._he(c1 * 3 * 3, c2)
            self.c2b = np.zeros(c2)
            self.f1w = self._he(self.flat_dim, self.fc_hidden)
            self.f1b = np.zeros(self.fc_hidden)
            # 输出层小初始化：初始 tanh 接近线性区，保证反向梯度充足
            self.f2w = self.rng.normal(0.0, 0.01, (self.fc_hidden, ACTION_DIM))
            self.f2b = np.zeros(ACTION_DIM)

        if weights is not None:
            self.load_arrays(weights)

    def _he(self, n_in, n_out):
        return self.rng.normal(0.0, math.sqrt(2.0 / n_in), (n_in, n_out))

    # ── 前向 ──────────────────────────────────────────────
    def forward(self, x):
        """x: (N, 84, 84) 或 (N, 1, 84, 84)，返回 (N,3)"""
        x = np.asarray(x, dtype=np.float64)
        if x.ndim == 3:
            x = x[:, None, :, :]
        self._x_shape = x.shape

        if self.arch_mlp:
            flat = x.reshape(x.shape[0], -1)
            h1 = np.maximum(flat @ self.W[0] + self.b[0], 0.0)
            z2 = h1 @ self.W[1] + self.b[1]
            out = np.tanh(z2)
            self._cache = (flat, h1, z2, out)
            return out

        n = x.shape[0]
        cols1, oh1, ow1 = _im2col(x, 5, 5, 2)
        z1 = cols1 @ self.c1w + self.c1b
        mask1 = z1 > 0.0
        a1 = np.maximum(z1, 0.0).reshape(n, oh1, ow1, -1).transpose(0, 3, 1, 2)
        self._c1 = (cols1, oh1, ow1, mask1)

        cols2, oh2, ow2 = _im2col(a1, 3, 3, 2)
        z2c = cols2 @ self.c2w + self.c2b
        mask2 = (z2c > 0.0).reshape(n, oh2, ow2, -1).transpose(0, 3, 1, 2)
        a2 = np.maximum(z2c, 0.0).reshape(n, oh2, ow2, -1).transpose(0, 3, 1, 2)
        self._c2 = (cols2, oh2, ow2, a1.shape, mask2)

        pool, a2_shape, ohp, owp = _avgpool2(a2, 2, 2)
        self._pool = (a2_shape, ohp, owp)
        flat = pool.reshape(n, -1)
        h1 = np.maximum(flat @ self.f1w + self.f1b, 0.0)
        z3 = h1 @ self.f2w + self.f2b
        out = np.tanh(z3)
        self._cache = (flat, h1, z3, out)
        return out

    # ── 反向 ──────────────────────────────────────────────
    def backward(self, y_true):
        y_true = np.asarray(y_true, dtype=np.float64)
        flat, h1, z3, out = self._cache
        n = out.shape[0]
        d = 2.0 * (out - y_true) / (n * ACTION_DIM) * (1.0 - out * out)

        grads = {}
        if self.arch_mlp:
            grads['f2w'] = h1.T @ d
            grads['f2b'] = d.sum(axis=0)
            dh1 = (d @ self.W[1].T) * (h1 > 0.0)
            grads['f1w'] = flat.T @ dh1
            grads['f1b'] = dh1.sum(axis=0)
            return grads, None

        grads['f2w'] = h1.T @ d
        grads['f2b'] = d.sum(axis=0)
        dh1 = (d @ self.f2w.T) * (h1 > 0.0)
        grads['f1w'] = flat.T @ dh1
        grads['f1b'] = dh1.sum(axis=0)
        dflat = dh1 @ self.f1w.T

        a2_shape, ohp, owp = self._pool
        dpool = dflat.reshape(n, self.channels[1], ohp, owp)
        da2 = _avgpool2_back(dpool, a2_shape, 2, 2)

        cols2, oh2, ow2, a1_shape, mask2 = self._c2
        dz2 = da2.transpose(0, 2, 3, 1).reshape(-1, self.channels[1])
        dz2 *= mask2.transpose(0, 2, 3, 1).reshape(-1, self.channels[1])
        grads['c2w'] = cols2.T @ dz2
        grads['c2b'] = dz2.sum(axis=0)
        dcols2 = dz2 @ self.c2w.T
        da1 = _col2im(dcols2, a1_shape, 3, 3, 2, oh2, ow2)

        cols1, oh1, ow1, mask1 = self._c1
        dz1 = da1.transpose(0, 2, 3, 1).reshape(-1, self.channels[0])
        dz1 *= mask1
        grads['c1w'] = cols1.T @ dz1
        grads['c1b'] = dz1.sum(axis=0)
        dcols1 = dz1 @ self.c1w.T
        dx = _col2im(dcols1, self._x_shape, 5, 5, 2, oh1, ow1)
        return grads, dx

    def params(self):
        if self.arch_mlp:
            # 顺序必须与 param_grads 一致: [W0, b0, W1, b1]
            return [self.W[0], self.b[0], self.W[1], self.b[1]]
        return [self.c1w, self.c1b, self.c2w, self.c2b,
                self.f1w, self.f1b, self.f2w, self.f2b]

    def param_grads(self, grads, param_list=None):
        if self.arch_mlp:
            return [grads['f1w'], grads['f1b'], grads['f2w'], grads['f2b']]
        return [grads['c1w'], grads['c1b'], grads['c2w'], grads['c2b'],
                grads['f1w'], grads['f1b'], grads['f2w'], grads['f2b']]

    def n_params(self):
        return int(sum(p.size for p in self.params()))

    # ── 存取 ──────────────────────────────────────────────
    def state_dict(self):
        if self.arch_mlp:
            return {'w1': self.W[0], 'b1': self.b[0],
                    'w2': self.W[1], 'b2': self.b[1]}
        return {'c1w': self.c1w, 'c1b': self.c1b,
                'c2w': self.c2w, 'c2b': self.c2b,
                'f1w': self.f1w, 'f1b': self.f1b,
                'f2w': self.f2w, 'f2b': self.f2b}

    def load_arrays(self, weights):
        if self.arch_mlp:
            self.W = [np.asarray(weights['w1'], dtype=np.float64),
                      np.asarray(weights['w2'], dtype=np.float64)]
            self.b = [np.asarray(weights['b1'], dtype=np.float64),
                      np.asarray(weights['b2'], dtype=np.float64)]
        else:
            self.c1w = np.asarray(weights['c1w'], dtype=np.float64)
            self.c1b = np.asarray(weights['c1b'], dtype=np.float64)
            self.c2w = np.asarray(weights['c2w'], dtype=np.float64)
            self.c2b = np.asarray(weights['c2b'], dtype=np.float64)
            self.f1w = np.asarray(weights['f1w'], dtype=np.float64)
            self.f1b = np.asarray(weights['f1b'], dtype=np.float64)
            self.f2w = np.asarray(weights['f2w'], dtype=np.float64)
            self.f2b = np.asarray(weights['f2b'], dtype=np.float64)


# ──────────────────────────────────────────────────────────────
# 训练
# ──────────────────────────────────────────────────────────────

def train_network(X, Y, architecture='small_cnn', epochs=25, batch_size=32,
                  lr=1e-3, val_split=0.2, seed=0, verbose=True):
    from rov_mujoco.nn_core import Adam
    X = np.asarray(X, dtype=np.float64)
    Y = np.asarray(Y, dtype=np.float64)
    rng = np.random.default_rng(seed)
    idx = rng.permutation(len(X))
    n_val = max(1, int(len(X) * val_split))
    val_idx, tr_idx = idx[:n_val], idx[n_val:]
    Xtr, Ytr = X[tr_idx], Y[tr_idx]
    Xva, Yva = X[val_idx], Y[val_idx]

    net = NumpyConvNet(architecture, seed=seed)
    opt = Adam(lr=lr)
    history = {'train': [], 'val': []}

    for epoch in range(epochs):
        perm = rng.permutation(len(Xtr))
        losses = []
        for start in range(0, len(Xtr), batch_size):
            b = perm[start:start + batch_size]
            pred = net.forward(Xtr[b])
            losses.append(float(np.mean((pred - Ytr[b]) ** 2)))
            grads, _ = net.backward(Ytr[b])
            opt.step(net.params(), net.param_grads(grads, net.params()))
        tr_loss = float(np.mean(losses))
        va_loss = float(np.mean((net.forward(Xva) - Yva) ** 2))
        history['train'].append(tr_loss)
        history['val'].append(va_loss)
        if verbose and (epoch % 5 == 0 or epoch == epochs - 1):
            print(f"    epoch {epoch + 1:3d}/{epochs} | train MSE={tr_loss:.5f} | "
                  f"val MSE={va_loss:.5f}")
    return net, history, (Xva, Yva)


# ──────────────────────────────────────────────────────────────
# 运行时控制器
# ──────────────────────────────────────────────────────────────

class VisuomotorController:
    """纯图像输入的端到端视觉伺服控制器"""

    def __init__(self, architecture='small_cnn', algorithm='bc',
                 weights_path=None, nominal_speed=NOMINAL_SPEED,
                 zero_weights=False, img_mean=0.0, img_std=1.0):
        self.architecture = architecture
        self.algorithm = algorithm
        self.nominal_speed = nominal_speed
        self.device = "CPU (NumPy)"
        self.weights_loaded = False
        self.weights_source = None
        self.zero_weights = zero_weights
        self.img_mean = float(img_mean)
        self.img_std = float(img_std) if img_std else 1.0

        self.net = NumpyConvNet(architecture, seed=0)
        if zero_weights:
            self._zero_params()
            self.weights_loaded = True
            self.weights_source = "zero_ablation"
            return

        candidates = []
        if weights_path:
            candidates.append(weights_path)
        cur = os.path.dirname(os.path.abspath(__file__))
        root = os.path.dirname(cur)
        for base in (cur, root):
            candidates.append(os.path.join(base, "models",
                                           f"task4_{architecture}_bc.npz"))
            candidates.append(os.path.join(base, "models",
                                           f"task4_{architecture}_{algorithm}.npz"))
        try:
            from ament_index_python.packages import get_package_share_directory
            share = get_package_share_directory('rov_mujoco')
            candidates.append(os.path.join(share, "models",
                                           f"task4_{architecture}_bc.npz"))
        except Exception:
            pass

        for path in candidates:
            if path and os.path.exists(path):
                try:
                    with np.load(path) as archive:
                        data = {k: archive[k] for k in archive.files}
                    self.net.load_arrays(data)
                    if 'img_mean' in data:
                        self.img_mean = float(data['img_mean'])
                        self.img_std = float(data['img_std'])
                    self.weights_loaded = True
                    self.weights_source = path
                    break
                except Exception as exc:
                    print(f"[WARN] 视觉策略权值加载失败 ({path}): {exc}")
                    break

        if not self.weights_loaded:
            print(f"[WARN] 未找到 task4_{architecture}_bc.npz，使用随机初始化权值")

    def _zero_params(self):
        for p in self.net.params():
            p[...] = 0.0

    def preprocess(self, image):
        gray = rgb_to_gray_resize(image, IMG_SIZE)
        return (gray - self.img_mean) / self.img_std

    def predict_action(self, image):
        t0 = time.perf_counter()
        x = self.preprocess(image)[None, :, :]
        out = self.net.forward(x)[0]
        latency = (time.perf_counter() - t0) * 1000.0
        return {
            "vx": float(np.clip(self.nominal_speed * (out[0] + 1.0) * 0.5,
                                0.0, self.nominal_speed)),
            "vy": float(np.clip(VY_SCALE * out[1], -VY_SCALE, VY_SCALE)),
            "wz": float(np.clip(WZ_SCALE * out[2], -WZ_SCALE, WZ_SCALE)),
            "action": out,
            "latency_ms": latency,
        }
