"""纯 NumPy 的小型卷积网络：2 层 3x3 卷积 + 2 层全连接，手写反向传播。

不依赖 PyTorch/TF，整包只需 numpy 就能训练与推理 —— 这是为了让
"端到端网络"在任意一台只装了 ROS 2 的机器上都能跑通。
"""

import numpy as np

IN_CH = 5          # 3 帧灰度堆叠 + 1 帧归一化深度 + 1 帧目标方位
CONV1 = 8
CONV2 = 16
FC1 = 64
OUT = 2            # (v_norm, w_norm)


def im2col(x, k, stride, pad):
    B, C, H, W = x.shape
    Ho = (H + 2 * pad - k) // stride + 1
    Wo = (W + 2 * pad - k) // stride + 1
    xp = np.pad(x, ((0, 0), (0, 0), (pad, pad), (pad, pad)))
    cols = np.empty((B, C, k, k, Ho, Wo), dtype=x.dtype)
    for i in range(k):
        for j in range(k):
            cols[:, :, i, j, :, :] = xp[:, :, i:i + stride * Ho:stride,
                                        j:j + stride * Wo:stride]
    return cols.reshape(B, C * k * k, Ho * Wo), (Ho, Wo), xp.shape


def col2im(dcols, xp_shape, k, stride, pad):
    """xp_shape 是 padding 之后的形状，输出尺寸由它直接决定。"""
    B, C, Hp, Wp = xp_shape
    Ho = (Hp - k) // stride + 1
    Wo = (Wp - k) // stride + 1
    d = dcols.reshape(B, C, k, k, Ho, Wo)
    dxp = np.zeros((B, C, Hp, Wp), dtype=dcols.dtype)
    for i in range(k):
        for j in range(k):
            dxp[:, :, i:i + stride * Ho:stride, j:j + stride * Wo:stride] += d[:, :, i, j]
    if pad:
        return dxp[:, :, pad:-pad, pad:-pad]
    return dxp


def _out_size(n, k, s, p):
    return (n + 2 * p - k) // s + 1


class Conv:
    def __init__(self, cin, cout, k, stride, pad, rng):
        self.k, self.stride, self.pad = k, stride, pad
        scale = np.sqrt(2.0 / (cin * k * k))
        self.W = rng.normal(0.0, scale, (cout, cin * k * k))
        self.b = np.zeros(cout)
        self.gW = np.zeros_like(self.W)
        self.gb = np.zeros_like(self.b)
        self._c = None

    def forward(self, x, train=False):
        cols, (Ho, Wo), shape = im2col(x, self.k, self.stride, self.pad)
        out = np.matmul(self.W, cols) + self.b[None, :, None]
        if train:
            self._c = (cols, shape, x.shape)
        return out.reshape(x.shape[0], -1, Ho, Wo)

    def backward(self, dout):
        cols, shape, xshape = self._c
        B = xshape[0]
        Ho, Wo = dout.shape[2], dout.shape[3]
        d = dout.reshape(B, dout.shape[1], Ho * Wo)
        self.gW = np.einsum('bok,bck->oc', d, cols) / B
        self.gb = d.sum(axis=(0, 2)) / B
        dcols = np.matmul(self.W.T, d)
        dx = col2im(dcols, shape, self.k, self.stride, self.pad)
        self._c = None
        return dx


class Dense:
    def __init__(self, nin, nout, rng, last=False):
        scale = np.sqrt(1.0 / nin) if last else np.sqrt(2.0 / nin)
        self.W = rng.normal(0.0, scale, (nin, nout))
        self.b = np.zeros(nout)
        self.gW = np.zeros_like(self.W)
        self.gb = np.zeros_like(self.b)
        self._x = None

    def forward(self, x, train=False):
        if train:
            self._x = x
        return x @ self.W + self.b

    def backward(self, dout):
        self.gW = self._x.T @ dout / dout.shape[0]
        self.gb = dout.mean(axis=0)
        dx = dout @ self.W.T
        self._x = None
        return dx


class CNN:
    """固定结构，便于权重用 npz 直接存取。"""

    def __init__(self, in_h, in_w, rng=None):
        rng = np.random.default_rng(0) if rng is None else rng
        self.in_h, self.in_w = in_h, in_w
        # 第一层 stride=4 直接做 4 倍下采样：48x64 -> 12x16，CPU 上训练快一个量级
        self.c1 = Conv(IN_CH, CONV1, 5, 4, 2, rng)
        self.c2 = Conv(CONV1, CONV2, 3, 2, 1, rng)
        h1, w1 = _out_size(in_h, 5, 4, 2), _out_size(in_w, 5, 4, 2)
        h2, w2 = _out_size(h1, 3, 2, 1), _out_size(w1, 3, 2, 1)
        self.flat = CONV2 * h2 * w2
        self.f1 = Dense(self.flat, FC1, rng)
        self.f2 = Dense(FC1, OUT, rng, last=True)

    def forward(self, x, train=False):
        a1 = self.c1.forward(x, train)
        self._c1_out = a1
        h = np.maximum(a1, 0.0)
        a2 = self.c2.forward(h, train)
        self._c2_out = a2
        h = np.maximum(a2, 0.0)
        self._shape = h.shape
        h = h.reshape(h.shape[0], -1)
        h = self.f1.forward(h, train)
        h = np.maximum(h, 0.0)
        self._h = h
        z = self.f2.forward(h, train)
        return np.tanh(z)

    def backward(self, dy):
        dz = dy * (1.0 - self._pred ** 2)
        dh = self.f2.backward(dz)
        dh = dh * (self._h > 0)
        dh = self.f1.backward(dh)
        dh = dh.reshape(self._shape) * (self._c2_out > 0)
        dh = self.c2.backward(dh)
        dh = dh * (self._c1_out > 0)
        self.c1.backward(dh)

    def params(self):
        return [self.c1, self.c2, self.f1, self.f2]

    def predict(self, x):
        out = self.forward(x, train=False)
        self._pred = out
        return out

    def train_step(self, x, y, opt, lw=1.0, sw=None):
        """sw: 每个样本的额外权重 (len(batch),)。

        转向帧（|w| 大）在专家数据里只占少数，普通 MSE 会被大量
        "直行帧"平均掉，学出来的策略在弯道前转向严重不足 —— 这正是
        行为克隆回归均值化的经典症状。给 |w| 大的样本更大权重，
        相当于过采样少数派。
        """
        out = self.forward(x, train=True)
        self._pred = out
        diff = out - y
        w = np.array([1.0, lw], dtype=np.float64)
        if sw is None:
            sw = np.ones(len(x))
        sw = np.asarray(sw, dtype=np.float64)[:, None]
        loss = float(np.mean((diff ** 2) * w * sw))
        # 注意：各层 backward 内部已经按 batch 求了平均，这里不能再除 batch
        dy = 2.0 * diff * w * sw
        self.backward(dy)
        opt.step()
        return loss

    # ------------------------------------------------------------ 存取
    def state(self):
        return {'c1W': self.c1.W, 'c1b': self.c1.b,
                'c2W': self.c2.W, 'c2b': self.c2.b,
                'f1W': self.f1.W, 'f1b': self.f1.b,
                'f2W': self.f2.W, 'f2b': self.f2.b,
                'shape': np.array([self.in_h, self.in_w])}

    def load_state(self, d):
        self.c1.W, self.c1.b = d['c1W'], d['c1b']
        self.c2.W, self.c2.b = d['c2W'], d['c2b']
        self.f1.W, self.f1.b = d['f1W'], d['f1b']
        self.f2.W, self.f2.b = d['f2W'], d['f2b']


class Adam:
    def __init__(self, params, lr=2e-3, b1=0.9, b2=0.999, eps=1e-8):
        self.p = params
        self.lr, self.b1, self.b2, self.eps = lr, b1, b2, eps
        self.m = [np.zeros_like(x.W) for x in params]
        self.v = [np.zeros_like(x.W) for x in params]
        self.mb = [np.zeros_like(x.b) for x in params]
        self.vb = [np.zeros_like(x.b) for x in params]
        self.t = 0

    def step(self):
        self.t += 1
        c1 = 1.0 - self.b1 ** self.t
        c2 = 1.0 - self.b2 ** self.t
        for i, layer in enumerate(self.p):
            self.m[i] = self.b1 * self.m[i] + (1 - self.b1) * layer.gW
            self.v[i] = self.b2 * self.v[i] + (1 - self.b2) * layer.gW ** 2
            mhat = self.m[i] / c1
            vhat = self.v[i] / c2
            layer.W -= self.lr * mhat / (np.sqrt(vhat) + self.eps)

            self.mb[i] = self.b1 * self.mb[i] + (1 - self.b1) * layer.gb
            self.vb[i] = self.b2 * self.vb[i] + (1 - self.b2) * layer.gb ** 2
            mbhat = self.mb[i] / c1
            vbhat = self.vb[i] / c2
            layer.b -= self.lr * mbhat / (np.sqrt(vbhat) + self.eps)
