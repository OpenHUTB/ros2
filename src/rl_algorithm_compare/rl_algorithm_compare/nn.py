"""纯 NumPy 实现的全连接神经网络（MLP）与 Adam 优化器。

不依赖 PyTorch，仅用 NumPy 完成前向传播、反向传播与参数更新，
从而在缺少 torch 的离线环境下也能训练神经网络策略。
"""
import numpy as np


def relu(x):
    return np.maximum(0.0, x)


def relu_grad(a):
    return (a > 0).astype(np.float64)


def softmax(x):
    e = np.exp(x - x.max(axis=-1, keepdims=True))
    return e / e.sum(axis=-1, keepdims=True)


class MLP:
    """多层感知机。sizes=[输入,隐层...,输出]。

    输出层激活：'linear'（用于 Q 值）或 'softmax'（用于策略概率）。
    """

    def __init__(self, sizes, out='linear', lr=1e-3):
        self.sizes = list(sizes)
        self.out = out
        self.lr = lr
        rng = np.random.RandomState(0)
        # Xavier / He 初始化
        self.W = [rng.randn(sizes[i], sizes[i + 1]) *
                  np.sqrt(2.0 / sizes[i]) for i in range(len(sizes) - 1)]
        self.b = [np.zeros(sizes[i + 1]) for i in range(len(sizes) - 1)]
        self.mW = [np.zeros_like(w) for w in self.W]
        self.mb = [np.zeros_like(x) for x in self.b]
        self.vW = [np.zeros_like(w) for w in self.W]
        self.vb = [np.zeros_like(x) for x in self.b]
        self.t = 0

    def n_params(self):
        return int(sum(w.size for w in self.W) +
                   sum(x.size for x in self.b))

    def forward(self, x):
        """前向：x 形状 (n,in)，返回 (输出, 每层 pre-act 的 relu 结果列表)。"""
        a = np.atleast_2d(x).astype(np.float64)
        hs = []          # 保存每层 relu 后的激活（含输入层）
        zs = []          # 保存每层线性输出
        for i in range(len(self.W) - 1):
            z = a.dot(self.W[i]) + self.b[i]
            zs.append(z)
            a = relu(z)
            hs.append(a)
        z = a.dot(self.W[-1]) + self.b[-1]
        zs.append(z)
        out = softmax(z) if self.out == 'softmax' else z
        return out, (hs, zs)

    def backward(self, x, dy):
        """反向传播：x 形状 (n,in)，dy 为输出层梯度 (n,out)。
        返回梯度列表 (gW, gb)。"""
        out, (hs, zs) = self.forward(x)
        n = x.shape[0]
        hs = [np.atleast_2d(x).astype(np.float64)] + hs   # 含输入层
        # 输出层
        gz = dy
        gW = [hs[-1].T.dot(gz) / n]
        gb = [gz.mean(axis=0)]
        # 隐藏层（从倒数第二层到第一层）
        for i in range(len(self.W) - 2, -1, -1):
            gz = gz.dot(self.W[i + 1].T) * relu_grad(zs[i])
            gW.append(hs[i].T.dot(gz) / n)
            gb.append(gz.mean(axis=0))
        gW.reverse()
        gb.reverse()
        return gW, gb

    def adam_update(self, gW, gb, beta1=0.9, beta2=0.999, eps=1e-8):
        """用 Adam 更新参数。"""
        self.t += 1
        for i in range(len(self.W)):
            self.mW[i] = beta1 * self.mW[i] + (1 - beta1) * gW[i]
            self.vW[i] = beta2 * self.vW[i] + (1 - beta2) * (gW[i] ** 2)
            mhat = self.mW[i] / (1 - beta1 ** self.t)
            vhat = self.vW[i] / (1 - beta2 ** self.t)
            self.W[i] -= self.lr * mhat / (np.sqrt(vhat) + eps)

            self.mb[i] = beta1 * self.mb[i] + (1 - beta1) * gb[i]
            self.vb[i] = beta2 * self.vb[i] + (1 - beta2) * (gb[i] ** 2)
            mhat = self.mb[i] / (1 - beta1 ** self.t)
            vhat = self.vb[i] / (1 - beta2 ** self.t)
            self.b[i] -= self.lr * mhat / (np.sqrt(vhat) + eps)

    def predict(self, x):
        """返回网络输出（不梯度）。"""
        out, _ = self.forward(x)
        return out

    def save(self, path):
        import pickle
        with open(path, 'wb') as f:
            pickle.dump({'W': self.W, 'b': self.b, 'sizes': self.sizes,
                         'out': self.out, 'lr': self.lr}, f)

    def load(self, path):
        import pickle
        with open(path, 'rb') as f:
            d = pickle.load(f)
        self.W = d['W']
        self.b = d['b']
        self.sizes = d['sizes']
        self.out = d['out']
        self.lr = d['lr']
        return self
