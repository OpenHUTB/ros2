"""纯 numpy 实现的多层感知机（MLP），无需 PyTorch/TensorFlow。

网络：输入 N 束归一化射线 → tanh 隐层 → 线性输出 [steer, throttle]。
输出在 predict 阶段被约束到合理范围（steer∈[-1,1]，throttle∈[0,1]）。
"""
import numpy as np


def _he(shape, rng):
    fan_in = shape[1]
    return rng.normal(0.0, np.sqrt(2.0 / fan_in), size=shape)


class MLP:
    def __init__(self, input_dim, hidden, output_dim, seed=42):
        rng = np.random.default_rng(seed)
        self.input_dim = input_dim
        self.hidden = hidden
        self.output_dim = output_dim
        self.W1 = _he((hidden, input_dim), rng)
        self.b1 = np.zeros(hidden, dtype=np.float64)
        self.W2 = _he((output_dim, hidden), rng)
        self.b2 = np.zeros(output_dim, dtype=np.float64)

    def forward(self, X):
        """X: (n, input_dim)；返回 (n, output_dim) 线性输出。"""
        self.z1 = X @ self.W1.T + self.b1
        self.a1 = np.tanh(self.z1)
        out = self.a1 @ self.W2.T + self.b2
        return out

    def predict(self, X):
        """前向推理并约束输出范围。X: (n, input_dim) 或 (input_dim,)。"""
        single = X.ndim == 1
        Xb = X[None, :] if single else X
        out = self.forward(Xb)
        steer = np.tanh(out[:, 0])
        throttle = np.clip(out[:, 1], 0.0, 1.0)
        res = np.stack([steer, throttle], axis=1)
        return res[0] if single else res

    def train_step(self, X, Y, lr):
        """一个 minibatch 的梯度下降；返回该 batch 的均方误差。"""
        n = X.shape[0]
        out = self.forward(X)
        d_out = 2.0 * (out - Y) / n
        # 输出层
        grad_W2 = d_out.T @ self.a1
        grad_b2 = d_out.sum(axis=0)
        # 隐层（tanh' = 1 - tanh^2）
        d_a1 = d_out @ self.W2
        d_z1 = d_a1 * (1.0 - self.a1 ** 2)
        grad_W1 = d_z1.T @ X
        grad_b1 = d_z1.sum(axis=0)
        # 更新
        self.W1 -= lr * grad_W1
        self.b1 -= lr * grad_b1
        self.W2 -= lr * grad_W2
        self.b2 -= lr * grad_b2
        return float(np.mean((out - Y) ** 2))

    def fit(self, X, Y, epochs=300, batch=64, lr=0.005, seed=42):
        """全量训练：minibatch 梯度下降，返回每个 epoch 的平均损失列表。"""
        rng = np.random.default_rng(seed)
        n = X.shape[0]
        losses = []
        for ep in range(epochs):
            perm = rng.permutation(n)
            ep_losses = []
            for s in range(0, n, batch):
                idx = perm[s:s + batch]
                ep_losses.append(self.train_step(X[idx], Y[idx], lr))
            losses.append(float(np.mean(ep_losses)))
        return losses

    def save(self, path):
        np.savez(path, W1=self.W1, b1=self.b1, W2=self.W2, b2=self.b2)

    @classmethod
    def load(cls, path):
        z = np.load(path)
        m = cls.__new__(cls)
        m.W1, m.b1 = z["W1"], z["b1"]
        m.W2, m.b2 = z["W2"], z["b2"]
        m.input_dim = m.W1.shape[1]
        m.hidden = m.W1.shape[0]
        m.output_dim = m.W2.shape[0]
        return m
