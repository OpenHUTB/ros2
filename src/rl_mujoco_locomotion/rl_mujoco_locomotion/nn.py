"""NumPy-based neural network primitives for RL."""

import numpy as np


def clip_grad_norm(grad, max_norm):
    """Clip gradient by global L2 norm (NaN guard)."""
    norm = np.sqrt(np.sum(grad ** 2))
    if not np.isfinite(norm):
        return np.zeros_like(grad)
    if norm > max_norm:
        return grad * (max_norm / (norm + 1e-8))
    return grad


class MLP:
    """Multi-layer perceptron with ReLU hidden activations."""

    def __init__(self, layer_sizes, seed=0):
        self.rng = np.random.default_rng(seed)
        self.weights = []
        self.biases = []
        for i in range(len(layer_sizes) - 1):
            w = self.rng.standard_normal((layer_sizes[i], layer_sizes[i + 1])) * np.sqrt(2.0 / layer_sizes[i])
            b = np.zeros((1, layer_sizes[i + 1]))
            self.weights.append(w.astype(np.float64))
            self.biases.append(b.astype(np.float64))

    def forward(self, x):
        self._acts = [x]
        for i, (w, b) in enumerate(zip(self.weights, self.biases)):
            x = x @ w + b
            if i < len(self.weights) - 1:
                x = np.maximum(0, x)  # ReLU
            self._acts.append(x)
        return x

    def backward(self, grad, lr=None):
        """Backprop. If lr is None only return d(loss)/d(input), no update."""
        for i in reversed(range(len(self.weights))):
            if i < len(self.weights) - 1:
                grad = grad * (self._acts[i + 1] > 0)  # ReLU derivative
            dw = self._acts[i].T @ grad
            db = np.sum(grad, axis=0, keepdims=True)
            grad = grad @ self.weights[i].T
            if lr is not None:
                self.weights[i] -= lr * dw
                self.biases[i] -= lr * db
        return grad

    def get_params(self):
        return [w.copy() for w in self.weights], [b.copy() for b in self.biases]

    def params(self):
        """返回参数数组的**引用**（对返回值原地写入即可修改网络），用于软/硬更新。"""
        return self.weights, self.biases

    def set_params(self, weights, biases):
        self.weights = [w.copy() for w in weights]
        self.biases = [b.copy() for b in biases]

    def save(self, path):
        np.savez(path, *self.weights, *self.biases)

    def load(self, path):
        data = np.load(path)
        n = len(data) // 2
        self.weights = [data[f'arr_{i}'] for i in range(n)]
        self.biases = [data[f'arr_{n + i}'] for i in range(n)]


class Adam:
    """Adam optimizer for arbitrary parameter arrays."""

    def __init__(self, params, lr=3e-4, beta1=0.9, beta2=0.999, eps=1e-8):
        self.params = params
        self.lr = lr
        self.b1 = beta1
        self.b2 = beta2
        self.eps = eps
        self.m = [np.zeros_like(p) for p in params]
        self.v = [np.zeros_like(p) for p in params]
        self.t = 0

    def step(self, grads):
        self.t += 1
        for i, (p, g) in enumerate(zip(self.params, grads)):
            self.m[i] = self.b1 * self.m[i] + (1 - self.b1) * g
            self.v[i] = self.b2 * self.v[i] + (1 - self.b2) * g * g
            m_hat = self.m[i] / (1 - self.b1 ** self.t)
            v_hat = self.v[i] / (1 - self.b2 ** self.t)
            p -= self.lr * m_hat / (np.sqrt(v_hat) + self.eps)


def soft_update(target, source, tau=0.005):
    """Polyak averaging; accepts (weights, biases) tuples or flat param lists."""
    for tp, sp in zip(target, source):
        if isinstance(tp, list):
            for t, s in zip(tp, sp):
                t[:] = tau * s + (1 - tau) * t
        else:
            tp[:] = tau * sp + (1 - tau) * tp


def hard_update(target, source):
    for tp, sp in zip(target, source):
        if isinstance(tp, list):
            for t, s in zip(tp, sp):
                t[:] = s.copy()
        else:
            tp[:] = sp.copy()
