"""车道线位置回归网络：两层全连接，手写反向传播。

输入是裁掉天空和车头后的路面条带（32x128），输出左右两条车道边界在 10 m 处的横向位置（米）。
参数量很小，纯 NumPy 实现，CPU 上几十秒就能训完一轮。
"""
import numpy as np

INPUT_H = 32
INPUT_W = 128
INPUT_DIM = INPUT_H * INPUT_W


def he_normal(rng, fan_in, fan_out):
    """He 正态初始化，配合 ReLU。"""
    return rng.normal(0.0, np.sqrt(2.0 / fan_in), (fan_in, fan_out))


class LaneRegressor(object):
    """4096 -> 64 -> 32 -> 2，输入是裁过的路面条带。"""

    def __init__(self, hidden=(64, 32), n_out=2, seed=0):
        rng = np.random.default_rng(seed)
        sizes = (INPUT_DIM,) + tuple(hidden) + (n_out,)
        self.weights = [he_normal(rng, sizes[i], sizes[i + 1])
                        for i in range(len(sizes) - 1)]
        self.biases = [np.zeros(sizes[i + 1]) for i in range(len(sizes) - 1)]

    # ---------- 前向与反向 ----------

    def forward(self, x):
        """返回 (预测值, 缓存)，缓存给反向传播用。"""
        activations = [x]
        pre = []
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases)):
            z = activations[-1] @ weight + bias
            pre.append(z)
            last = index == len(self.weights) - 1
            activations.append(z if last else np.maximum(z, 0.0))
        return activations[-1], (activations, pre)

    def backward(self, cache, grad_out):
        """链式求导，返回各层 (dW, db)。"""
        activations, pre = cache
        grads_w = [None] * len(self.weights)
        grads_b = [None] * len(self.weights)
        delta = grad_out
        for index in range(len(self.weights) - 1, -1, -1):
            grads_w[index] = activations[index].T @ delta
            grads_b[index] = delta.sum(axis=0)
            if index:
                delta = (delta @ self.weights[index].T) * (pre[index - 1] > 0)
        return grads_w, grads_b

    def predict(self, x):
        return self.forward(x)[0]

    # ---------- 训练 ----------

    def fit(self, images, targets, epochs=30, batch_size=256, lr=0.01,
            momentum=0.9, seed=0, verbose=True):
        """小批量梯度下降，返回每轮的训练损失。"""
        rng = np.random.default_rng(seed)
        x = flatten_images(images)
        y = np.asarray(targets, dtype=np.float64)
        n = x.shape[0]
        velocity_w = [np.zeros_like(w) for w in self.weights]
        velocity_b = [np.zeros_like(b) for b in self.biases]
        history = []

        for epoch in range(epochs):
            order = rng.permutation(n)
            total = 0.0
            for start in range(0, n, batch_size):
                idx = order[start:start + batch_size]
                pred, cache = self.forward(x[idx])
                diff = pred - y[idx]
                total += float(np.mean(diff ** 2)) * len(idx)
                grad = 2.0 * diff / diff.size
                grads_w, grads_b = self.backward(cache, grad)
                for i in range(len(self.weights)):
                    velocity_w[i] = momentum * velocity_w[i] - lr * grads_w[i]
                    velocity_b[i] = momentum * velocity_b[i] - lr * grads_b[i]
                    self.weights[i] += velocity_w[i]
                    self.biases[i] += velocity_b[i]
            history.append(total / n)
            if verbose:
                print('  第 %2d 轮  训练损失 %.5f' % (epoch + 1, history[-1]))
        return history

    # ---------- 存取 ----------

    def save(self, path):
        payload = {}
        for index, (weight, bias) in enumerate(zip(self.weights, self.biases)):
            # 存 float32：精度对推理没影响，体积小一半
            payload['w%d' % index] = weight.astype(np.float32)
            payload['b%d' % index] = bias.astype(np.float32)
        np.savez_compressed(path, **payload)

    @classmethod
    def load(cls, path):
        with np.load(path) as data:
            keys = sorted(k for k in data.files if k.startswith('w'))
            hidden = [data[k].shape[1] for k in keys[:-1]]
            model = cls(hidden=tuple(hidden), n_out=data[keys[-1]].shape[1])
            for index, key in enumerate(keys):
                model.weights[index] = data[key].astype(np.float64)
                model.biases[index] = data['b%d' % index].astype(np.float64)
        return model


def flatten_images(images):
    """(N, H, W) 或 (N, H, W, 1) 的灰度图压成 (N, H*W) 的浮点特征。"""
    array = np.asarray(images, dtype=np.float64)
    if array.ndim == 4:
        array = array[:, :, :, 0]
    if array.ndim == 2:
        array = array[None]
    return array.reshape(array.shape[0], -1) / 255.0


def count_parameters(model):
    return int(sum(w.size for w in model.weights) + sum(b.size for b in model.biases))