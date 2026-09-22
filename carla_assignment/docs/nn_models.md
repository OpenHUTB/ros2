# 神经网络库 `nn_models.py`

本包为满足老师"**感知、规划、控制、端到端算法需要为神经网络**"的硬性要求，提供了
一个统一的最小神经网络库 `nn_models.py`。它用**纯 numpy** 实现（不依赖 TensorFlow / GPU），
因此可在无 CARLA 的本机**离线训练与推理**，也便于在文档中逐行讲解前向传播与反向传播。

## 1. 结构

| 类 | 用途 | 输出 | 损失 |
|---|---|---|---|
| `MLPClassifier` | 感知（分类） | softmax 各类概率 | 交叉熵 |
| `MLPPolicy` | 控制 / 规划（回归） | 连续控制量 | MSE |

两个类共享 `_MLPBase` 的前向 / 反向 / 训练 / 存取逻辑。

## 2. 数学原理

### 2.1 全连接前向

$$
\mathbf h^{[l]} = \mathrm{ReLU}\big(W^{[l]}\mathbf a^{[l-1]} + \mathbf b^{[l]}\big)
$$

输出层：
- 分类：$\mathbf p = \mathrm{softmax}(W \mathbf a + \mathbf b)$
- 回归：$\delta = \tanh(W\mathbf a + \mathbf b)$

### 2.2 反向传播

对损失 $\mathcal L$ 用链式法则逐层求梯度：

$$
\delta^{[l]} = (W^{[l+1]})^\top \delta^{[l+1]} \odot \mathrm{ReLU}'(z^{[l]}),
\qquad
\frac{\partial \mathcal L}{\partial W^{[l]}} = \delta^{[l]} {\mathbf a^{[l-1]}}^\top
$$

参数更新（小批量随机梯度下降）：

$$
W^{[l]} \leftarrow W^{[l]} - \eta\,\frac1B\!\sum_{b=1}^{B}\frac{\partial \mathcal L}{\partial W^{[l]}}
$$

## 3. 模型存取

训练完成后把权重与偏置保存为 JSON（文本，可入库 / 跨机拷贝）：

- `02_perception` 模型文件 `models/nn_percept.json`：含 `{sens, ctrl}` 两个网络
- `03_navigation` 模型文件 `models/nn_plan.json`：规划网络

推理时 `load()` 加载权重即可在线前向，无需再训练。

## 4. 自测

```bash
python nn_models.py        # 运行内置自测（分类 acc≈0.99，回归 MSE≈0.005）
```

自测用随机可分类/可回归数据验证前向与反向一致、能收敛，保证库可离线使用。

## 5. 与端到端 CNN 的关系

`04_end_to_end` 的 CNN 用 `tf.keras`（图像→steer），与这里的 numpy MLP 属于两种实现，
但都满足"神经网络"要求：本库 MLP 用于低维特征（感知/控制/规划），CNN 用于图像端到端。
