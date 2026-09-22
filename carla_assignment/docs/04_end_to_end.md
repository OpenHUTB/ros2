# 作业四 · CARLA 端到端神经网络【图像 → 控制】

> 对应老师任务 (4)：端到端模型（输入图像、输出控制指令）。

## 1. 任务目标

用 CNN 实现端到端驾驶：**前视相机图像 → 神经网络 → 转向控制**。
三阶段：采集数据 → 训练 → 自主驾驶。

> **两种训练后端**（都可满足"端到端模型为神经网络"）：
> - `--backend tf`（默认）：`tf.keras` CNN，需 TensorFlow，模型存 `.h5`
> - `--backend numpy`：**纯 numpy SimpleCNN**（`nn_models.py`，无需 TensorFlow），模型存 `.json`，
>   可在本机离线训练/验证端到端图像→转向链路。

## 2. 数据流

```
CARLA RGB相机(60×80×3) → [CNN] → steer ∈ [-1,1] → (throttle=0.5, steer)
```

## 3. 计算原理（CNN）

### 3.1 卷积特征提取

输入图像归一化后，经多层卷积提取空间特征。第 $l$ 层输出：

$$
A^{[l]} = \text{ReLU}\big(W^{[l]} * A^{[l-1]} + b^{[l]}\big)
$$

卷积核滑动提取道路/边界/障碍特征，池化降采样增强平移不变性。

### 3.2 全连接输出转向

全局平均池化压成向量 $z$，经全连接与 $tanh$ 激活输出：

$$
\text{steer} = \tanh(W_o z + b_o) \in [-1,1]
$$

$tanh$ 保证输出严格落在 $[-1,1]$，直接映射为转向。

### 3.3 损失（有监督回归）

用平均绝对误差回归：

$$
\mathcal{L} = \frac{1}{N}\sum_{i=1}^{N} \big|\hat{y}_i - y_i\big|
$$

$y_i$ 为采集时真实施加的转向角，$\hat{y}_i$ 为网络预测。

## 4. 算法流程

```
collect: spawn自车 → 正弦转向生成多样轨迹 → 每帧存(相机图, steer) → dataset
train  : 读取dataset → CNN回归训练(loss=MAE) → 保存 .h5（tf）或 .json（numpy）
test   : 加载模型 → 相机图 → CNN → steer → apply_control 自主驾驶
```

## 5. 源码解析（`04_end_to_end/main.py` + `nn_models.py`）

- `build_cnn()`：`tf.keras` 3 层卷积 + 全局池化 + Dense(64)+Dropout + `tanh` 单输出。
- `SimpleCNN`（`nn_models.py`）：纯 numpy 实现的轻量 CNN，含卷积前向/反向与 2×2 池化。
- `collect()`：正弦控制 `steer = 0.5 sin(i/12)+0.2 cos(i/5)` 生成多样轨迹，保存图像+标签。
- `train()`：`cv2` 读图、按 `--backend` 选择 tf 或 numpy 网络训练、`save`。
- `test()`：加载模型预测 steer，`apply_control` 自主驾驶。
  ```python
  steer = float(net.predict(img_in, verbose=0)[0][0])   # tf
  steer = float(net.predict(img.astype(np.uint8))[0,0])  # numpy
  apply_control(vehicle, throttle=0.5, steer=steer)
  ```

## 6. 运行

```bash
source .venv/bin/activate
# 采集（需 CARLA）
python main.py --task end_to_end --mode collect --frames 200 --out_dir dataset

# 训练（无需 CARLA）
#   tf 后端（需 TensorFlow）
python main.py --task end_to_end --mode train --epochs 30 --data_dir dataset
#   numpy 后端（纯 numpy，本机即可）
python main.py --task end_to_end --mode train --backend numpy \
       --epochs 120 --data_dir dataset --model_path models/cnn.json

# 测试 / 自主驾驶（需 CARLA）
python main.py --task end_to_end --mode test --model_path models/cnn.h5
python main.py --task end_to_end --mode test --backend numpy --model_path models/cnn.json
```

ROS launch 见第 6 节（`04_end_to_end/launch/`，`mode:=test`）。

### 6.1 生成训练曲线（性能评价）

采集+训练完成后，可用自带脚本离线生成训练/验证 loss 曲线 PNG（供文档引用、无需 CARLA）：

```bash
python scripts/gen_train_loss.py --data_dir dataset --epochs 30
# 产出 docs/assets/train_loss.png
```

> 若尚无采集数据集，先 `python main.py --task end_to_end --mode collect ...`。

## 7. 录屏剧本

1. 采集一段（证明数据来源）、训练打印 loss 下降曲线。
2. 自主驾驶：车辆沿直线偏置校正方向行驶一段，录制 10-15 秒。

!!! note "运行动图（需在装有 CARLA 的机器上录制后放入）"
    录屏后替换此占位图为真实动图：`![](assets/placeholder_end_to_end.png)` → `![](assets/04_end_to_end.gif)`

## 8. 性能评价

（在装有 CARLA 的机器上采集 + `scripts/gen_train_loss.py` 生成曲线后填写）

![训练/验证 MAE 曲线](assets/placeholder_train_loss.png) <!-- 训练曲线生成后用 assets/train_loss.png 替换 -->

| 指标 | 数值 |
|---|---|
| 训练 MAE（最后） | （运行后填写） |
| 验证 MAE（最后） | （运行后填写） |
| 自主驾驶 steer 平滑度（AoS） | （运行后填写） |

- **训练 loss / MAE**：随 epoch 下降，越小预测越准。
- **自主驾驶 steer 平滑度**：预测转向变化不突变。
- **能否自主行驶一段不频繁撞障碍**。
