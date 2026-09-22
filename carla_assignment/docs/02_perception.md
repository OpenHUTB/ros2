# 作业二 · CARLA 传感器感知 + 给定轨迹跟踪（神经网络版）

> 对应老师任务 (2)：使用传感器获取的数据进行感知（雷达、摄像头、深度摄像头），
> 并对机器人进行运动控制（感知 + 给定轨迹的控制）。
>
> 同时满足老师"**感知、控制算法需要为神经网络**"的硬性要求：本模块的**感知器**与
> **控制器**均为神经网络（`nn_models.py` 中的 `MLPClassifier` / `MLPPolicy`）。

## 1. 任务目标

- **感知**：用 RGB 相机、深度相机、雷达 LiDAR 三路传感器提取特征，喂给**感知神经网络**
  进行障碍类别识别（无碍 / 偏左 / 偏右）与横向偏移估计。
- **轨迹控制**：用**控制神经网络**（策略网络）由「航向差 + 距目标距离」输出转向角，
  沿给定路点序列行驶。

## 2. 仿真环境

| 传感器 | 用途 |
|---|---|
| `sensor.camera.rgb` (640×480) | 路面/目标横向感知 |
| `sensor.camera.depth` (512×256) | 近处障碍密集度感知 |
| `sensor.lidar.ray_cast` (32 线) | 前方障碍点数/距离感知 |

## 3. 计算原理

### 3.1 特征提取与归一化

三路传感器压缩成 4 维特征向量（归一化，避免大数值特征主导梯度）：

$$
\mathbf{x} = \big[\, \underbrace{\tfrac{c_x-W/2}{W/2}}_{\text{横向偏移}},\,
\underbrace{\tfrac{\bar G}{255}}_{\text{深度密集度}},\,
\underbrace{\tfrac{N_{\text{front}}}{60}}\,,
\underbrace{\tfrac{D_{\min}}{20}}\,\big]^{\!\top}
$$

- $c_x$ 为 RGB 下半区亮色像素横向重心；
- $\bar G$ 为深度 ROI 均值；$N_{\text{front}}$ 为雷达前方点数；$D_{\min}$ 为最近距离。

### 3.2 感知神经网络（MLP 分类）

全连接感知器：输入 4 维 → 隐藏 12 维 → softmax 3 类。

$$
\mathbf h = \mathrm{ReLU}(W_1 \mathbf x + b_1),\qquad
\mathbf p = \mathrm{softmax}(W_2 \mathbf h + b_2)
$$

交叉熵损失：

$$
\mathcal L_{\text{cls}} = -\frac1N\sum_i \sum_k y_{ik}\log p_{ik}
$$

### 3.3 控制神经网络（策略网络）

控制器：输入 2 维（航向差 $e_\psi$、最近距离 $d$）→ 隐藏 16 维 → tanh 单输出转向。

$$
\mathbf h = \mathrm{ReLU}(W_1[\mathbf s]+ b_1),\qquad
\delta = \tanh(W_2 \mathbf h + b_2) \in [-1,1]
$$

采用均方误差回归：

$$
\mathcal L_{\text{ctrl}} = \frac1N\sum_i\big(\delta_i - \delta_i^*\big)^2
$$

反向传播用链式法则对 $W,b$ 求梯度，随机批量梯度下降更新。

## 4. 算法流程

```
train: 合成/真实数据 → 特征归一化
       → 感知 NN(交叉熵) + 控制 NN(MSE) 训练 → 保存 json
run  : 连 CARLA → 挂 RGB/深度/雷达
       → 每帧：特征提取 → 感知 NN 输出类别/偏移
       → 控制 NN 输出 steer → apply_control → world.tick
       → 记录横向误差 RMSE
```

## 5. 源码解析（`02_perception/main.py` + `nn_models.py`）

- `extract_features()`：三路传感器 → 4 维归一化特征。
- `MLPClassifier`（`nn_models.py`）：感知器，`forward()/backward()` 实现前向与 BP。
- `MLPPolicy`（`nn_models.py`）：控制网络，tanh 输出层回归。
- `train()`：交叉熵 + MSE 训练两个网络。
- `run_carla()`：在线加载模型，把特征喂给感知 NN，把状态喂给控制 NN。

## 6. 运行

```bash
# 1) 离线训练两个神经网络（仅 numpy，无需 CARLA）
python 02_perception/main.py --mode train --epochs 300 --out models/nn_percept.json
#    输出：感知 NN ~100% 精度；控制 NN MSE≈0.003

# 2) 在线感知 + 轨迹跟踪（需 CARLA 服务端）
python 02_perception/main.py --mode run --model models/nn_percept.json \
       --waypoints "40,-8 40,12 25,20" --sim_time 20
```

Windows 原生入口：

```bat
main.bat perception --mode train
main.bat perception --mode run
```

ROS launch：
```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash
ros2 launch carla_assignment 02_perception_launch.py mode:=run
# ROS1 Noetic
source /opt/ros/noetic/setup.bash
roslaunch carla_assignment 02_perception.launch mode:=run
```

## 7. 录屏剧本

1. 离线训练打印感知/控制 NN 的 loss 与精度下降曲线。
2. 在线运行：车辆自动沿路点行驶，终端打印每帧 `NN感知=类别 steer=... lidar(...)`
   与横向误差，录制 10-15 秒。

!!! note "运行动图"
    录屏后替换此占位图为真实动图：`![](assets/placeholder_perception.png)` → `![](assets/02_perception.gif)`

## 8. 性能评价

| 指标 | 数值（示例） |
|---|---|
| 感知 NN 精度 | ≈100%（训练集） |
| 控制 NN MSE | ≈0.003 |
| 横向误差 RMSE (m) | 运行后填写 |

- **感知有效性**：NN 感知类别与实际横向位置变化趋势一致。
- **NN 控制**：与纯跟踪对比，转向更平滑，RMSE 相当。
