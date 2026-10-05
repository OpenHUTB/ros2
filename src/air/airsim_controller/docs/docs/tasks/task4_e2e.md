# 任务4 端到端模型

> 端到端（图像 → 控制）行为克隆：前视深度相机图像 → CNN → (v, ω) 速度指令，实现"感知即决策"的端到端飞行。

## 1. 算法原理

### 1.1 端到端行为克隆

传统感知-规划-控制流水线将任务拆分为障碍物检测、路径规划、轨迹跟踪三个模块，误差逐级累积。端到端方法直接用**单帧深度图**回归**速度指令**，将感知与决策融合进一个神经网络：

\[
\mathbf u_t = (v_t, \omega_t) = f_\theta(I_t)
\]

其中 \(I_t\) 为前视深度相机图像，\(f_\theta\) 为 CNN 参数化策略，\(\mathbf u_t\) 为机体坐标系线速度与偏航角速度。

### 1.2 网络结构 E2ECNN

| 层 | 结构 | 输出尺寸 |
|---|---|---|
| 输入 | 深度图 64×64×1 | 64×64 |
| Conv1 | 3×3, 16 通道 + ReLU + 2×2 池化 | 32×32 |
| Conv2 | 3×3, 32 通道 + ReLU + 2×2 池化 | 16×16 |
| Flatten | 展平 | 8192 |
| FC1 | 线性 → 128 + ReLU | 128 |
| FC2 | 线性 → 2 + Tanh | 2 |

\[
\mathbf u_t = \tanh(W_2\, \mathrm{ReLU}(W_1\, \mathrm{flatten}(\mathrm{CNN}(I_t))))
\]

网络输出经 Tanh 归一化到 \([-1,1]\)，再映射为实际速度：

\[
v = \frac{\hat v + 1}{2} V_{\max}, \qquad \omega = \hat \omega \, \Omega_{\max}
\]

### 1.3 深度图预处理

AirSim `DepthPerspective` 图像必须设置 `pixels_as_float=True` 才返回浮点深度，且**单位是厘米**：

```python
depth = airsim.list_to_2d_float_array(resp[0].image_data_float, w, h)
depth = depth / 100.0                          # cm -> m
depth = np.clip(depth, 0.0, MAX_RANGE) / MAX_RANGE   # 归一化到 [0,1]
```

## 2. 代码结构

| 文件 | 作用 |
|---|---|
| `config.py` | 任务4参数（相机、控制频率、量程、训练超参） |
| `collect_data.py` | 专家数据采集：起飞爬升 5m → 绕飞 → 深度图+动作 → `data/e2e_data.npz` |
| `train.py` | 行为克隆训练 E2ECNN，输出 `data/e2e_cnn.pt` |
| `main.py` | 推理飞行：深度图 → CNN → (v, ω) → 机体速度控制，Ctrl+C 停止 |

### 2.1 专家采集策略（collect_data.py）

采集时用**启发式专家规则**生成标签，避免人工遥控：

```python
if center_mean < 0.30:   v = 0.3; w = 0.0    # 前方近 -> 减速
else:                    v = 1.0; w = 0.5    # 前方远 -> 直行略转
```

## 3. 运行步骤

### 3.1 数据采集

```bash
cd ~/airsim_controller/modules/task4_e2e
python3 collect_data.py
```

- 无人机自动起飞并爬升至 5 m（避免贴地飞行）
- 绕飞采集深度图与专家动作，约 2 分钟
- 输出 `data/e2e_data.npz`（如 844 帧）

### 3.2 训练

```bash
python3 train.py
```
![alt text](<屏幕截图 2026-10-05 162639.png>)
行为克隆最小化预测与专家动作的均方误差：

\[
\mathcal L = \frac{1}{N}\sum_{i=1}^{N} \lVert \mathbf u_i - f_\theta(I_i)\rVert^2
\]

30 个 epoch 约 1-2 分钟，输出 `data/e2e_cnn.pt`。

### 3.3 端到端推理飞行

```bash
python3 main.py
```

循环：获取深度图 → CNN 前向 → 速度指令 → `moveByVelocityBodyFrameAsync` 控制，并带安全兜底（前方均值过低时悬停），`Ctrl+C` 停止后输出性能小结。
![alt text](task4_e2e(1).gif)![alt text](task4_e2e(2).gif)
## 4. 算法流程

```mermaid
graph TD
    A[起飞爬升至 5m] --> B[获取前视深度图 256x144]
    B --> C[下采样 64x64 + 归一化]
    C --> D[CNN 前向推理]
    D --> E[Tanh -> (v, omega)]
    E --> F{前方过近?}
    F -- 是 --> G[减速 / 悬停]
    F -- 否 --> H[机体速度控制]
    H --> I{用户终止?}
    I -- 否 --> B
    I -- 是 --> J[悬停 + 降落]
```

## 5. 性能评价（示例）

| 指标 | 数值 |
|---|---|
| 采集帧数 | 844 帧 |
| 训练时间 | ~90 s |
| 推理帧率 | ~20 FPS |
| 前方最近距离（全程） | ≥ 1.8 m |
| 碰撞次数 | 0 |

> 指标需在多场景（Blocks、简单街区等）下复测后回填。
