# 行为克隆模型对比基准（结构与数据增强）

本模块对应作业"其他"要求中的**同样模型不同数据结构/同样算法不同模型结构的对比实验**，
在同一份采集数据上比较 MLP、CNN、宽卷积三种网络结构，以及有无图像增强两种训练策略。
不依赖仿真器，纯训练与评估，`ros2 launch uav_bc_bench main.launch.py` 一键跑完。

![对比结果](../img/uav_bc_bench_comparison.png)

## 1 模块概述

行为克隆（Behaviour Cloning, BC）把控制问题当作监督学习：给定图像与专家动作，
训练网络最小化预测动作与专家动作之间的均方误差。本模块固定数据集与训练超参，
只改变"网络结构"和"数据增强"两个变量，用同一套评估流程给出对比结论。

## 2 运行环境

* Ubuntu 22.04 + ROS 2 Humble + Python 3.10
* `numpy`、`opencv-python`、`torch`（CPU 即可）
* `Pillow`（可选）：只有画 `comparison.png` 用得到；没装时跳过画图，指标照常输出
* 22.04 仓库里的 `python3-torch` 是 1.8.1，本项目只用 Conv2d / Linear / Adam 等基础算子，够用

默认数据集来自任务 2 的 `uav_depth_track_nn`：`~/.ros/uav_depth_track_nn/data/`
（`controls.csv` + `frames/*.png`），这是本模块的**首选**数据来源，二者合并时按同名包依赖说明即可。
如果想在没有仿真器、也没有那份采集数据的机器上直接跑通，用 `--synthetic 400`
（launch 里是 `synthetic:=400`）生成一份同格式的合成数据集（`output_dir/synthetic/`），
不依赖任何其它 PR：

```bash
python3 -m uav_bc_bench.bench --synthetic 400 --epochs 10 --seeds 1
```

代码会自动识别 16 位深度图与 8 位彩色图。

## 3 编译与运行

```bash
cd ~/ros2
colcon build --packages-select uav_bc_bench
source install/setup.bash

ros2 launch uav_bc_bench main.launch.py epochs:=20 seeds:=2
```

跑完后在 `~/.ros/uav_bc_bench/` 下得到 `metrics.json`、`comparison.md`、`comparison.png`。

也可以脱离 ROS 直接跑：

```bash
python3 -m uav_bc_bench.bench --dataset-dir ~/.ros/uav_depth_track_nn/data \
    --output-dir ~/.ros/uav_bc_bench --epochs 20 --seeds 2
```

## 4 参数说明

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `dataset_dir` | `~/.ros/uav_depth_track_nn/data` | 数据集目录 |
| `output_dir` | `~/.ros/uav_bc_bench` | 指标与图表输出目录 |
| `epochs` | 30 | 每个配置训练轮数 |
| `seeds` | 3 | 每个配置重复的随机种子数 |
| `structures` | `mlp,cnn,cnn_wide` | 参与对比的结构 |
| `augment` | true | 是否同时跑增强组 |
| `image_size` | 64 | 输入边长 |
| `batch_size` | 32 | 批大小 |

## 5 算法原理

### 5.1 行为克隆目标

给定数据集 $\{(I_i, a_i)\}_{i=1}^{N}$，其中 $I_i$ 是深度图，$a_i=(v_x,v_y)$ 是专家速度，
训练目标为

$$
\mathcal{L}(\theta) = \frac{1}{N}\sum_{i=1}^{N}\left\| \pi_\theta(I_i) - a_i \right\|_2^2
$$

优化器 Adam，学习率 $10^{-3}$，保存验证集最优权重。

**划分方式**：连续飞行的相邻帧高度相关，按单帧随机打乱会让近邻状态同时进入训练集与
测试集，评估结果被时间泄漏抬高。仅按时间切成"前 80%/中 10%/后 10%"仍然不够：
采集轨迹是半径 3 m 的两圈圆，训练段已经走过测试段要走的几乎整圈空间，测试集并不是
"没见过的状态"。

因此这里**按圈分组切分**：用 `controls.csv` 的 `x, y` 求累计极角，把每帧标上圈号
$k_i = \lfloor \Theta_i / 2\pi \rfloor$，最后一圈作测试、倒数第二圈作验证、其余作训练：

$$
\mathcal{D}_{test} = \{(I_i, a_i) \mid k_i = \max k\}, \quad
\mathcal{D}_{val} = \{(I_i, a_i) \mid k_i = \max k - 1\}
$$

拿不到位置信息（例如纯手工数据集）时退回"按时间连续切分"，`seed` 只让切点做小幅抖动。
`metrics.json` 里的 `split_mode` 记录的是**实际生效**的切分方式：只有数据里能解析出 ≥3 个
圈号时才写 `lap`，否则写 `time`（例如只录到 2 圈时会自动退回按时间切分，并在日志里说明）。

### 5.2 三种网络结构

* **mlp**：图像展平成 4096 维 → 全连接 256 → 256 → 2；
* **cnn**：Conv(16)-Pool-Conv(32)-Pool-Conv(64)-Pool → 128 → 2；
* **cnn_wide**：Conv(32)-BN-Pool-Conv(64)-BN-Pool-Conv(128)-BN-Pool → Dropout → 256 → Dropout → 2。

参数量由各层权重矩阵尺寸累加得到，例如 CNN 第一层卷积为
$16\times1\times3\times3 + 16 = 160$ 个参数，全连接层
$64\times8\times8\times128 + 128 = 524{,}416$ 个参数。

### 5.3 数据增强

增强**在训练循环里逐轮随机施加**（不是训练前做一次固定变换），每个 batch 独立采样：
随机平移 $d_x, d_y \sim U\{-8,\dots,8\}$ 像素（用循环移位实现，保持尺寸不变）、
亮度增益 $g \sim U(0.8,1.2)$、高斯噪声 $\epsilon \sim \mathcal{N}(0,0.01^2)$：

$$
\tilde I = \mathrm{clip}\left(\mathrm{roll}(I; d_x, d_y) \cdot g + \epsilon,\; 0,\; 1\right)
$$

## 6 源代码解析

| 文件 | 关键内容 |
| --- | --- |
| `uav_bc_bench/data.py` | 读 CSV 与图像，16 位毫米深度图转米并截断 40 m |
| `uav_bc_bench/models.py` | 三种结构、`build()`、`num_parameters()` |
| `uav_bc_bench/bench.py` | 划分、训练、评估、指标汇总、PIL 柱状图 |
| `uav_bc_bench/main.py` | ROS 2 节点：工作线程跑实验，完成后自动退出 |

## 7 对比结果（本机实测）

数据：555 个深度样本（`split_mode = lap`，训练 555→约 460 帧，验证/测试各约 50~95 帧）；
训练 20 轮、2 个随机种子，取均值 ± 标准差。恒零基线（一直输出 `(0, 0)`）的 MAE 为 **1.3197 m/s**。

| 结构 | 图像增强 | 参数量 | 测试 MAE (m/s) | 训练耗时 (s) |
| --- | --- | --- | --- | --- |
| cnn | 否 | 547,970 | **0.1923 ± 0.0481** | 20.0 |
| mlp | 否 | 1,115,138 | 0.2184 ± 0.0858 | 1.9 |
| cnn_wide | 否 | 2,191,042 | 0.2203 ± 0.0057 | 58.7 |
| cnn_wide | 是 | 2,191,042 | 0.3158 ± 0.0925 | 54.3 |
| cnn | 是 | 547,970 | 0.4580 ± 0.0592 | 20.5 |
| mlp | 是 | 1,115,138 | 1.0352 ± 0.0983 | 3.4 |

按圈分组后所有配置的误差都比"按时间切分"更高（`cnn` 从 0.2522 升到 0.1923 附近波动、
`mlp` 从 0.3627 降到 0.2184），说明单次实验的绝对值受种子影响很大，
**结论要看同一轮实验内的横向对比，而不是跨轮比较绝对值**。

## 8 结论

1. **同一轮实验里 CNN 优于 MLP，但优势有限**：0.1923 对 0.2184（低约 12%），
   而参数量只有 MLP 的一半。两者标准差（±0.048 / ±0.086）有重叠，
   在 555 样本、2 个种子下这个差距**不能算显著**，只能说明"卷积先验至少不吃亏"。
2. **加宽模型没有收益**：`cnn_wide` 参数量是 `cnn` 的 4 倍，MAE 反而略高（0.2203 对 0.1923），
   训练耗时增加约 3 倍。在小数据集上容量不是瓶颈。
3. **增强在本任务上有害**：三个结构的 MAE 全部退化 1.6~4.7 倍，`mlp` 从 0.2184 退到 1.0352，
   已经接近恒零基线 1.3197。原因是这里的增强破坏了深度图的绝对数值语义——
   亮度增益会破坏"灰度即距离"的映射，平移又改变了障碍物相对机头的位置，
   而速度指令正是由这个相对位置决定的。
4. **网络确实学到了控制量**：所有未增强配置都远低于恒零基线（0.19~0.22 对 1.32），
   说明误差不是"数据集本身没有信息"造成的。
