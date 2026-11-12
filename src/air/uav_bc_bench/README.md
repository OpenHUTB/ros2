# uav_bc_bench

行为克隆模型对比基准：同一数据集上比较 MLP / CNN / 宽卷积三种结构，以及有无图像增强两种策略。
对应作业"同样算法不同模型结构"的对比实验，不依赖仿真器。

## 环境

Ubuntu 22.04 + ROS 2 Humble + Python 3.10，依赖 `numpy` / `opencv-python` / `torch`（CPU）
以及可选的 `Pillow`（只用于画 `comparison.png`，缺了会跳过画图，指标照常输出）。

- 22.04 仓库里的 `python3-torch` 是 **1.8.1**，本项目只用基础算子（Conv2d/Linear/Adam），
  该版本可用；若你装了更新的 torch（如 2.x）同样兼容。
- `python3-pil` 在 `package.xml` 里声明为 `exec_depend`；`apt install python3-pil` 即可。

数据集默认取任务 2 的 `~/.ros/uav_depth_track_nn/data`（`controls.csv` + `frames/`）。
**没有这份数据也能跑**：`synthetic:=200`（launch）或 `--synthetic 200`（命令行）会现场生成
同格式的合成数据集到 `~/.ros/uav_bc_bench/synthetic/`，不依赖任何仿真数据。

数据集不存在且没有开 `synthetic` 时，节点会打印明确的错误并以**退出码 1** 结束，
不会被误当成"跑通了"。

## 运行

```bash
colcon build --packages-select uav_bc_bench
source install/setup.bash
ros2 launch uav_bc_bench main.launch.py epochs:=20 seeds:=2
# 没有真实数据集时：
ros2 launch uav_bc_bench main.launch.py synthetic:=200 epochs:=10 seeds:=1
```

或直接：

```bash
python3 -m uav_bc_bench.bench --epochs 20 --seeds 2
python3 -m uav_bc_bench.bench --synthetic 400 --epochs 10 --seeds 1   # 无仿真数据时
```

结果写到 `~/.ros/uav_bc_bench/`：`metrics.json`、`comparison.md`、`comparison.png`（需要 Pillow）。

`metrics.json` 里的 `split_mode` 记录的是**实际生效**的切分方式：只有拿到 ≥3 个圈号才会写
`lap`，否则写 `time`（数据里的 `x,y` 不足以区分圈数时会自动退回时间切分）。

## 数据划分

连续飞行的相邻帧高度相关，按单帧随机打乱会让近邻帧同时进入训练集与测试集（时间泄漏）。
仅按时间切三段也不够：两圈圆轨迹里训练段已经走过测试段的空间，测试集并非"没见过的状态"。

因此本模块**按圈分组切分**：用 `controls.csv` 的 `x, y` 累计极角得到圈号，
最后一圈做测试、倒数第二圈做验证、其余做训练；拿不到位置信息时退回"按时间连续切分"。
`metrics.json` 的 `split_mode` 记录实际使用的方式（`lap` / `time`）。

## 实测（555 样本，20 轮，2 种子，split_mode = lap）

| 结构 | 增强 | 参数量 | 测试 MAE (m/s) |
| --- | --- | --- | --- |
| cnn | 否 | 547,970 | 0.1923 ± 0.0481 |
| mlp | 否 | 1,115,138 | 0.2184 ± 0.0858 |
| cnn_wide | 否 | 2,191,042 | 0.2203 ± 0.0057 |

恒零基线（一直输出 0 速度）的 MAE 是 1.3197 m/s，所有模型都远低于它。

结论：同一轮实验里 CNN 比 MLP 低约 12%（0.192 对 0.218），参数量只有一半，
但两者标准差有重叠，在小样本下这个差距不算显著；加宽没有收益；
亮度/平移类增强在本任务上有害（0.19 → 0.46~1.04），因为它破坏了"灰度即距离"的语义。
详细分析见 `docs/air/uav_bc_bench.md`。
