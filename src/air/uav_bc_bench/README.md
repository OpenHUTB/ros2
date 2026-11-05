# uav_bc_bench

行为克隆模型对比基准：同一数据集上比较 MLP / CNN / 宽卷积三种结构，以及有无图像增强两种策略。
对应作业"同样算法不同模型结构"的对比实验，不依赖仿真器。

## 环境

Ubuntu 22.04 + ROS 2 Humble + Python 3.10，`numpy` / `opencv-python` / `torch`（CPU）。

数据集默认取任务 2 的 `~/.ros/uav_depth_track_nn/data`（`controls.csv` + `frames/`）。

## 运行

```bash
colcon build --packages-select uav_bc_bench
source install/setup.bash
ros2 launch uav_bc_bench main.launch.py epochs:=20 seeds:=2
```

或直接：

```bash
python3 -m uav_bc_bench.bench --epochs 20 --seeds 2
```

结果写到 `~/.ros/uav_bc_bench/`：`metrics.json`、`comparison.md`、`comparison.png`。

## 实测（555 样本，20 轮，2 种子）

| 结构 | 增强 | 参数量 | 测试 MAE (m/s) |
| --- | --- | --- | --- |
| cnn | 否 | 547,970 | 0.1803 ± 0.0039 |
| cnn_wide | 否 | 2,191,042 | 0.1828 ± 0.0035 |
| mlp | 否 | 1,115,138 | 0.2929 ± 0.0036 |

结论：CNN 明显优于 MLP；加宽收益很小；本任务上亮度/裁剪类增强有害。
详细分析见 `docs/air/uav_bc_bench.md`。
