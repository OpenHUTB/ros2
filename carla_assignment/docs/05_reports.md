# 作业五 · CARLA 整合 + 性能评价

> 把前四份作业整合，统一入口启动，并提供性能评价指标方法。

## 1. 任务目标

- 用一个入口管理 4 次作业（整合）。
- 提供性能评价指标与生成工具（`scripts/gen_train_loss.py`、两个 NN 训练 loss/精度）。
- 作业二/三/四均为神经网络：感知（MLP）、规划（MLP）、端到端（CNN）；本作业汇总其
  评价指标（转向平滑度 AoS、速度、横向误差 RMSE、NN loss/MSE）。

## 2. 计算原理（性能指标）

### 2.1 转向平滑度 (AoS)

$$
\text{AoS} = \frac{1}{N-1}\sum_{k=2}^{N} |\text{steer}_k - \text{steer}_{k-1}|
$$

越小越平滑。

### 2.2 平均 / 最大速度

$$
\bar v = \frac{1}{N}\sum_k v_k,\qquad v_{\max}=\max_k v_k
$$

### 2.3 横向误差 RMSE

$$
\text{RMSE} = \sqrt{\tfrac{1}{N}\sum_k e_{y,k}^{\,2}}
$$

## 3. 算法流程

```
用户选 target（control/perception/navigation/end_to_end）或 --benchmark
→ subprocess 启动对应模块 main.py
（--benchmark 离线运行评价示例）
端到端训练/验证 loss 曲线：scripts/gen_train_loss.py（离线）
```

## 4. 源码解析（`05_reports/main.py`）

- `MODULES`：子作业 main.py 路径映射。
- `run_module()`：subprocess 拉起。
- `benchmark()`：离线生成模拟速度/转向/误差并计算指标。

## 5. 运行

```bash
# 性能评价（示例）
python 05_reports/main.py --benchmark

# 启动某作业（示例）
python 05_reports/main.py --target navigation --goal 20,8

# 生成端到端训练 loss 曲线（采集后离线执行）
python scripts/gen_train_loss.py --data_dir dataset --epochs 30
```

## 6. 性能评价表

本机 `--benchmark` 示例输出如下（真实值请用真车运行后填写）：

| 指标 | 示例值 |
|---|---|
| 平均速度 (m/s) | 9.01 |
| 最大速度 (m/s) | 10.2 |
| 转向均值 | 0.009 |
| 转向标准差 | 0.123 |
| 转向变化量 AoS | 0.138 |
| 横向误差 RMSE (m) | 0.19 |

端到端训练/验证曲线见 `04_end_to_end` 文档（由 `scripts/gen_train_loss.py` 生成）。
