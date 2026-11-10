# 复现指南

从零复现本文所有结果，大约需要 **25 分钟**（4 核 CPU，无 GPU）。

## 0. 环境

```bash
python --version        # 3.8 - 3.11 均可
pip install -r requirements.txt
pip install -e .
```

依赖只有五个：`numpy`、`Pillow`、`matplotlib`、`mkdocs`、`mkdocs-material`。
**不需要** PyTorch / CUDA / OpenCV / ROS（ROS 只在部署时需要）。

## 1. 冒烟测试（30 秒）

```bash
python main.py --module sim --scene sparse
```

预期：`runs/<时间>_sim/frame_sparse.png`，控制台打印场景与渲染耗时。

```bash
python main.py --module full --scene sparse --gif
```

预期：约 8 秒跑完 435 步，输出 `full_sparse.gif`（约 9 MB）。

## 2. 重新生成数据（1 分钟）

```bash
python tools/make_data.py --out data --e2e-episodes 60 --e2e-limit 14000
```

四个数据集：

| 文件 | 样本 | 内容 |
| --- | --- | --- |
| `perception.npz` | 1,200 | 48×64 深度 + 占用 mask |
| `planner.npz` | 6,500 | 11×11 patch + 目标偏移 + cost-to-go |
| `control.npz` | 15,305 | 86 维特征 + 机体系速度 |
| `e2e.npz` | 7,373 | 48×64 深度 + 目标 + 速度（蒸馏 + 随机目标） |

## 3. 训练（约 10 分钟）

```bash
python tools/train.py --module all --data data --weights weights
```

分模块训练：

```bash
python tools/train.py --module perception --epochs 4
python tools/train.py --module planner --epochs 12
python tools/train.py --module control --epochs 60
python tools/train.py --module e2e --epochs 12
```

以上轮数就是 `ca_drone/training/train.py` 的默认值，也是随附权重实际训练的轮数；
`--epochs` 可省略，写出来是为了让读者一眼看到口径。

预期最终指标：

| 网络 | 指标 | 值 |
| --- | --- | --- |
| perception | val IoU | 0.956 |
| planner | val MAE / R² | 2.52 m / 0.910 |
| control | val MAE / 角度 | 0.0356 m/s / 1.9° |
| e2e | val MAE | 0.0348 m/s |

## 4. 评测（约 1.5 分钟）

```bash
python tools/evaluate.py --scenes all --modes expert,control,full,e2e --seeds 3
```

输出 `results/episodes.csv`（60 行 = 5 场景 × 4 方法 × 3 种子）与
`results/summary.json`。预期：expert / control / full 成功率 100%，e2e 40%
（dynamic 与留出的 unseen）。

## 5. 图表与文档

```bash
python tools/make_figs.py          # docs/assets/figs/*.png
mkdocs serve                       # http://127.0.0.1:8000
mkdocs build --strict              # 站点输出到 site/
```

## 6. 训练可复现性

- 数据生成、训练、评测都固定了 `seed`；
- 权重以 `npz` 明文存储，可用 `numpy.load` 直接检查；
- 训练日志在 `weights/<模块>_history.json`，逐 epoch 记录 loss 与验证指标；
- 由于 NumPy 的浮点归约顺序与 BLAS 实现有关，不同机器上的末位小数可能不同，
  但成功率与指标量级可复现。

## 7. 只想要结果，不想训练

仓库已附带 `weights/*.npz`。直接：

```bash
python tools/evaluate.py --scenes all --modes full --seeds 3
python main.py --module full --scene corridor --gif
```
