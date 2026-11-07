# muscle_force_distribution

肌肉冗余系统的力分配：最小激活范数与加权最小二乘。

## 运行

只依赖 Python 标准库：

```bash
cd src/musculoskeletal/muscle_force_distribution
python3 scripts/main.py --outdir scripts
```

生成 `force_alloc.png`（各肌激活度随时间变化）与 `force_tracking.png`（力矩跟踪），
并输出跟踪最大误差。

## 测试

```bash
cd src/musculoskeletal/muscle_force_distribution
python3 -m unittest discover -s tests
```

## 内容

- `scripts/force_alloc.py`：肌肉模型、可行区间、最小范数/加权最小二乘分配、刚度代理
- `scripts/simulate.py`：对时变期望力矩做分配并作图
- `scripts/simple_plot.py`：仅用 zlib 的极简折线图输出
- `tests/test_force_alloc.py`：单元断言

## 说明

单自由度关节被多块肌肉冗余驱动，每块肌肉提供力矩 `tau_i = r_i * (a_i * Fmax_i)`。
给定期望力矩后，满足 `sum tau_i = tau_des` 的解有无穷多个；本模块给出最小激活范数
（least effort）与加权最小二乘两种分配，并展示冗余肌群如何平分负载、拮抗共激活
如何在不产生净力矩的前提下提高关节刚度。文档见
`docs/musculoskeletal/muscle_force_distribution.md`。
