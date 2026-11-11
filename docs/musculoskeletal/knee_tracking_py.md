# 膝关节肌肉骨骼仿真与轨迹跟踪（纯 Python 复现版）

> 模块归属：**肌肉骨骼人** · 目录：`src/musculoskeletal/knee_tracking_py/`
> 运行环境：Python ≥ 3.10 + NumPy + matplotlib（GIF 需要 Pillow），**不需要 ROS / MuJoCo / CARLA**

这是 [musculoskeletal/knee_tracking.md](../../musculoskeletal/knee_tracking.md) 的独立纯 Python 复现：保留原页面的模型、控制律与指标口径，
用 NumPy 重写执行层，因此可以在没有 ROS 与仿真器的机器上完整跑通并复现实验数字。
上游同名模块 `src/musculoskeletal/knee_tracking/` 保持不变，本目录以 `_py` 后缀与它并存。

## 1. 包含什么

- `main.py`：单一入口，跑完生成 CSV、PNG、GIF 与指标文件；
- `scripts/` 或 `src/`：模型、求解器、指标与绘图，按职责拆分；
- `tests/`：单元测试；
- `README.md`：模型推导、参数表、结果对照表、已知限制。

## 2. 怎么跑

```bash
cd src/musculoskeletal/knee_tracking_py
python main.py          # 完整实验
python -m pytest -q     # 单元测试
```

产物写到 `out/`（或 `results/`），不依赖任何服务或外部进程。

## 3. 关键结果

- 同一组 PD 增益下，无重力前馈稳态下垂 −5.97°，加前馈后 0.0000°、调节时间 0.29 s；
- 正弦跟踪 RMSE 8.61°→8.33°，剩余残差来自肌肉激活/失活时间常数（0.02/0.05 s）；
- 91 个单元测试覆盖模型、指标、配置与 CLI。

## 4. 与上游模块的关系

- 上游 `src/musculoskeletal/knee_tracking/` 依赖 ROS 运行时（部分依赖 MuJoCo/CARLA），面向真实仿真链路；
- 本目录是**离线、可独立验证**的复现，用于教学、对照与回归，不替代上游实现；
- 两者的参数口径与差异已写在本目录 `README.md` 的"已知限制"与"复现说明"中。

## 5. 边界

不含 ROS 节点、话题、launch 与真机实验；所有数字由本目录代码实跑得到，
不照抄文档。若后续要与上游实现统一，可按维护者意见合并或改名。
