# 人形机器人全身逆运动学与可操作性（纯 Python 复现版）

> 模块归属：**人形机器人** · 目录：`src/humanoid/whole_body_ik_py/`
> 运行环境：Python ≥ 3.10 + NumPy + matplotlib（GIF 需要 Pillow），**不需要 ROS / MuJoCo / CARLA**

这是 [humanoid/whole_body_ik.md](../../humanoid/whole_body_ik.md) 的独立纯 Python 复现：保留原页面的模型、控制律与指标口径，
用 NumPy 重写执行层，因此可以在没有 ROS 与仿真器的机器上完整跑通并复现实验数字。
上游同名模块 `src/humanoid/whole_body_ik/` 保持不变，本目录以 `_py` 后缀与它并存。

## 1. 包含什么

- `main.py`：单一入口，跑完生成 CSV、PNG、GIF 与指标文件；
- `scripts/` 或 `src/`：模型、求解器、指标与绘图，按职责拆分；
- `tests/`：单元测试；
- `README.md`：模型推导、参数表、结果对照表、已知限制。

## 2. 怎么跑

```bash
cd src/humanoid/whole_body_ik_py
python main.py          # 完整实验
python -m pytest -q     # 单元测试
```

产物写到 `out/`（或 `results/`），不依赖任何服务或外部进程。

## 3. 关键结果

- 2R 正解与解析式偏差 2.2e−16 m，几何雅可比与中心差分偏差 6.3e−10；
- 伪逆步长按 1/σ_min 增长（8.944），自适应 DLS 压到 2.9e−4；
- 7 自由度手臂圆轨迹最大误差 3.6e−7 m（竖直）/ 1.8e−9 m（水平），可操作性闭式解对照 1.4e−16。

## 4. 与上游模块的关系

- 上游 `src/humanoid/whole_body_ik/` 依赖 ROS 运行时（部分依赖 MuJoCo/CARLA），面向真实仿真链路；
- 本目录是**离线、可独立验证**的复现，用于教学、对照与回归，不替代上游实现；
- 两者的参数口径与差异已写在本目录 `README.md` 的"已知限制"与"复现说明"中。

## 5. 边界

不含 ROS 节点、话题、launch 与真机实验；所有数字由本目录代码实跑得到，
不照抄文档。若后续要与上游实现统一，可按维护者意见合并或改名。
