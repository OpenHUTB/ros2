# sliding_mode_control

滑模控制（SMC）：二阶系统的鲁棒轨迹跟踪。

## 运行

只依赖 Python 标准库（无需 ROS / MuJoCo / matplotlib）：

```bash
cd src/control/sliding_mode_control
python3 scripts/main.py --outdir ../../../docs/img/control/sliding_mode_control
```

会在输出目录生成 `smc_response.png`（SMC 与 PD 在扰动下的跟踪对比），
并打印两种控制器的平均/稳态误差。

## 测试

```bash
cd src/control/sliding_mode_control
python3 -m unittest discover -s tests
```

## 内容

- `scripts/smc_model.py`：被控对象、参考、扰动、SMC 与 PD 控制器与仿真
- `scripts/simulate.py`：组装曲线并调用手写 PNG 作图（`simple_plot.py`）
- `scripts/simple_plot.py`：仅用 zlib 的极简折线图输出
- `tests/test_smc.py`：单元断言（7 项，含 tanh 边界层常值扰动残差与解析值对照）

## 说明

被控对象 `m*x_ddot + c*x_dot + k*x = u + d(t)` 含未知有界扰动 `d(t)`。
SMC 定义滑模面 `s = e_dot + lambda*e`，用等效控制抵消标称动力学、用边界层切换项
`u_sw = -K*tanh(s/phi)` 抑制扰动；只要 `K` 大于扰动幅值上界即可进入滑动模态。
文档见 `docs/control/sliding_mode_control.md`。
