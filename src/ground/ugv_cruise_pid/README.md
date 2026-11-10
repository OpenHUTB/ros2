# ugv_cruise_pid

地面车 PID 巡航速度跟踪仿真：在阶跃目标速度（0 → 5 → 8 → 3 m/s）与
t=12–14s 逆风阵风扰动下，用带积分抗饱和与加速度限幅的 PID 控制器跟踪
目标速度，输出阶跃响应指标与速度/控制量曲线。纯 NumPy 离线仿真，不需要
ROS、Gazebo 或 CARLA。

## 目录

| 文件 | 职责 |
|---|---|
| `main.py` | 包根入口，薄封装转调 `scripts/cruise_pid_node.py` |
| `scripts/cruise_pid_node.py` | 唯一实现：车辆模型 `UGVPlant`、PID 控制器 `PIDCruiseController`、仿真与指标评测、绘图 |
| `tests/test_cruise_pid.py` | 回归测试：稳态误差 / 超调 / 总行程 / 末速 / 积分抗饱和断言 |

> 本模块只保留一份控制器实现；`main.py` 与测试均从 `scripts/cruise_pid_node.py`
> 复用，不再维护第二份重复脚本。

## 环境

- Python 3.13，NumPy，matplotlib（纯离线仿真，无 ROS / 仿真器依赖）
- 评审环境记录：Windows 11 + Python 3.14.5，numpy 2.5.2，matplotlib 3.11.2

## 运行

在本目录下执行：

```bash
python3 main.py
# 或直接运行核心脚本
python3 scripts/cruise_pid_node.py
```

运行后打印阶跃响应指标，并在当前目录生成 `cruise_response.png`。

## 指标口径

阵风作用在 t=12–14s。阶跃响应指标在阵风到来前的平稳段测量，阵风峰值与
扰动后残差单独报告，避免扰动窗口污染阶跃响应数据：

| 指标 | 测量窗口 | 口径 |
|---|---|---|
| 阶跃稳态误差 | 11.0–11.9s | 8 m/s 目标段、阵风前 |
| 阶跃超调 | 10.0–12.0s | 8 m/s 阶跃、阵风前 |
| 阵风峰值 | 12.0–14.0s | 逆风阵风期间最大速度偏差 |
| 扰动后残差 | 14.5–14.9s | 阵风结束后、3 m/s 阶跃前 |

实测参考（NumPy 离线仿真）：稳态误差 ≈ 0.11 m/s，超调 ≈ 0.21 m/s，
总行程 ≈ 95.6 m，末速 ≈ 2.94 m/s（目标 3.0 m/s）。

## 回归测试

```bash
python -m unittest discover -s tests -v
```

断言覆盖：阶跃稳态误差 < 0.2 m/s、超调 < 0.3 m/s、总行程 80–110 m、
末速收敛到 3 m/s 附近、积分项被抗饱和限幅。

## 大模型使用声明

代码由 AI 辅助生成，作者已在本地验证运行结果与上述指标。
