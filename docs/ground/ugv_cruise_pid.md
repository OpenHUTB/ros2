# 地面车 PID 巡航速度跟踪（ugv_cruise_pid）

`ugv_cruise_pid` 是一个纯 NumPy 离线仿真模块，模拟一维地面车速度通道在
阶跃目标速度（0 → 5 → 8 → 3 m/s）与 t=12–14s 逆风阵风扰动下的闭环跟踪。
控制器采用带积分抗饱和与加速度限幅的 PID，输出阶跃响应指标与速度/控制量
曲线。不需要 ROS、Gazebo 或 CARLA。

源码在 `src/ground/ugv_cruise_pid/`，包根入口是 `main.py`，核心实现是
`scripts/cruise_pid_node.py`。

## 它怎么工作的

- **车辆模型 `UGVPlant`**：一维质点动力学，加速度 = 控制指令 / 质量 + 扰动，
  速度非负并积分得到行程。
- **PID 控制器 `PIDCruiseController`**：对速度误差做比例-积分-微分，积分项
  限幅抗饱和，加速度指令限幅。
- **速度曲线 `speed_profile`**：t<2s 为 0，2–10s 为 5 m/s，10–15s 为 8 m/s，
  之后为 3 m/s。
- **阵风扰动 `wind_gust`**：12–14s 叠加 -1.0 m/s² 的逆风加速度。

阶跃响应指标在阵风到来前的平稳段测量，阵风峰值与扰动后残差单独报告，避免
扰动窗口污染阶跃响应数据。

## 运行

环境与步骤见模块内
[`README.md`](https://github.com/OpenHUTB/ros2/blob/master/src/ground/ugv_cruise_pid/README.md)。

```bash
cd src/ground/ugv_cruise_pid
python3 main.py
# 或
python3 scripts/cruise_pid_node.py
```

运行后打印指标并在当前目录生成 `cruise_response.png`。回归测试：

```bash
python -m unittest discover -s tests -v
```

## 运行效果

![运行效果](../img/ground/ugv_cruise_demo.gif)

## 指标口径与验证记录

| 指标 | 测量窗口 | 口径 |
|---|---|---|
| 阶跃稳态误差 | 11.0–11.9s | 8 m/s 目标段、阵风前 |
| 阶跃超调 | 10.0–12.0s | 8 m/s 阶跃、阵风前 |
| 阵风峰值 | 12.0–14.0s | 逆风阵风期间最大速度偏差 |
| 扰动后残差 | 14.5–14.9s | 阵风结束后、3 m/s 阶跃前 |

本地 Python 3.13 + NumPy 离线仿真实测：末速 ≈ 2.94 m/s（目标 3.0），
阶跃稳态误差 ≈ 0.11 m/s，阶跃超调 ≈ 0.21 m/s，总行程 ≈ 95.6 m。
回归测试 5 项断言全部通过（稳态误差 < 0.2 m/s、超调 < 0.3 m/s、
总行程 80–110 m、末速收敛、积分抗饱和限幅）。

## 声明

代码由 AI 辅助生成，作者已在本地验证运行结果与上述指标，并对提交内容负责。
