# 附带日志说明

`gust_ppo_101.csv` 来自 [`uav_vertical_ppo` 的 gust 场景评估](https://github.com/Gao2006-12/ros2/tree/919e56cf0694fcb451ff088f82a1daacb8acdcfe/src/air/uav_vertical_ppo)，用于本模块的离线统计与绘图。该源版本对应 PR #490。

| 项目 | 取值 |
|---|---|
| 场景 | gust（2–3 s 施加 1.5 N 向下外力） |
| 策略 | PPO，随包模型 `hover_ppo.zip` |
| 评估种子 | 101 |
| 目标高度 | 1.5 m |
| 采样 | 50 Hz，首末时间约 0.02–8.00 s，共 400 条 |
| 生成方式 | 在 `uav_vertical_ppo` 中执行 `python3 main.py evaluate --gif --output outputs/eval`，取 `outputs/eval/gust_ppo_101.csv` |

在上述源版本的模块目录运行生成命令，即可得到同名日志。本模块独立读取 `time_s,height_m,target_m` 三列，不依赖训练或仿真包；其余列（推力、速度等）不参与统计。
