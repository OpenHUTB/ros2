# 附带日志说明

`gust_ppo_101.csv` 是 `uav_vertical_ppo`（PR #490）gust 场景的一次评估回合，仅用于演示本模块的离线统计与绘图。

| 项目 | 取值 |
|---|---|
| 场景 | gust（2–3 s 施加 1.5 N 向下外力） |
| 策略 | PPO，随包模型 `hover_ppo.zip` |
| 评估种子 | 101 |
| 目标高度 | 1.5 m |
| 采样 | 50 Hz，0–8 s，共 400 条 |
| 生成方式 | 在 `uav_vertical_ppo` 中执行 `python3 main.py evaluate --gif --output outputs/eval`，取 `outputs/eval/gust_ppo_101.csv` |

该包尚未合并进本仓库，因此这份日志无法在仓库内重新生成；如需复现，请在该 PR 的分支上运行上面的命令。日志只读取 `time_s,height_m,target_m` 三列，其余列（推力、速度等）在本模块中不使用。
