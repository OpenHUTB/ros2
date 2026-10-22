# 无人机制动仿真入口 / UAV braking entry

当前模块的物理模型、控制方程、运行步骤与结果见[室内无人机安全制动与定高悬停](../air/mujoco_uav_range_neural_braking.md)。

See [Indoor UAV safe braking and altitude hold](../air/mujoco_uav_range_neural_braking.md) for the physical model, controller, execution steps and results.

```bash
python src/ground/mujoco_drone_distance_stop/main.py --headless --output results
```

该入口转发至 `src/air/mujoco_uav_range_neural_braking/main.py`。模型保留重力，采用连续施力制动与高度反馈。

This entry forwards to `src/air/mujoco_uav_range_neural_braking/main.py`. The model retains gravity and uses continuous braking forces with altitude feedback.
