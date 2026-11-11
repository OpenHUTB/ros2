# 电机温度预测评估

本包从按时间排序的 CSV 序列评估五秒电机温度预测，输入字段有明确约定，不依赖其他 ROS 包。

## 验证

测试构造确定性时序，检查预测目标时间对齐、按时间留出测试段，并拒绝乱序数据。运行命令：

```bash
PYTHONPATH=src/ground/mujoco_motor_temperature_predictor python -m pytest -q src/ground/mujoco_motor_temperature_predictor/test
```

RMSE 仅对应输入的时间序列和预测时域。
