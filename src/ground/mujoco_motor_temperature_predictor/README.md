# 电机温度时序预测器

本包对用户提供的温度时序 CSV 拟合紧凑的多项式回归模型，并在后续时间段评估预测误差。不导入仿真器、传感器记录器或控制器包。

## CSV 格式

必需字段：`time_s`、`speed_mps`、`left_temperature_c`、`left_current_a` 和 `ambient_c`。数据行必须按时间严格递增。预测目标为当前时刻之后达到指定预测时域的第一条左电机测温记录。

## 运行

```bash
python3 main.py --csv path/to/motor_sensors.csv --horizon 5 --model output/temperature_model.json --results output/forecast_results.json
```

前 80% 的时间序列用于拟合；训练/测试边界隔离一个完整预测时域，最后 20% 用于计算 RMSE。

## ROS 2 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_motor_temperature_predictor
source install/setup.bash
ros2 launch mujoco_motor_temperature_predictor evaluate.launch.py csv:=path/to/motor_sensors.csv
```
