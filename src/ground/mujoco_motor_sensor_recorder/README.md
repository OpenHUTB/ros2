# 电机传感器 CSV 记录器

本包为通用电机状态 CSV 加入可复现的温度和电流测量噪声。包可独立运行，不导入也不要求仿真器包。

## 输入 CSV

必需字段：`time_s`、`speed_mps`、`left_temperature_c`、`right_temperature_c`、`left_torque_nm`、`right_torque_nm` 和 `ambient_c`。

## 运行

```bash
python3 main.py --input config/motor_state.csv --output output/motor_sensors.csv --seed 7
```

温度噪声标准差为 0.12 °C，电流噪声标准差为 0.04 A。电流由转矩按 0.18 Nm/A 转矩常数估算，并加入 0.25 A 空载电流。

## ROS 2 启动

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_motor_sensor_recorder
source install/setup.bash
ros2 launch mujoco_motor_sensor_recorder record.launch.py input:=config/motor_state.csv output:=output/motor_sensors.csv
```
