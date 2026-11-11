# ROS 2 电机温度保护节点

本包订阅标准 `sensor_msgs/Temperature` 和 `geometry_msgs/Twist` 消息，根据连续测温估计短时温升，按迟滞阈值缩放速度并发布限幅后的 `Twist` 指令。不依赖仿真器、预测器或传感记录包。

## 话题

- Input temperature: `/motor/temperature`
- Input command: `/cmd_vel`
- Output command: `/thermal_guard/cmd_vel`
- Predicted temperature and speed scale: `/thermal_guard/predicted_temperature`, `/thermal_guard/speed_scale`

## 运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_motor_thermal_guard
source install/setup.bash
ros2 launch mujoco_motor_thermal_guard thermal_guard.launch.py limit_c:=65 release_c:=60 horizon_s:=5
```

在首次收到测温或测温超过 `temperature_timeout_s`（默认 0.5 秒）后，线速度和角速度输出为零。
