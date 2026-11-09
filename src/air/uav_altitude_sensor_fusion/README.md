# uav_altitude_sensor_fusion

无人机 IMU 垂直加速度与气压高度的一维融合仿真。比较原始气压高度、加速度积分高度和
卡尔曼融合高度，输出 CSV 和误差曲线。

## 运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch uav_altitude_sensor_fusion main.launch
```

纯 Python 仿真：

```bash
cd src/air/uav_altitude_sensor_fusion
python3 scripts/main.py sim --duration 12 --rate 50
```

## 话题

- `/uav/altitude/barometer`：模拟气压高度，`std_msgs/Float32`
- `/uav/altitude/estimate`：融合高度，`std_msgs/Float32`
- `/uav/altitude/error`：融合高度误差，`std_msgs/Float32`
- `/uav/altitude/status`：状态，`std_msgs/String`

## 测试

```bash
python3 test/test_altitude_filter.py
```

## 范围限制

这是传感器融合教学仿真，不接真实 IMU 或气压计。真实传感器还需要时间戳、
温度补偿和故障检测。

本次 50 Hz、12 s 合成数据的 RMSE：原始气压 `0.573 m`、加速度积分
`24.140 m`、融合 `0.478 m`；融合最终误差 `0.426 m`。
