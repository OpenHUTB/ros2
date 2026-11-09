# uav_altitude_sensor_fusion

无人机 IMU 垂直加速度与气压高度的一维融合仿真。比较原始气压高度、加速度积分高度和
卡尔曼融合高度，输出 CSV 和误差曲线。

## 环境

- Ubuntu 20.04 + ROS Noetic（catkin）
- Python 3
- `python3-numpy`、`python3-pil`

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
python3 scripts/main.py --duration 12 --rate 50
```

## 话题

- `/uav/altitude/barometer`：模拟气压高度，`std_msgs/Float32`
- `/uav/altitude/estimate`：融合高度，`std_msgs/Float32`
- `/uav/altitude/error`：融合高度误差，`std_msgs/Float32`
- `/uav/altitude/status`：状态，`std_msgs/String`

## 测试

catkin 自动测试：

```bash
cd ~/catkin_ws
catkin_make run_tests_uav_altitude_sensor_fusion
catkin_test_results
```

也可以单独运行 unittest：

```bash
python3 -m unittest discover -s test -v
```

## 范围限制

这是传感器融合教学仿真，不接真实 IMU 或气压计。真值使用 8 s 五次多项式从地面平滑
起飞到 8 m；加速度积分与真值使用相同的初始高度和速度，以免把初速度不一致误算成
积分漂移。

气压计包含 `0.45 m` 常值偏置和 `0.35 m` 零均值噪声。滤波器状态只有高度和速度，
没有估计偏置，因此融合误差仍受该常值偏置限制。`Q=0.015/0.08` 是为本仿真整定的
教学参数，并非实机标定值。

本次 50 Hz、12 s 合成数据的 RMSE：原始气压 `0.573 m`、加速度积分
`2.832 m`、融合 `0.484 m`；融合最终误差 `0.426 m`。

## AI 使用声明

使用 DeepSeek Harness 辅助代码、测试和文档整理；算法参数、运行结果与提交内容由作者复核并负责。
