# 无人机 IMU 与气压高度融合

本实验用合成的 IMU 垂直加速度和气压高度数据演示一维卡尔曼滤波。无人机从地面
平滑上升到目标高度，比较原始气压高度、加速度积分高度和融合高度。

源码位置：`src/air/uav_altitude_sensor_fusion/`

## 实验目标

- 理解 IMU 加速度积分产生漂移；
- 理解气压高度存在偏置和噪声；
- 用一维卡尔曼滤波融合两种传感器；
- 比较融合前后的 RMSE 和最终误差；
- 提供 ROS 话题、CSV 和响应曲线。

## 融合模型

状态为：

```text
x = [altitude, vertical_velocity]
```

IMU 预测：

```text
altitude += velocity * dt + 0.5 * acceleration * dt²
velocity += acceleration * dt
```

气压计更新使用高度观测，过程噪声和气压计噪声在 `altitude_filter.py` 中作为参数。

## 运行

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch uav_altitude_sensor_fusion main.launch
```

纯 Python：

```bash
cd src/air/uav_altitude_sensor_fusion
python3 scripts/main.py sim --duration 12 --rate 50
```

## 输出

- `/uav/altitude/barometer`
- `/uav/altitude/estimate`
- `/uav/altitude/error`
- `/uav/altitude/status`

指标包括气压高度 RMSE、加速度积分 RMSE、融合 RMSE 和最终误差。

![高度融合响应](../img/air/uav_altitude_sensor_fusion/altitude_response.png)

本次 50 Hz、12 s 合成数据的结果：

| 高度来源 | RMSE |
|---|---:|
| 原始气压高度 | 0.573 m |
| 加速度积分 | 24.140 m |
| 卡尔曼融合 | 0.478 m |

融合最终误差为 `0.426 m`。加速度积分因垂直加速度偏差持续累积而明显漂移，
融合结果则由气压高度持续修正。

## 测试

```bash
python3 test/test_altitude_filter.py
```

## 范围限制

当前使用合成传感器，不包含真实 IMU 温度漂移、气压计延迟或硬件故障。

## AI 使用声明

使用 Codex 辅助代码、测试和文档整理；算法参数、运行结果与提交内容由作者复核并负责。
