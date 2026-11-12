# vio_time_offset_estimation

视觉-惯性时间偏移的互相关估计：陀螺角速率与图像平面角速率之间存在固定延迟 δ，
在 [-80, 80] ms 的候选滞后上做归一化互相关，再用二次插值细化峰值，
最后给出补偿前后与"真值补偿"的残差对比。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select vio_time_offset_estimation
source install/setup.bash
ros2 launch vio_time_offset_estimation main.launch.py offset:=0.025
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/vio/gyro` | `sensor_msgs/Imu` | 输出（仿真） | 200 Hz 陀螺 z 轴 |
| `/vio/visual_rate` | `std_msgs/Float64` | 输出（仿真） | 延时后的视觉角速率 |
| `/vio/time_offset` | `std_msgs/String` | 输出（估计） | 偏移、相关系数、补偿前后残差（JSON） |

## 离线实验

```bash
cd src/ground/vio_time_offset_estimation
python3 main.py --out results --figure-dir ../../../docs/img/ground/vio_time_offset_estimation
python3 -m unittest discover -s tests -v
```

## 主要结果（200 次试验，真值 25 ms）

| 指标 | 数值 |
|---|---|
| 偏移误差均值 | +0.59 ms |
| 偏移误差 RMSE | 0.89 ms |
| 95% 绝对误差 | 1.48 ms |
| 最大绝对误差 | 1.70 ms |

| 残差 | 数值 |
|---|---|
| 补偿前（按 0 ms） | 0.1911 |
| 补偿后 | **0.0907** |
| 真值补偿（oracle） | 0.0907 |
| 改善 | **52.5 %** |

详见 `docs/ground/vio_time_offset_estimation.md`。
