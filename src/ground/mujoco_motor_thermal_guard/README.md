# ROS 2 电机温度保护节点

本包订阅标准 `sensor_msgs/Temperature` 和 `geometry_msgs/Twist` 消息，根据连续测温估计短时温升，按迟滞阈值缩放速度并发布限幅后的 `Twist` 指令。不依赖仿真器、预测器或传感记录包。

## 话题

- 温度输入：`/motor/temperature`
- 速度输入：`/cmd_vel`
- 速度输出：`/thermal_guard/cmd_vel`
- 预测温度：`/thermal_guard/predicted_temperature`
- 速度比例：`/thermal_guard/speed_scale`

## 运行

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select mujoco_motor_thermal_guard
source install/setup.bash
ros2 launch mujoco_motor_thermal_guard thermal_guard.launch.py limit_c:=65 release_c:=60 horizon_s:=5
```

首次收到有效温度消息前，以及测温超过 `temperature_timeout_s`（默认 0.5 秒）后，每次收到速度输入时，输出指令的全部六个分量均为零。有效测温期间，线速度和角速度的全部分量使用同一个比例缩放。乱序或非有限温度样本会被丢弃，不刷新测温有效期。

`limit_c`、`release_c`、`horizon_s` 和 `temperature_timeout_s` 均按浮点数解析，命令中使用 `65` 或 `65.0` 都可以。预测时域默认 5 秒。

## 构建后测试

在已加载 ROS 2 Humble 的工作区根目录执行：

```bash
colcon test --packages-select mujoco_motor_thermal_guard --python-testing pytest --return-code-on-test-failure
colcon test-result --verbose
```

ROS 环境下共 5 项测试，覆盖温升外推、迟滞、非法阈值、时间戳乱序，以及六分量速度缩放与失去测温时的停止输出。若构建使用 `--merge-install`，测试命令也需加入该选项。普通 Python 环境可运行 3 项核心测试，2 项 ROS 适配测试由 pytest 跳过。
