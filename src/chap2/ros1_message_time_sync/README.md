# ROS Noetic 多传感器话题时间同步

本模块以 10 Hz 和 7 Hz 发布两路模拟传感器数据，使用
`message_filters.ApproximateTimeSynchronizer` 按消息时间戳进行近似配对，
并将配对结果发布到 `/sensors/sync_pair`。

## 运行环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- 依赖：`rospy`、`message_filters`、`sensor_msgs`、`std_msgs`

## 构建

```bash
mkdir -p ~/ros1_ws/src
cp -r <仓库路径>/src/chap2/ros1_message_time_sync ~/ros1_ws/src/
cd ~/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 启动

```bash
roslaunch ros1_message_time_sync main.launch
```

## 查看同步结果

```bash
rostopic hz /sensors/temperature
rostopic hz /sensors/humidity
rostopic hz /sensors/sync_pair
rostopic echo /sensors/sync_pair
```

默认配置下，同步输出约为 7 Hz，`timestamp_diff_sec` 应小于
`sync_slop_sec`。

## 参数

参数位于 `config/params.yaml`：

| 参数 | 默认值 | 说明 |
| --- | ---: | --- |
| `temperature_rate_hz` | `10.0` | 温度话题发布频率 |
| `humidity_rate_hz` | `7.0` | 湿度话题发布频率 |
| `sync_slop_sec` | `0.14` | 允许的最大时间差 |
| `sync_queue_size` | `30` | 同步队列长度 |
| `temperature_period_sec` | `20.0` | 模拟温度变化周期 |
| `humidity_period_sec` | `30.0` | 模拟湿度变化周期 |

## 测试

```bash
cd ~/ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_ros1_message_time_sync
catkin_test_results build/test_results/ros1_message_time_sync
```
