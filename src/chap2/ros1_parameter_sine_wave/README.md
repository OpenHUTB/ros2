# ROS Noetic 运行参数热更新：正弦信号

本模块使用 `dynamic_reconfigure` 在线调整正弦信号的幅值、频率、相位和
消息发布频率。参数修改后无需重启节点。

## 运行环境

- Ubuntu 20.04
- ROS Noetic
- Python 3.8
- 依赖：`rospy`、`std_msgs`、`dynamic_reconfigure`

## 构建

```bash
mkdir -p ~/ros1_ws/src
cp -r <仓库路径>/src/chap2/ros1_parameter_sine_wave ~/ros1_ws/src/
cd ~/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 启动

```bash
roslaunch ros1_parameter_sine_wave main.launch
```

## 查看结果

```bash
rostopic echo /sine_wave/value
rostopic hz /sine_wave/value
rosrun dynamic_reconfigure dynparam get /sine_wave_node
```

## 在线修改参数

```bash
rosrun dynamic_reconfigure dynparam set /sine_wave_node amplitude 3.0
rosrun dynamic_reconfigure dynparam set /sine_wave_node frequency_hz 1.0
rosrun dynamic_reconfigure dynparam set /sine_wave_node publish_rate_hz 50.0
rosrun dynamic_reconfigure dynparam set /sine_wave_node enabled false
```

修改 `frequency_hz` 时会按新参数重新计算正弦相位，切换瞬间可能出现相位跳变；
本实验不作为连续调频建模。

## 参数

| 参数 | 默认值 | 范围 | 说明 |
| --- | ---: | ---: | --- |
| `amplitude` | `1.0` | `0.0` 到 `10.0` | 正弦信号幅值 |
| `frequency_hz` | `0.25` | `0.0` 到 `5.0` | 正弦信号频率 |
| `publish_rate_hz` | `20.0` | `1.0` 到 `100.0` | 消息发布频率 |
| `phase_rad` | `0.0` | `-pi` 到 `pi` | 初相位，单位弧度 |
| `enabled` | `true` | 布尔值 | 是否发布消息 |

## 测试

```bash
cd ~/ros1_ws
source /opt/ros/noetic/setup.bash
source devel/setup.bash
catkin_make run_tests_ros1_parameter_sine_wave
catkin_test_results build/test_results/ros1_parameter_sine_wave
```
