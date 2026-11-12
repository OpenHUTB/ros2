# flight_path_simulator

预设航线飞行模拟器 - 按时间表播放一段多旋翼巡航飞行，逐帧记录数据并生成可视化图表。

## 与仓库已有模块的区别

| 模块 | 类型 | 输入方式 | 用途 |
|------|------|----------|------|
| `drone_teleop` | 实时遥控 | 键盘输入 | 终端键盘遥控飞行 |
| `drone_ros_teleop` | 实时遥控 | ROS 消息 | 解耦键盘遥控 |
| `uav_keyboard_control` | 实时遥控 | 键盘 + ROS | ROS 节点键盘控制 |
| **本模块** | **离线模拟** | **预设时间表** | **快速生成可复现的飞行数据集与可视化** |

本模块为纯 Python 离线脚本，不依赖 ROS、仿真器或输入设备，用于快速生成教学/测试用的飞行数据。

## 为什么不做 ROS 封装

本模块产物是静态的 CSV 与图表，不发布/订阅任何 ROS 话题，也没有需要持续运行的节点，因此没有 `package.xml`、`launch/` 或话题接口可提供。若强行包成 ROS 节点，反而需要先起 `roscore`、等节点退出后再取文件，与"一条命令即可复现数据集"的定位相悖。

若后续需要与 ROS 生态联动，可以在此基础上加一层话题发布节点（把逐帧记录改为 `nav_msgs/Odometry` 或 `sensor_msgs/Imu` 输出），届时再补标准的 ament_python 包结构。

## 运行

```bash
python3 flight_path_simulator.py                  # 输出到当前目录
python3 flight_path_simulator.py --outdir results/ # 输出到指定目录
```

依赖：numpy, matplotlib

## 输出

- `flight_log.csv` — 逐帧飞行数据（600 帧 @ 20Hz）
- `trajectory.png` — XY 平面飞行轨迹俯视图
- `altitude.png` — 高度随时间变化曲线
- `velocity.png` — 三轴速度分量曲线
- `state_timeline.png` — 飞行状态时间线

## 测试

```bash
python3 test_simulator.py
```
