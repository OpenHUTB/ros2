# ros_topic_demo

最小的 ROS Noetic 话题可视化示例：节点以固定频率发布 `/odom`（`nav_msgs/Odometry`），
同时广播 `odom -> base_link` 的 TF，RViz 加载预置配置后就能看到一个小箭头沿半径
1 m 的圆匀速绕圈。

对应文档：[`docs/humanoid/ros_topic_demo/index.md`](../../../docs/humanoid/ros_topic_demo/index.md)

## 文件

```
src/ros_topic_demo/circle_path.py  圆形轨迹纯数学 + 离线自检 CLI（不需要 ROS）
scripts/trajectory_node.py         ROS 节点：参数、Odometry、TF
setup.py                           catkin_python_setup 安装 Python 包
launch/rviz_demo.launch            节点 + RViz 一键启动
config/rviz.rviz                   Fixed Frame=odom，已加 Odometry 显示
tests/test_circle_path.py          轨迹数学/采样/CSV 测试（不需要 ROS）
```

## 环境

- Ubuntu 20.04 + ROS Noetic（`catkin_make`）
- 离线自检和测试只需要 Python 3.7+，不需要 ROS

## 放进工作区并编译

```bash
mkdir -p ~/catkin_ws/src
cp -r ros_topic_demo ~/catkin_ws/src/
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

## 运行

```bash
roslaunch ros_topic_demo rviz_demo.launch
```

RViz 会自动加载 `config/rviz.rviz`：Fixed Frame 为 `odom`，Odometry 订阅 `/odom`。
终端里节点同时广播 `odom -> base_link`，所以不会出现 `No TF data`。

参数可以在 launch 里覆盖：

```bash
roslaunch ros_topic_demo rviz_demo.launch period:=5.0 radius:=1.5
```

| 参数 | 默认值 | 说明 |
|---|---|---|
| `radius` | 1.0 m | 圆半径 |
| `period` | 31.4 s | 一圈秒数（默认值与旧版速度一致） |
| `rate` | 10.0 Hz | 发布频率 |
| `frame_id` | `odom` | 固定坐标系 |
| `child_frame_id` | `base_link` | 机器人坐标系 |

节点按 `phase = 2*pi*t/period` 计算位姿，phase=0 时在 `(radius, 0)`，位姿（`pose` 的 yaw）沿逆时针切线，RViz 箭头因此也沿切线；`twist.linear.x = radius*omega`、`twist.angular.z = omega` 另外给出机体线速度/角速度。

## 无 ROS 自检

没有 ROS 时也能先验证轨迹数学（在包目录下执行）：

```bash
PYTHONPATH=src python3 -m ros_topic_demo.circle_path --period 10 --rate 10 --radius 1
```

输出采样数、首末点，并可选写 CSV：

```bash
PYTHONPATH=src python3 -m ros_topic_demo.circle_path --period 5 --rate 20 --csv circle.csv
```

编译并 `source devel/setup.bash` 后，`PYTHONPATH` 已包含包，可以直接
`python3 -m ros_topic_demo.circle_path ...`。

## 测试

```bash
python3 -m unittest discover -s tests -v
```

应显示 `Ran 10 tests ... OK`。测试覆盖角度归一化、起点、四分之一圆、半径不变、
切向朝向、采样数、完整一圈和 CSV 输出。

## 限制

- 这是 ROS 1 Noetic 示例，不包含 ROS 2 版本。
- 只发布 Odometry/TF，不发布 `JointState` 或机器人模型，RViz 里看到的是
  Odometry 箭头而不是完整机器人。
- 轨迹是理想圆，不含噪声、打滑或控制器。
