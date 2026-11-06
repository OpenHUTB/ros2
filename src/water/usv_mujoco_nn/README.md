# usv_mujoco_nn

MuJoCo 水面无人艇（USV）仿真：二维激光雷达建图 + 神经网络局部规划 + ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- mujoco 3.2.3（`pip3 install mujoco`）
- numpy、matplotlib

## 安装

```bash
pip3 install mujoco numpy matplotlib
```

## 训练（生成专家示范 + 训练 MLP）

```bash
cd scripts
python3 train.py --episodes 40 --steps 260 --epochs 35
```

产物写入 `models/`：`planner_mlp.npz`、`norm_stats.npz`、`train_loss.npy`、`train_report.txt`。

## 评测

```bash
cd scripts
python3 evaluate.py --episodes 30
```

结果写入 `results/eval_report.json`。

## 可视化运行

```bash
cd scripts
python3 main.py --mode nn --save-gif demo.gif --duration 600   # 神经网络自主
python3 main.py --mode expert                                   # A*+纯跟踪专家
python3 main.py --mode manual                                   # 键盘：W/S 油门，A/D 转向，Q 退出
```

## ROS 节点

```bash
cd ~/usv_ws
catkin_make
source devel/setup.bash
roslaunch usv_mujoco_nn main.launch mode:=nn
```

话题：

| 话题 | 类型 | 方向 |
|---|---|---|
| `/usv/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/usv/odom` | `nav_msgs/Odometry` | 发布 |
| `/usv/map` | `nav_msgs/OccupancyGrid` | 发布 |
| `/usv/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 三自由度平面模型，未建模横摇/纵摇/垂荡；
- 推进器为一阶推力模型，无螺旋桨水动力与空化；
- 激光雷达为理想射线模型，无噪声与雨雾衰减；
- 神经网络为纯 NumPy 实现的 MLP，未使用 GPU 加速。
