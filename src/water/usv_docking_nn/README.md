# usv_docking_nn

MuJoCo 无人艇自主靠泊：激光雷达 + 神经网络末段对准，ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- MuJoCo 3.2.3（`pip3 install mujoco`）
- numpy、matplotlib

## 训练与评测

```bash
cd scripts
python3 train.py --episodes 60 --epochs 40       # 约 3 分钟
python3 evaluate.py --seeds 20
```

产物：`models/dock_planner.npz`、`models/dock_norm.npz`、`results/dock_eval.json`。

## 可视化

```bash
python3 main.py --mode nn --save-gif dock.gif --every 6
python3 main.py --mode expert
```

## ROS

```bash
cd ~/usv_ws && catkin_make && source devel/setup.bash
roslaunch usv_docking_nn main.launch mode:=nn
```

| 话题 | 类型 | 方向 |
|---|---|---|
| `/usv/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/usv/odom` | `nav_msgs/Odometry` | 发布 |
| `/usv/berth` | `std_msgs/Float32MultiArray` | 发布（横向/艏向/纵向偏差） |
| `/usv/state` | `std_msgs/String` | 发布（approach/align/docked） |
| `/usv/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 三自由度平面模型，未建模横摇、纵摇与侧推器；
- 激光雷达为理想射线模型，无噪声与雨雾衰减；
- 泊位为静止刚性结构，未考虑风、流等环境扰动；
- 神经网络为纯 NumPy 的 MLP，未使用 GPU 加速。
