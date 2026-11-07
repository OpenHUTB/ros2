# usv_docking_nn

MuJoCo 无人艇自主靠泊：激光雷达 + 神经网络末段对准，ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- MuJoCo 3.2.3、NumPy、Matplotlib

## 安装

```bash
# 系统依赖（rosdep 可解析部分）
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg

# MuJoCo 不在 rosdep 中，必须用 pip 安装并锁定版本
pip3 install mujoco==3.2.3
```

## 构建（catkin 工作空间）

```bash
mkdir -p ~/usv_ws/src && cp -r usv_docking_nn ~/usv_ws/src/
cd ~/usv_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

`catkin_make` 会把节点安装到 `devel/lib/usv_docking_nn/`，把运行期脚本安装到
`devel/share/usv_docking_nn/scripts/`，把 `launch`、`config`、`models` 安装到
`devel/share/usv_docking_nn/`。节点通过 `rospkg` 定位包路径后导入这些脚本。

## 训练与评测

仓库已附带训练好的权重，可直接跳过训练。

```bash
cd ~/usv_ws/src/usv_docking_nn/scripts
python3 train.py --episodes 60 --epochs 40       # 约 3 分钟
python3 evaluate.py --seeds 20
```

产物：`models/dock_planner.npz`、`models/dock_norm.npz`、`results/dock_eval.json`。

## 可视化

```bash
cd ~/usv_ws/src/usv_docking_nn/scripts
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
