# usv_mujoco_nn

MuJoCo 水面无人艇（USV）仿真：二维激光雷达建图 + 神经网络局部规划 + ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- MuJoCo 3.2.3、NumPy 1.17.4、Matplotlib 3.1.2

## 安装

系统依赖（catkin 构建时 `rosdep` 可解析的部分）：

```bash
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg
```

**MuJoCo 本体通过 pip 安装，不属于 rosdep 可解析依赖**，必须单独执行：

```bash
pip3 install mujoco==3.2.3
```

安装后自检：

```bash
python3 -c "import mujoco, numpy, matplotlib; print(mujoco.__version__)"
```

预期输出 `3.2.3`。

## 构建（catkin 工作空间）

本包需要放在 catkin 工作空间的 `src/` 下：

```bash
mkdir -p ~/usv_ws/src
cp -r usv_mujoco_nn ~/usv_ws/src/          # 或在 src/ 下建立软链接
cd ~/usv_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

`catkin_make` 会把节点安装到 `devel/lib/usv_mujoco_nn/`，把运行期脚本安装到
`devel/share/usv_mujoco_nn/scripts/`，把 `launch`、`config`、`models` 安装到
`devel/share/usv_mujoco_nn/`。节点通过 `rospkg` 定位包路径后导入这些脚本，
因此不依赖源码树的相对路径。

## 训练（生成专家示范 + 训练 MLP）

```bash
cd ~/usv_ws/src/usv_mujoco_nn/scripts
python3 train.py --episodes 50 --steps 320 --epochs 45 --seed 0 --noise 0.15
```

产物写入 `models/`：`planner_mlp.npz`、`norm_stats.npz`、`train_loss.npy`、`train_report.txt`。
本机实测：15140 样本，验证集 MSE 0.0669，线速度 MAE 0.031 m/s，角速度 MAE 0.054 rad/s。

## 评测

```bash
cd ~/usv_ws/src/usv_mujoco_nn/scripts
python3 evaluate.py --episodes 30 --seed 123
```

结果写入 `results/eval_report.json`。

## 可视化运行

```bash
cd ~/usv_ws/src/usv_mujoco_nn/scripts
python3 main.py --mode nn --save-gif demo.gif --duration 250 --seed 4 --every 3 --dpi 80
python3 main.py --mode expert
python3 main.py --mode manual          # 键盘：W/S 油门，A/D 转向，Q 退出
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
