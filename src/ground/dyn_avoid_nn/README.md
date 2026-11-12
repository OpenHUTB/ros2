# dyn_avoid_nn

MuJoCo 动态障碍物避让：激光雷达帧间差分 + 神经网络时机决策，ROS Noetic 封装。

## 环境

- Ubuntu 20.04 / ROS Noetic / Python 3.8
- MuJoCo 3.2.3、NumPy、Matplotlib

```bash
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg
pip3 install mujoco==3.2.3
```

## 构建

```bash
mkdir -p ~/dyn_ws/src && cp -r dyn_avoid_nn ~/dyn_ws/src/
cd ~/dyn_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

## 训练与评测

仓库已附带训练好的权重。

```bash
cd ~/dyn_ws/src/dyn_avoid_nn/scripts
python3 train.py --episodes 40 --steps 700 --epochs 50 --noise 0.06
python3 evaluate.py --seeds 20
python3 make_figures.py
```

## 可视化

```bash
python3 main.py --mode expert --save-gif dyn.gif --every 5
python3 main.py --mode nn
```

## 测试

```bash
cd ~/dyn_ws/src/dyn_avoid_nn
python3 -m unittest discover -s test -v      # 8 项，不需要 MuJoCo
```

`test/test_dyn_avoid.py` 覆盖网络前向/保存加载、路径生成与专家的让行行为；
场地常量已抽到 `scripts/consts.py`，因此这组测试在没有 mujoco 的机器上也能跑。

## ROS

```bash
roslaunch dyn_avoid_nn main.launch mode:=nn
```

| 话题 | 类型 | 方向 |
|---|---|---|
| `/dyn/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/dyn/odom` | `nav_msgs/Odometry` | 发布 |
| `/dyn/state` | `std_msgs/String` | 发布（cruising/reached） |
| `/dyn/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 平面三自由度模型，未建模轮胎侧滑；
- 激光雷达为理想射线模型，无噪声；
- 移动障碍为匀速往复运动，未考虑突然变向；
- 专家使用障碍的真实速度（监督信号），部署时只有雷达帧。
