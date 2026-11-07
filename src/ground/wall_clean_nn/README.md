# wall_clean_nn

MuJoCo 室内清扫车沿墙清扫：激光雷达 + 神经网络贴墙跟随，ROS Noetic 封装。

## 环境

- Ubuntu 20.04
- ROS Noetic（`ros-noetic-ros-base`）
- Python 3.8
- MuJoCo 3.2.3、NumPy、Matplotlib

## 安装

```bash
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg
pip3 install mujoco==3.2.3
```

## 构建（catkin 工作空间）

```bash
mkdir -p ~/wall_ws/src && cp -r wall_clean_nn ~/wall_ws/src/
cd ~/wall_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

节点安装到 `devel/lib/wall_clean_nn/`，运行期脚本安装到
`devel/share/wall_clean_nn/scripts/`，`launch`、`config`、`models` 安装到
`devel/share/wall_clean_nn/`；节点通过 `rospkg` 定位包路径后导入脚本。

## 训练与评测

仓库已附带训练好的权重，可直接跳过训练。

```bash
cd ~/wall_ws/src/wall_clean_nn/scripts
python3 train.py --episodes 60 --steps 1200 --epochs 60 --noise 0.12   # 约 4 分钟
python3 evaluate.py --seeds 10
python3 make_figures.py
```

## 可视化

```bash
cd ~/wall_ws/src/wall_clean_nn/scripts
python3 main.py --mode expert --save-gif wall.gif --every 40 --dpi 70
python3 main.py --mode nn
```

## ROS

```bash
roslaunch wall_clean_nn main.launch mode:=nn
```

| 话题 | 类型 | 方向 |
|---|---|---|
| `/cleaner/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/cleaner/odom` | `nav_msgs/Odometry` | 发布 |
| `/cleaner/coverage` | `std_msgs/Float32` | 发布（沿墙覆盖率） |
| `/cleaner/state` | `std_msgs/String` | 发布（approach/following/done） |
| `/cleaner/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 三自由度平面模型，未建模轮子打滑与悬挂；
- 激光雷达为理想射线模型，无噪声；
- 家具为静止矩形，未考虑动态障碍；
- 神经网络只在训练分布（房间内随机起点）内验证，未测试未建模的房间形状。
