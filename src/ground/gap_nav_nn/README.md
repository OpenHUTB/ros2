# gap_nav_nn

MuJoCo 狭窄通道通行：激光雷达开口辨识 + 神经网络选口穿行，ROS Noetic 封装。

## 环境

```bash
sudo apt install python3-numpy python3-matplotlib python3-pil python3-yaml python3-rospkg
pip3 install mujoco==3.2.3
```

## 构建

```bash
mkdir -p ~/gap_ws/src && cp -r gap_nav_nn ~/gap_ws/src/
cd ~/gap_ws && source /opt/ros/noetic/setup.bash && catkin_make
source devel/setup.bash
```

## 训练与评测

仓库已附带训练好的权重。

```bash
cd ~/gap_ws/src/gap_nav_nn/scripts
python3 train.py --episodes 90 --steps 600 --epochs 80 --noise 0.10
python3 evaluate.py --seeds 20
python3 make_figures.py
```

## 可视化

```bash
python3 main.py --mode expert --save-gif gap.gif --every 5
python3 main.py --mode nn
```

## ROS

```bash
roslaunch gap_nav_nn main.launch mode:=nn
```

`launch/main.launch` 用 `<rosparam>` 加载 `config/params.yaml` 到节点私有命名空间，
节点通过 `rospy.get_param("~xxx")` 读取。以下参数可在命令行覆盖：

```bash
roslaunch gap_nav_nn main.launch mode:=expert min_passable:=2.0 v_max:=0.6 lookahead:=0.9
```

| 话题 | 类型 | 方向 |
|---|---|---|
| `/gap/scan` | `sensor_msgs/LaserScan` | 发布 |
| `/gap/odom` | `nav_msgs/Odometry` | 发布 |
| `/gap/state` | `std_msgs/String` | 发布（approached/crossing/reached） |
| `/gap/cmd_vel` | `geometry_msgs/Twist` | 订阅（manual 模式） |

## 已知限制

- 平面三自由度模型，未建模轮胎侧滑；
- 激光雷达为理想射线模型；
- 隔墙与开口为固定几何，未测试可变布局；
- 专家直接读取开口真值，网络只有雷达。
