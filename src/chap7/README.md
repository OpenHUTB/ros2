# 第 7 章 阿克曼四轮小车运动控制代码

本目录包含两个 catkin 功能包：

| 功能包 | 内容 |
| --- | --- |
| `smartcar_description` | 整车 xacro 模型、ros_control 控制器配置、Gazebo 启动文件、Rviz 配置 |
| `smartcar_control` | 键盘遥控与阿克曼运动解算节点、前轮 TF 补全节点、离线运动学库与 Pure Pursuit 演示 |

## 使用方法

```bash
# 1. 将两个功能包复制到工作空间 src 目录
mkdir -p ~/catkin_ws/src
cp -r smartcar_description smartcar_control ~/catkin_ws/src/

# 2. 编译
cd ~/catkin_ws
catkin_make
source devel/setup.bash

# 3. 启动仿真环境（Gazebo + 模型 + 控制器）
roslaunch smartcar_description smartcar_gazebo.launch

# 4. 新开终端，补全前轮 TF
rosrun smartcar_control front_wheel_jsp.py

# 5. 新开终端，启动键盘遥控
roslaunch smartcar_control smartcar_gazebo_controller.launch

# 6. 新开终端，启动 Rviz
rviz -d $(rospack find smartcar_description)/config/smartcar.rviz
```

按键说明：`w/s` 加减速，`e/q` 前进时左/右转，`z/c` 后退时右/左转，`x` 转向回正，空格紧急停车。

## 离线运动学与路径跟踪（不需要 ROS / Gazebo）

`smartcar_control/scripts/ackermann_kinematics.py` 把 C++ 节点里的阿克曼解算抽成纯 Python，
并补上自行车模型与 Pure Pursuit 控制器；`pure_pursuit_demo.py` 可以在本机跑三种路径并输出指标。

```bash
cd src/chap7/smartcar_control
python3 scripts/pure_pursuit_demo.py                  # 圆 / 换道 / 双移线
python3 scripts/pure_pursuit_demo.py --path circle --lookahead 0.8 --speed 0.5
python3 -m unittest discover -s test                  # 36 个单元测试
```

实测（前视 0.5 m、车速 0.4 m/s）：圆周 R=1 m 的横向偏差 RMS 1.0 mm，正弦换道 2.1 mm，
双移线 13.5 mm，三种路径均无转向饱和。

配套文档见 `docs/chapter/chap7.md`。
