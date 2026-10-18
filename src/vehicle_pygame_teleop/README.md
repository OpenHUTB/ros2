# vehicle_pygame_teleop

基于 pygame 的 ROS 车辆键盘遥操作功能包，**内置俯视二维驾驶模拟**：键盘输入实时驱动画面中的车辆运动，同时以 30Hz 向 `/cmd_vel` 发布 `geometry_msgs/Twist` 速度指令，无需启动外部仿真器即可验证手动控制效果。

## 功能

- 方向键 / WASD 控制车辆油门、刹车/倒车与左右转向，加减速过程平滑
- 空格键手刹急停；ESC、关闭窗口或 Ctrl+C 退出时自动发布零速度，车辆可靠停车
- 内置俯视二维模拟：网格地图、车身与车头朝向随动、实时线速度/角速度显示
- 30Hz 稳定输出 `/cmd_vel`，接口格式与 Carla 等仿真器的遥控接口一致

## 运行环境

- Ubuntu 20.04 + ROS Noetic（Python 3）
- pygame

## 1. 安装依赖

```bash
sudo apt update
sudo apt install -y python3-pygame
# 或使用 pip 安装：
pip3 install pygame
```

## 2. 放入工作空间并构建

将整个 `vehicle_pygame_teleop` 文件夹放入 ROS 工作空间的 `src` 目录（例如 `~/catkin_ws/src`），然后编译：

```bash
cd ~/catkin_ws
catkin_make
```

## 3. 加载工作空间环境

每个新终端使用本功能包前，先 source 工作空间环境（只需执行一次，建议写入 `~/.bashrc`）：

```bash
source ~/catkin_ws/devel/setup.bash
```

## 4. 运行

```bash
roslaunch vehicle_pygame_teleop main.launch
```

启动后弹出二维模拟窗口，**点击窗口使其获得键盘焦点**，即可开始控制车辆。

## 按键说明

| 按键 | 功能 |
| --- | --- |
| W 或 ↑ | 油门加速 |
| S 或 ↓ | 前进时刹车，静止时倒车 |
| A 或 ← | 左转（`angular.z` 为正） |
| D 或 → | 右转（`angular.z` 为负） |
| 空格 | 手刹急停 |
| ESC / 关闭窗口 | 发布零速度后退出 |

## 话题接口

- 发布话题：`/cmd_vel`（`geometry_msgs/Twist`）
  - `linear.x`：纵向线速度，前进为正、倒车为负（m/s）
  - `angular.z`：偏航角速度，左转为正、右转为负（rad/s）
- 发布频率：30Hz

## 目录结构

```
vehicle_pygame_teleop/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── main.launch
└── scripts/
    └── vehicle_pygame_teleop.py
```
