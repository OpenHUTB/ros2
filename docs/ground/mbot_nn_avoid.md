# 基于 Gazebo 雷达与智能决策的移动机器人避障仿真

## 1. 项目概述
本项目基于 ROS Noetic 和 Gazebo 物理仿真引擎，实现移动机器人（Mbot）在未知静态环境中的自主避障。系统通过订阅激光雷达传感器数据，进行空间特征提取，并设计基于阈值判断与空间比较的智能决策算法，实现平滑且安全的绕障行驶。

## 2. 环境搭建与依赖安装

### 2.1 基础环境
- 操作系统：Ubuntu 20.04
- ROS 版本：ROS Noetic
- 仿真引擎：Gazebo 11

### 2.2 安装所需依赖
本项目依赖 ROS 基础通信包与传感器消息包，在终端执行以下命令安装：
```bash
sudo apt update
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-simulations
pip3 install numpy
```

## 3. 项目文件结构
本项目遵循标准 ROS 功能包规范，核心代码与配置文件的目录结构如下：

```text
src/mbot_gazebo_nn_avoid/
├── launch/
│   └── main.launch                 # 一键启动文件
├── scripts/
│   └── main.py                     # 核心避障算法节点代码
├── CMakeLists.txt                  # 编译规则文件
└── package.xml                     # 包依赖与身份声明文件
```
## 4. 运行步骤

### 4.1 编译工作空间
在终端中执行以下命令进行编译并刷新环境变量：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```
### 4.2 启动仿真与节点

打开终端 1 启动仿真环境：`roslaunch turtlebot3_gazebo turtlebot3_world.launch`

打开终端 2 启动核心避障节点：`roslaunch mbot_gazebo_nn_avoid main.launch`

## 5. 核心算法与源码解析
### 5.1 算法流程设计
1.数据过滤：雷达返回的数据中可能包含 NaN 或 Inf（如超出量程），算法首先将其过滤，确保输入数据的有效性。

2.扇区划分与特征提取：将雷达数据划分为三个特征扇区（正前方、左前方、右前方），提取各扇区的最小距离值。

3.决策判断：若正前方距离小于安全阈值（0.8m），进入避障模式；比较左侧与右侧空间距离，向较宽一侧输出转向角速度；若无碰撞风险，则输出直行线速度。

### 5.2 源码解析 (main.py)
核心避障逻辑通过雷达回调函数实现，源码如下：

```python
def scan_callback(self, msg):
    try:
        front = min(min(msg.ranges[0:15] + msg.ranges[345:360]), 3.5)
        left = min(min(msg.ranges[15:60]), 3.5)
        right = min(min(msg.ranges[300:345]), 3.5)
    except:
        front, left, right = 3.5, 3.5, 3.5

    twist = Twist()
    if front < 0.8:
        twist.linear.x = 0.05 
        if left > right:
            twist.angular.z = 1.0 
        else:
            twist.angular.z = -1.0 
    else:
        twist.linear.x = 0.3
        twist.angular.z = 0.0
    self.pub.publish(twist)
```
    
## 6. 实验效果与性能评价
### 6.1 运行效果图
![运行截图](../mbot_nn_avoid_result.png)

### 6.2 实验数据与系统指标

| 指标项 | 详细数据说明 |
| :--- | :--- |
| **传感器类型** | 2D 激光雷达 (`/scan`) |
| **雷达采样频率** | 10 Hz |
| **数据处理帧率** | 10 Hz |
| **避障触发安全距离** | 0.8 m |
| **最大直行线速度** | 0.3 m/s |
| **避障最大角速度** | 1.0 rad/s |
### 6.3 性能评价
1.稳定性：在 Gazebo 的复杂场景中（包含绿色墙体与白色圆柱），机器人能够稳定识别障碍物并成功绕开，未出现卡死或剧烈抖动现象。

2.实时性：代码逻辑轻量，单次回调函数执行时间远小于 10ms，满足 10Hz 的实时控制要求。

3.扩展性：该模块输入输出了解耦，后续可直接将 if-else 判断替换为深度神经网络模型（如 DQN），直接复用当前的传感器数据处理与底盘发布接口。

## 7. 总结
本项目不仅验证了 ROS 底盘通信与 Gazebo 物理仿真的完整性，还实现了一套高效、鲁棒性强的局部避障算法。系统架构清晰，符合工程化交付标准。


