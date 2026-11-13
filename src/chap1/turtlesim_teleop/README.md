# turtlesim_teleop — 小海龟键盘遥控模块（第一章实验）

读取方向键或 WASD，向 `/turtle1/cmd_vel` 发布 `geometry_msgs/Twist` 速度指令，
控制 turtlesim 小海龟前后移动、左右转向。配套教程见
[docs/chapter/chap1/turtlesim_teleop.md](../../docs/chapter/chap1/turtlesim_teleop.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（VMware 虚拟机） |
| ROS | ROS Noetic（Python 3）；代码同时兼容 Ubuntu 16.04 + Kinetic（Python 2.7） |
| 依赖功能包 | turtlesim、rospy、geometry_msgs（desktop-full 自带） |
| 其他 | roslaunch 一键启动时键盘节点需要 xterm（`sudo apt install -y xterm`） |

## 运行步骤

**方式一：roslaunch 一键启动（推荐）**

```bash
# 1. 把本模块拷贝到 catkin 工作空间
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/chap1/turtlesim_teleop ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/turtlesim_teleop/main.py

# 2. 编译并加载环境
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 3. 一键启动：turtlesim 窗口 + 键盘遥控 xterm 窗口
roslaunch turtlesim_teleop main.launch
```

若没有安装 xterm，可用 `roslaunch turtlesim_teleop main.launch use_xterm:=false`，
再另开一个终端执行 `rosrun turtlesim_teleop main.py`。

**方式二：手动三终端**

```bash
# 终端 1
roscore
# 终端 2
rosrun turtlesim turtlesim_node
# 终端 3（键盘输入必须在这个前台终端里）
rosrun turtlesim_teleop main.py
```

## 按键说明

| 按键 | 功能 |
| ---- | ---- |
| 方向键 / WASD | 前进、后退、左转、右转 |
| 空格 | 立即停车 |
| `+` / `-` | 加速 / 减速（步长 0.5，范围 0.5～5.0） |
| `q` 或 `Ctrl+C` | 退出；退出前自动发布零速度停车 |

## 可调参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~linear_speed` | 2.0 | 线速度（m/s） |
| `~angular_speed` | 2.0 | 角速度（rad/s） |

在 launch 中传参示例：

```xml
<node pkg="turtlesim_teleop" type="main.py" name="turtlesim_teleop" output="screen">
  <param name="linear_speed" value="3.0"/>
</node>
```

## 预期效果

turtlesim 窗口中的小海龟按方向键移动，终端实时打印当前速度；
松开按键后海龟保持最后一次速度指令，按空格或 `q` 退出时发布零速度停车。
