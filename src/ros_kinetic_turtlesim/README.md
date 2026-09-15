# ros_kinetic_turtlesim — 小海龟自动画圆演示模块

`roslaunch` 一键启动小海龟仿真器，并自动生成第二只海龟 `turtle2`，控制它连续画出多个不同颜色的圆（花瓣效果）。配套详细教程见 [docs/ros_kinetic_turtlesim](../../docs/ros_kinetic_turtlesim/ros_kinetic_turtlesim.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 16.04（Kinetic）/ 20.04（Noetic）均可 |
| ROS | ROS Kinetic（Python 2.7）或 Noetic（Python 3），代码为 2/3 兼容写法 |
| 依赖功能包 | turtlesim、rospy、geometry_msgs（desktop-full 自带） |

## 运行步骤

**方式一：roslaunch 一键启动（推荐）**

```bash
# 1. 把本模块拷贝（或软链接）到 catkin 工作空间
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/ros_kinetic_turtlesim ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/ros_kinetic_turtlesim/main.py

# 2. 编译并加载环境
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 3. 一键启动（roslaunch 会自动拉起 roscore，无需单独启动）
roslaunch ros_kinetic_turtlesim main.launch
```

**方式二：手动三终端**

```bash
roscore
rosrun turtlesim turtlesim_node
python main.py          # 直接运行入口脚本
```

## 可调参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~petals` | 6 | 画几个圆（花瓣数） |
| `~linear_speed` | 1.5 | 线速度（m/s） |
| `~angular_speed` | 1.0 | 角速度（rad/s），圆半径 r = v/ω |

在 launch 中传参示例：`<node pkg="ros_kinetic_turtlesim" type="main.py" name="turtle_circle_drawer"><param name="petals" value="3"/></node>`

## 预期效果

仿真窗口中出现第二只小海龟 `turtle2`，以公共中心 (5.5, 5.5) 为交点依次画出 6 个半径 1.5 m、两两相扣的彩色花瓣圆；终端输出每个花瓣的画笔颜色和完成日志。
