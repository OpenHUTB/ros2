# turtle_sim_experiment — 小海龟自动画花瓣演示模块（第一章扩展实验）

`roslaunch` 一键启动小海龟仿真器，控制小海龟画出 6 个两两相扣的彩色花瓣圆。配套详细教程见 [docs/chapter/turtle_sim_experiment](../../docs/chapter/turtle_sim_experiment.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（主环境，已适配 Noetic） |
| ROS | ROS Noetic（Python 3）；同时兼容 Ubuntu 16.04 + Kinetic（Python 2.7），代码为 2/3 兼容写法 |
| 依赖功能包 | turtlesim、rospy、geometry_msgs（desktop-full 自带） |

## 运行步骤

**方式一：roslaunch 一键启动（推荐）**

```bash
# 1. 把本模块拷贝（或软链接）到 catkin 工作空间
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/chap1/turtle_sim_experiment ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/turtle_sim_experiment/main.py

# 2. 编译并加载环境
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 3. 一键启动（roslaunch 会自动拉起 roscore，无需单独启动）
roslaunch turtle_sim_experiment main.launch
```

**方式二：手动三终端**

```bash
roscore
rosrun turtlesim turtlesim_node
python3 main.py          # 直接运行入口脚本（16.04 下为 python main.py）
```

## 可调参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~petals` | 6 | 画几个圆（花瓣数） |
| `~linear_speed` | 1.5 | 线速度（m/s） |
| `~angular_speed` | 1.0 | 角速度（rad/s），圆半径 r = v/ω |

在 launch 中传参示例：`<node pkg="turtle_sim_experiment" type="main.py" name="turtle_circle_drawer"><param name="petals" value="3"/></node>`

## 预期效果

仿真窗口中小海龟以公共中心 (5.5, 5.5) 为交点依次画出 6 个半径 1.5 m、两两相扣的彩色花瓣圆；终端输出每个花瓣的画笔颜色和完成日志。
