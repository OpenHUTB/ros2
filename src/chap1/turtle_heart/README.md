# 小海龟画爱心实验

## 修复目标

turtlesim 默认处于落笔状态。原程序直接前往爱心首点，会在默认位置与爱心顶部凹口之间
留下一条多余连线。本模块在前往首点前先抬笔，到达起点后再落笔，从第二个轨迹点开始
绘制闭合爱心。

## 运行环境

| 项目 | 版本或说明 |
|---|---|
| 操作系统 | Ubuntu 20.04 |
| ROS | ROS Noetic |
| Python | 3.8 |
| 仿真器 | turtlesim |

## 安装与启动

将 `turtle_heart` 功能包放入 catkin 工作空间的 `src` 目录，然后构建：

```bash
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

一键启动 ROS 主节点、turtlesim 和画爱心节点：

```bash
roslaunch turtle_heart main.launch
```

也可以不使用 launch，分别启动各进程：

```bash
# 终端 1
roscore

# 终端 2
rosrun turtlesim turtlesim_node

# 终端 3
rosrun turtle_heart main.py
```

程序先抬笔移动到爱心首点，再落笔绘制完整轮廓。完成后抬笔并使小海龟停止。

## 无 ROS 回归检查

在仓库根目录执行：

```bash
python3 -m pytest src/chap1/turtle_heart/tests/test_pen_sequence.py -q
```

该检查验证画笔顺序为“抬笔 → 移动到首点 → 落笔 → 绘制其余 60 个点”。
