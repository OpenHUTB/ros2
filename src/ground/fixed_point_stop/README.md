# fixed_point_stop 定点停车

一个 TurtleBot3 定点停车的 ROS1 包。机器人在 Gazebo 仿真里从起点驶向指定目标点，
到达目标后自动停车并打印停点误差。正前方离障碍太近时会先停车、原地右转躲一下再继续走。

## 环境

* Ubuntu 20.04，ROS 1（Noetic）
* Python 3（3.8 以上都可以）
* 用到了 `rospy`、`sensor_msgs`、`nav_msgs`、`geometry_msgs`、`tf`
* 在 TurtleBot3 Gazebo 仿真里跑，需要 `/odom`、`/scan`、`/cmd_vel` 这些话题

## 编译运行

把包放进工作空间编译：

```bash
cd ~/catkin_ws/src
cp -r <仓库路径>/src/ground/fixed_point_stop ./
cd ~/catkin_ws && catkin_make
source ~/catkin_ws/devel/setup.bash
```

先启动 TurtleBot3 仿真（burger 模型）：

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

再启动这个包：

```bash
roslaunch fixed_point_stop fixed_point_stop.launch
```

直接 `rosrun` 也可以：

```bash
rosrun fixed_point_stop main.py
```

## 目标点

默认目标点 (1.5, 1.5)（单位米），可通过 launch 参数修改：

```bash
roslaunch fixed_point_stop fixed_point_stop.launch target_x:=2.0 target_y:=1.0
```

到达目标点（容差 0.1 m 内）自动停车，并打印实际位置与误差。

## 运行效果

仿真里跑的录屏：

![运行效果](../../../docs/img/ground/fixed_point_stop.gif)

## 验证记录

在自己的机器上实际构建、启动、运行过，环境和结果如下：

* 系统：Ubuntu 20.04.6 LTS（focal）
* ROS：Noetic
* Python：3.8.10
* catkin：0.8.12
* 仿真：Gazebo 11.15.1，TurtleBot3 burger 模型

`catkin_make` 编译通过；`roslaunch turtlebot3_gazebo turtlebot3_world.launch` 正常启动仿真；
`roslaunch fixed_point_stop fixed_point_stop.launch` 启动节点后，小车从起点驶向目标点
并自动停车。节点终端输出（以 `target_x:=3.0` 为例）：

```
定点停车启动，目标点 (3.00, 0.00)，容差 0.10 m
已到达目标点，停车！实际位置 (2.75, 0.01)，误差 0.250 m
```

## 声明

`main.py` 是作业的原始代码，其余文件（`package.xml`、`CMakeLists.txt`、`launch`、`README.md`、文档页）在整理时用到了大模型辅助。我已逐项核对，对提交内容负责。
