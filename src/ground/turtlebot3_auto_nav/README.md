# turtlebot3_auto_nav

TurtleBot3 (Burger) 在 Gazebo 仿真环境下的自主导航功能包，使用 AMCL 定位、DWA 局部规划，`map_server` 加载静态地图，通过 RViz 的 2D Nav Goal 下发目标点。

完整说明（原理、参数、常见报错）见 [docs/ground/turtlebot3_auto_nav/README.md](../../../docs/ground/turtlebot3_auto_nav/README.md)。

## 环境依赖

- Ubuntu 20.04 + ROS Noetic
- `ros-noetic-turtlebot3`、`ros-noetic-turtlebot3-gazebo`、`ros-noetic-turtlebot3-bringup`
- `ros-noetic-dwa-local-planner`

```bash
sudo apt install ros-noetic-turtlebot3 ros-noetic-turtlebot3-gazebo \
                 ros-noetic-turtlebot3-bringup ros-noetic-dwa-local-planner
```

## 编译

```bash
cd ~/nav_ws
catkin_make
source devel/setup.bash
```

## 启动

终端 1，启动 Gazebo 仿真世界：

```bash
export TURTLEBOT3_MODEL=burger
roslaunch turtlebot3_gazebo turtlebot3_world.launch
```

终端 2，启动导航与 RViz（等 Gazebo 加载完成后再执行）：

```bash
export TURTLEBOT3_MODEL=burger
cd ~/nav_ws
source devel/setup.bash
roslaunch turtlebot3_auto_nav main.launch
```

启动后在 RViz 中先用 "2D Pose Estimate" 给出初始位姿，再用 "2D Nav Goal" 指定目标点。

## 目录结构

```text
turtlebot3_auto_nav/
├── launch/main.launch
├── maps/{map.pgm, map.yaml}
├── param/*.yaml
├── rviz/turtlebot3_nav.rviz
├── CMakeLists.txt
└── package.xml
```
