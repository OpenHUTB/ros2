# octree_uav_3d_pathfinding

空域载具（无人机）的八叉树三维寻路模块。
从传感器点云构建八叉树占用地图，在八叉树上执行 A* 全局寻路，
并进行安全裕度处理与路径平滑，最终实现无人机的三维路径跟踪。

## 运行环境

| 类别 | 项目 | 版本 |
|---|---|---|
| 操作系统 | Ubuntu | 20.04.6 LTS |
| ROS | 发行版 | Noetic (ROS 1, 1.16.0) |
| 仿真器 | Gazebo | 11.13.0 |
| 三维地图 | octomap | 1.9.8 |
| 点云库 | PCL | 1.10 |
| 构建工具 | catkin | 0.8.10 |
| Python | 版本 | 3.8 |

## 编译

在终端中依次执行：

    mkdir -p ~/uav_ws/src
    cd ~/uav_ws/src && catkin_init_workspace
    cd ~/uav_ws && catkin_make
    source ~/uav_ws/devel/setup.bash

## 运行

一条命令启动仿真器与主入口节点：

    roslaunch octree_uav_3d_pathfinding main.launch

带图形界面运行：

    roslaunch octree_uav_3d_pathfinding main.launch gui:=true

延长自检时长：

    roslaunch octree_uav_3d_pathfinding main.launch duration:=20

### 运行效果

模块启动后，终端会输出运行环境信息与三项链路检查结果：

![模块运行效果](docs/run_verify.png)

---

## 主节点做了什么

scripts/main.py 是模块入口，用于验证三条关键链路：

1. 仿真时钟桥接：订阅 /clock，确认 Gazebo 仿真时间已发布到 ROS
2. Gazebo 服务：调用 /gazebo/get_world_properties，确认仿真器在线
3. 话题通信：发布心跳话题 /uav_status

节点运行指定时长后自动退出，launch 随之结束，终端可直接看到完整输出。

## 参数配置

参数位于 config/params.yaml，由 main.launch 载入。

| 参数 | 默认值 | 说明 |
|---|---|---|
| check_duration | 10.0 | 自检运行时长（秒） |
| clock_timeout | 30.0 | 等待 /clock 的超时时间（秒） |
| service_timeout | 30.0 | 等待 Gazebo 服务的超时时间（秒） |
| status_topic | /uav_status | 心跳话题名 |

## 常见问题

| 现象 | 原因与处理 |
|---|---|
| Unable to communicate with master | ROS 主节点未启动，用 roslaunch 会自动启动 |
| /clock 等待超时 | Gazebo 未启动，或直接运行 gzserver 未拉起 ROS 主节点 |
| catkin_make: command not found | 未载入 ROS 环境，执行 source /opt/ros/noetic/setup.bash |

## 后续计划

| 提交 | 内容 |
|---|---|
| 1 | 模块骨架、launch 入口、环境自检（当前） |
| 2 | 质点四旋翼模型与传感器 |
| 3 | 点云转八叉树占用地图 |
| 4 | 八叉树上的 A* 全局寻路 |
| 5 | 安全裕度与路径平滑 |
| 6 | 路径跟踪与闭环飞行 |

## 参考

- OctoMap 官网：https://octomap.github.io/
- Hornung et al., OctoMap: An Efficient Probabilistic 3D Mapping Framework Based on Octrees, Autonomous Robots, 2013
