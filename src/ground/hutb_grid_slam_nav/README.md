# hutb_grid_slam_nav

在 Gazebo 仿真环境中，用**自研 2D 激光雷达栅格 SLAM**（不依赖 gmapping/cartographer）实现
**建图、定位、导航同步仿真**闭环：

- 建图：激光扫描经位姿变换累加为**贝叶斯 log-odds 占用栅格地图**；
- 定位：scan-to-map **迭代最近点（ICP）匹配**修正里程计位姿，发布 map→odom TF；
- 导航：将建好的地图交给 move_base（TrajectoryPlannerROS），发布目标点后机器人自动规划并移动。

## 运行环境

- Ubuntu 20.04 + ROS Noetic + Gazebo 11
- Python 3.8 + NumPy，依赖 `turtlebot3_description` / `turtlebot3_navigation` / `move_base` / `gazebo_ros` / `xacro`

## 运行步骤

```bash
# 1) 编译安装
cd ~/catkin_ws/src && git clone <本模块> hutb_grid_slam_nav
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# 2) 阶段一：建图（自研 SLAM，纯里程计积分）
roslaunch hutb_grid_slam_nav main.launch phase:=slam

# 3) 阶段二：导航（加载 data/map.pgm + move_base，目标点 (2,1)）
roslaunch hutb_grid_slam_nav main.launch phase:=nav
```

建图阶段可通过键盘（teleop_twist_keyboard）或任务2 模块的神经网络漫游驱动机器人移动。

## 算法要点

- 贝叶斯占用栅格（log-odds）：命中 +0.9、穿过 -0.4、先验 0，概率由 $p=1/(1+e^{-\ell})$ 反变换；
- scan-to-map ICP：最小化 $\sum_i \|q_i - R(\theta)p_i - t\|^2$，SVD 求解，单步修正限幅防发散。

## 目录结构

```
launch/main.launch          phase:=slam|nav 一键启动
scripts/main.py             自研栅格 SLAM 节点（建图/定位/发布 map->odom）
scripts/nav_goal.py         导航目标发布与性能统计
data/map.pgm + map.yaml     建图结果（800x800 @ 0.05 m/cell）
worlds/task3_world.world    自建障碍场景
config/slam.yaml            SLAM 参数
```
