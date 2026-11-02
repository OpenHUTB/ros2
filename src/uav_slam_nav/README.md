# uav_slam_nav - AirSim 无人机建图与自主导航

基于 AirSim 深度相机的占据栅格建图 + A* 路径规划 + 自主导航闭环（ROS 课程任务3）。

## 流程

1. map_build.py：无人机沿扫描轨迹飞行，前视深度相机反投影成 3D 点云，投影到 2D 占据栅格地图（200x200，0.25m/格）
2. planner.py：A* 算法在栅格地图上搜索无碰撞路径（8 邻域）
3. navigate.py：沿规划路径自动飞行至目标点
4. nav_track_plot.py：规划路径与实际飞行轨迹对比图

## 运行

    python3 main.py
    ros2 launch uav_slam_nav uav_slam_nav.launch.py

## 依赖

- Ubuntu 22.04 / ROS2 Humble
- AirSim Python API（连接宿主机 10.0.2.2）
- numpy / opencv-python / matplotlib
