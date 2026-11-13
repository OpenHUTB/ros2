# AirSim 无人机激光雷达感知与扫描校正

本页对应 `src/air/uav_nn_perception_navigation` 的激光雷达感知部分。模块在 AirSim 1.8.1 与 ROS Noetic 中读取 `lidar_1` 点云，发布 `/uav/lidar/scan`（`sensor_msgs/LaserScan`）和 `/uav/lidar/features`（12 维扇区距离）。控制、安全层、模型和评估文件沿用主分支现有实现。

## 坐标与扫描

AirSim 的 `SensorLocalFrame` 遵循 NED：x 向前、y 向右、z 向下；ROS 扫描 0° 向前、逆时针向左为正。因此生成扫描时对 `atan2(y, x)` 取负号。给定视场角 `fov` 和 `N` 个采样格，角度从 `-fov/2` 起，间隔 `fov/N`；最后一格角度为 `-fov/2 + (N-1)×fov/N`，与 `LaserScan.angle_max` 一致。

随包雷达配置的水平视场为 -90°～+90°，所以 `scan_fov` 默认 180°。配置中的雷达位置 `Z=-0.5` 是 NED 坐标，在 ROS `base_link → laser` 静态变换中对应 `z=+0.5 m`。

每帧点云在累积前记录采样时的三维位置和四元数姿态；发布扫描前，将各帧点转换到当前机体系，补偿平移、roll、pitch、yaw 和雷达安装偏移。随包配置的雷达无额外旋转，安装偏移为 `(0, 0, -0.5)`（NED）。实现按帧补偿，尚未对单帧内的每个点单独去畸变；快速运动时应减速并检查地图质量。未命中的方向填最大量程只表示本次没有回波，不代表该方向安全。

## 安装与启动

按模块 [README](https://github.com/OpenHUTB/ros2/blob/master/src/air/uav_nn_perception_navigation/README.md) 安装依赖与 AirSim 官方 PythonClient，把 `config/settings.json` 放入 AirSim 配置目录，再在 ROS 工作空间编译：

```bash
catkin_make
source devel/setup.bash
roslaunch uav_nn_perception_navigation scan.launch airsim_ip:=<主机IP>
rostopic hz /uav/lidar/scan
rostopic echo -n 1 /uav/lidar/scan
```

`scan.launch` 启动桥接节点、感知节点及 `base_link → laser` 静态 TF，默认不自动起飞。验证时检查 `header.frame_id=laser`、`angle_min/angle_max/angle_increment`、话题频率和左右已知障碍的方向。离线单元测试覆盖左右约定、末端角度、平移与三轴旋转补偿；它不能代替 AirSim 中的实际扫描检查。

## 使用范围

这次改动针对感知链路；完整神经网络控制、SLAM 和端到端演示的运行说明见后续模块文档。若机器人姿态变化较快，单帧内运动畸变仍可能影响建图精度。

大模型使用声明：本页面的整理与代码复核使用了大模型辅助，提交者对内容负全部责任。
