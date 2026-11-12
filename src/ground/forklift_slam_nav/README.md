# MuJoCo 叉车同步建图与导航

本包在 MuJoCo 中运行采用平面运动学积分的移动底盘，并通过 MuJoCo 射线查询生成 360 束激光扫描。单一 launch 同时启动仿真、`slam_toolbox` 同步建图节点和 Nav2。仿真器发布 `odom -> base_link` 与 `base_link -> laser` TF；SLAM 根据 `/scan` 和 `/odom` 发布 `/map` 与 `map -> odom`。Nav2 使用在线地图和激光扫描规划并避障。

## 构建与启动

在仓库根目录先构建本项目现有 MuJoCo/Humble 基础镜像，再构建导航镜像：

```bash
docker build -t mujoco-forklift-pallet-docking:latest src/ground/mujoco_forklift_pallet_docking
docker build -f src/ground/forklift_slam_nav/Dockerfile -t forklift-slam-nav:humble .
docker run --rm -it --name forklift-slam-nav forklift-slam-nav:humble
```

基础镜像需包含 ROS 2 Humble；导航镜像按 `requirements.txt` 安装 MuJoCo 3.12.0 和 NumPy，并安装 Humble `slam_toolbox`、Nav2 与 `colcon`。在本地 Humble 工作区安装相同依赖并执行 `colcon build --symlink-install` 后，可用 `ros2 launch forklift_slam_nav main.launch.py` 启动。

地图建立后可通过 Nav2 的 `NavigateToPose` action 下发目标：

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 2.0}, orientation: {w: 1.0}}}}"
```

## 离线单元测试

平面位姿推进与 freejoint 航向换算位于纯 Python 模块，可在不安装 ROS 2 或 MuJoCo 的环境中运行测试：

```bash
cd src/ground/forklift_slam_nav
python3 -m unittest discover -s test -v
```

## 话题与坐标系

- `/scan`：`sensor_msgs/LaserScan`，360 个水平射线，坐标系 `laser`。
- `/odom`：`nav_msgs/Odometry`，里程计坐标系 `odom`，底盘坐标系 `base_link`。
- `/tf`：仿真器发布 `odom -> base_link`；SLAM 发布 `map -> odom`；静态变换为 `base_link -> laser`。
- `/map`：同步 `slam_toolbox` 发布的占据栅格地图。
- `/navigate_to_pose`：Nav2 导航 action。

## 启动入口与模型高度

规范启动文件为 `launch/main.launch.py`（包含原 `launch/bringup.launch.py`，原命令仍可使用），通过 `ros2 launch forklift_slam_nav main.launch.py` 调用。仿真可执行入口 `mujoco_robot` 对应 `forklift_slam_nav.simulator:main`。`odom -> base_link` 的高度为 0.22 m；`base_link -> laser` 的相对高度为 0.04 m，激光在模型中的绝对高度为 0.26 m。

源码根目录的 `main.py` 调用安装后的具名模块 `forklift_slam_nav.main`，该模块统一转发至仿真节点；安装后可用 `ros2 run forklift_slam_nav main` 启动仿真节点，原 `mujoco_robot` 可执行入口继续兼容。

## 运行环境与复现条件

全栈运行环境为 Ubuntu 22.04、ROS 2 Humble、MuJoCo 3.12.0 和 NumPy 1.24 或更新版本。导航镜像依赖先行构建的 `mujoco-forklift-pallet-docking:latest` 基础镜像。启动统一 launch 后，需等待 SLAM 发布在线地图、地图覆盖目标区域且 Nav2 导航服务就绪，再发送地图坐标系下的 `(0, 2)` 目标。地图 157×156 与目标返回 `SUCCEEDED` 为该容器环境的一次运行记录；地图尺寸随实际观测范围变化。
