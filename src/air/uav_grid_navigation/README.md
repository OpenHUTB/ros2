# MuJoCo 无人机栅格激光建图与路径重规划

本模块在 MuJoCo 平面场景中控制无人机模型沿 A* 航点移动。控制循环读取 MuJoCo 位姿，使用 `mj_ray` 获取 180 束虚拟激光测距，更新占据栅格，并在每个航点重新规划路线。

## 环境与运行

支持 Python 3.10 及以上版本。安装运行和测试依赖后，执行导航并运行测试：

```bash
cd src/air/uav_grid_navigation
python -m pip install -r requirements.txt
python -m pip install -r requirements-dev.txt
python main.py
python -m pytest -q test_navigation.py
```

Windows 可双击 `launch.bat` 启动同一入口。

从仓库根目录运行测试：

```bash
python -m pytest -q src/air/uav_grid_navigation/test_navigation.py
```

默认场景为 24×16 栅格，包含三组矩形障碍物。MuJoCo 的位置执行器逐段跟踪 A* 航点；每到达一个格点，程序读取新位姿、采集激光测距并更新地图，再计算后续路线。

运行结果写入 `output/`：`navigation_map.png` 为最终占据栅格与实际行进路线，`navigation_run.gif` 展示地图和航点进度，`trajectory.json` 保存 MuJoCo 位姿轨迹，`navigation_metrics.json` 记录目标到达、内部地图覆盖率、扫描次数和重规划次数。覆盖率统计 308 个内部格点；外圈 76 个边界格作为已知障碍初始化，并在指标中单独记录。

English: The MuJoCo planar UAV follows A* waypoints using simulated position feedback. At each waypoint, 180 virtual lidar rays update the occupancy grid and the planner computes the next route. Map coverage is measured over the 308 interior cells; the 76 boundary cells are initialized as occupied and reported separately. Outputs include the final map, an animated route, pose telemetry, and navigation metrics.
