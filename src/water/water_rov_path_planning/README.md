# water_rov_path_planning

基于热力图的洋流干扰下 ROV 自适应路径规划 ROS 2 模块。

## 功能概述

本模块面向遥控水下机器人（ROV）在复杂海底环境中的路径规划需求，提供三级处理：

1. **海底热力图构建**：将 50m×50m 海域离散化为 100×100 网格，根据障碍物位置生成风险热力图（0～1），风险随距离高斯衰减（衰减系数 0.15，障碍物影响半径 2 m）。
2. **洋流场建模**：支持均匀流、涡流、潮汐振荡、剪切流四种洋流模式，计算每个位置的流速向量，评估洋流对 ROV 运动的推力干扰。
3. **自适应路径规划**：基于 A* 算法，代价函数综合热力图风险、洋流推力（顺流奖励、逆流惩罚）和路径长度，自动避开高风险区，并在逆流段施加惩罚、顺流段给予奖励，规划出安全节能的路径。

模块附带演示节点，按固定随机种子生成海底障碍物和起终点，自动触发路径规划。

## 运行环境

- 操作系统：Ubuntu 20.04
- ROS 2 版本：Humble
- 依赖：rclcpp、nav_msgs、geometry_msgs、std_msgs、sensor_msgs、cv_bridge、image_transport、OpenCV 4.x、Eigen3
- 演示动图渲染脚本依赖：python3-opencv、python3-numpy、python3-pil

## 核心文件结构

```
src/water/water_rov_path_planning/
├── include/water_rov_path_planning/
│   ├── heatmap.hpp            # 热力图：障碍物风险扩散
│   ├── ocean_current.hpp      # 洋流场：四种洋流模式
│   └── path_planner.hpp       # A* 自适应路径规划器
├── src/
│   ├── heatmap.cpp            # 热力图实现
│   ├── ocean_current.cpp      # 洋流场实现
│   ├── path_planner.cpp       # A* + 洋流补偿 + 路径平滑
│   ├── planning_node.cpp      # 主节点：环境更新 + 路径规划
│   └── demo_node.cpp          # 演示节点：模拟海底环境
├── launch/
│   └── main.launch.py
├── config/
│   └── params.yaml
├── rviz/
│   └── path_planning.rviz
├── scripts/
│   └── render_demo.py         # 订阅节点输出并离线渲染演示动图
├── CMakeLists.txt
├── package.xml
└── README.md
```

## 快速运行

1. 创建工作空间并拷贝模块：

   ```
   mkdir -p ~/ros2_ws/src
   cp -r ~/ros2/src/water/water_rov_path_planning ~/ros2_ws/src/
   cd ~/ros2_ws
   ```

2. 安装依赖并编译：

   ```
   rosdep install --from-paths src --ignore-src -r -y
   colcon build --packages-select water_rov_path_planning
   source install/setup.bash
   ```

   若系统存在多个 OpenCV 版本导致找不到 OpenCV 配置，编译时显式指定：

   ```
   colcon build --packages-select water_rov_path_planning --cmake-args -DOpenCV_DIR=/usr/lib/x86_64-linux-gnu/cmake/opencv4
   ```

3. 启动演示（自动生成障碍物 + 起点终点 + 路径规划）：

   ```
   ros2 launch water_rov_path_planning main.launch.py
   ```

4. 查看规划结果：

   ```
   ros2 topic echo /rov/path_cost
   ros2 topic echo /rov/planned_path
   ```

5. 可选：RViz2 查看热力图和路径：

   ```
   ros2 run rviz2 rviz2 -d install/water_rov_path_planning/share/water_rov_path_planning/rviz/path_planning.rviz
   ```

## 话题说明

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/rov/obstacle` | `geometry_msgs/PointStamped` | 输入 | 障碍物位置（米） |
| `/rov/start` | `geometry_msgs/PointStamped` | 输入 | 起点位置 |
| `/rov/goal` | `geometry_msgs/PointStamped` | 输入 | 终点位置（触发规划） |
| `/rov/planned_path` | `nav_msgs/Path` | 输出 | 规划路径 |
| `/rov/heatmap` | `sensor_msgs/Image` | 输出 | 热力图可视化（蓝=安全，红=危险） |
| `/rov/ocean_current` | `sensor_msgs/Image` | 输出 | 洋流场向量图 |
| `/rov/path_cost` | `std_msgs/Float32MultiArray` | 输出 | [总代价, 路径长度(m), 路径点数] |

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `grid_size` | 100 | 热力图网格边长 |
| `resolution` | 0.5 | 每个格子对应实际尺寸（米） |
| `risk_weight` | 5.0 | 热力图风险权重 |
| `current_weight` | 3.0 | 洋流权重 |
| `current_compensation` | 0.5 | 顺流奖励系数（0～1） |
| `max_risk_threshold` | 0.7 | 风险阈值，超过则不可通行 |
| `current_pattern` | vortex | 洋流模式（uniform/vortex/tidal/shear） |
| `current_magnitude` | 0.4 | 基础洋流流速（m/s） |
| `current_direction` | 0.0 | 均匀流方向（弧度） |
