# water_rov_path_planning

基于热力图的洋流干扰下 ROV 自适应路径规划 ROS 2 模块。

## 功能概述

本模块面向遥控水下机器人（ROV）在复杂海底环境中的路径规划需求，提供三级处理：

1. **海底热力图构建**：将 50m×50m 海域离散化为 100×100 网格，根据障碍物位置生成风险热力图（0～1），风险随距离高斯衰减（衰减系数 0.15，障碍物影响半径 2 m）。风险在 2 m 处截断，截断处约 0.55，按阈值 0.7 实际封禁半径约 1.54 m；颜色由蓝（安全）经绿、黄过渡到红（危险）。
2. **洋流场建模**：支持均匀流、涡流、潮汐振荡、剪切流四种洋流模式，计算每个位置的流速向量，评估洋流对 ROV 运动的推力干扰。
3. **自适应路径规划**：基于 A* 算法，代价函数综合热力图风险、洋流推力（顺流奖励、逆流惩罚）和路径长度，自动避开高风险区，并在逆流段施加惩罚、顺流段给予奖励，规划出安全节能的路径。

模块附带演示节点，按固定随机种子生成海底障碍物和起终点，自动触发路径规划；障碍物发布后先等待一张时间戳更新的热力图，确认障碍物已被规划节点处理，再发布终点触发规划，避免 `obstacle` 与 `goal` 走不同话题时终点先到。

路径平滑采用 Jacobi 加权平均（每轮用上一轮结果整体更新，避免单向偏移），平滑后逐点、逐段中点以及段内按半格采样校验风险，任一点超过 `max_risk_threshold` 就整条回退到 A* 原始路径。

## 运行环境

- 操作系统：Ubuntu 20.04
- ROS 2 版本：Humble（20.04 官方无二进制包，需按官方文档从源码编译安装）
- 依赖：rclcpp、nav_msgs、geometry_msgs、std_msgs、sensor_msgs、cv_bridge、image_transport、OpenCV 4.x、Eigen3
- 演示动图渲染脚本依赖：rclpy、python3-opencv、python3-numpy、python3-pil

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
├── test/
│   └── test_core.cpp          # 热力图风险、顺逆流代价、参考流速、平滑回退单元测试
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
   ros2 topic echo /path_cost
   ros2 topic echo /planned_path
   ```

5. 可选：RViz2 查看热力图和路径：

   ```
   ros2 run rviz2 rviz2 -d install/water_rov_path_planning/share/water_rov_path_planning/rviz/path_planning.rviz
   ```

6. 单元测试（热力图风险衰减、坐标边界、潮汐相位、顺逆流代价、顺流奖励上限、参考流速归一化、平滑风险回退）：

   ```
   colcon test --packages-select water_rov_path_planning
   ```

7. 可选：离线渲染演示动图。前置条件是 launch（demo_node + planning_node）正在运行并已在发布 `/heatmap`、`/ocean_current`、`/planned_path`；另开一个终端执行：

   ```
   source install/setup.bash
   ros2 run water_rov_path_planning render_demo.py
   ```

   脚本收齐三个话题后渲染 24 帧三联动图，输出到 `~/demo_rov_path_planning.gif`。

## 运行结果

在 Ubuntu 20.04 + ROS 2 Humble 虚拟机上实测（环境静态，多次重规划结果一致）：

| 场景 | 路径点数 | 路径长度 | 加权累计代价 |
| --- | --- | --- | --- |
| 涡流（默认） | 81 | 40.48 m | 37.25 |
| 均匀流（+x 顺流） | 81 | 40.00 m | 16.00 |
| 零流速基线（magnitude = 0） | 81 | 40.00 m | 40.00 |

代价为平滑后最终路径上的加权累计值：三项权重全程无量纲，都按“单位距离代价 × 本段米数”累加，洋流项用无量纲对齐度 `(û·v)/current_ref_speed` 归一化，因此代价按米制口径累计、量纲为米（数值上可与路径长度直接比较），两者是同一条平滑后路径的指标（零流速且沿途无风险时代价恰好等于长度）。重算风险按半格步长采样整段取最大值，而 A* 搜索按目标格取风险，故发布代价可能略大于搜索累计的 g 值。

默认涡流场景下路径不再完全沿直线：长度 40.48 m 略长于直线 40.00 m，代价 37.25 低于零流速基线 40.00，说明规划器利用涡流场中部分顺流段降低了代价。改用均匀流（`current_pattern:=uniform`、`current_magnitude:=0.4`、`current_direction:=0.0`）时路径几何不变（40.00 m），加权累计代价由 40.00 降到 16.00：每米顺流奖励 k·w_c·a = 0.5×3.0×0.4 = 0.6，即每米代价 1−0.6 = 0.4，40 m 共 16.0，洋流补偿项生效。参数不在 launch 中重复定义，临时切换可用 `ros2 run ... --ros-args -p` 覆盖，或直接修改 `config/params.yaml`。

## 话题说明

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/obstacle` | `geometry_msgs/PointStamped` | 输入 | 障碍物位置（米） |
| `/start` | `geometry_msgs/PointStamped` | 输入 | 起点位置 |
| `/goal` | `geometry_msgs/PointStamped` | 输入 | 终点位置（触发规划） |
| `/planned_path` | `nav_msgs/Path` | 输出 | 规划路径 |
| `/heatmap` | `sensor_msgs/Image` | 输出 | 热力图可视化（蓝=安全，红=危险） |
| `/ocean_current` | `sensor_msgs/Image` | 输出 | 洋流场向量图 |
| `/path_cost` | `std_msgs/Float32MultiArray` | 输出 | [加权累计代价, 路径长度(m), 路径点数]，代价与长度均为平滑后最终路径的指标 |

代码中使用相对话题名（不带前导 `/`），默认解析到根命名空间，两个节点因此共享同一组名字；加 `__ns` 后可在同机并行运行多组实例，互不抢占。

## 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `grid_size` | 100 | 热力图网格边长（格子数） |
| `resolution` | 0.5 | 每个格子对应实际尺寸（米） |
| `obstacle_radius` | 2.0 | 障碍物影响半径（米） |
| `risk_decay` | 0.15 | 风险高斯衰减系数 α |
| `risk_weight` | 5.0 | 热力图风险权重（须 ≥ 0） |
| `current_weight` | 3.0 | 洋流权重（须 ≥ 0，作用于无量纲对齐度 (û·v)/`current_ref_speed`） |
| `current_compensation` | 0.5 | 顺流奖励系数 k（0～1，且须 k·w_c·magnitude/v_ref < length_weight，保证顺流边代价不为 0） |
| `current_ref_speed` | 1.0 | 参考流速（m/s），把 û·v 归一化到无量纲 |
| `length_weight` | 1.0 | 路径长度权重 |
| `max_iterations` | 100000 | A* 最大迭代次数 |
| `max_risk_threshold` | 0.7 | 风险阈值，超过则不可通行 |
| `current_pattern` | vortex | 洋流模式（uniform/vortex/tidal/shear，非法值报错退出） |
| `current_magnitude` | 0.4 | 基础洋流流速（m/s，须 ≤ `current_ref_speed`） |
| `current_direction` | 0.0 | 均匀流方向（弧度，须满足 \|current_direction\| ≤ 2π） |
| `vortex_x` / `vortex_y` | 25.0 / 25.0 | 涡流中心坐标（米，须落在 [0, grid_size×resolution] 内） |
| `vortex_radius` | 15.0 | 涡流影响半径（米） |
| `tidal_period` | 60.0 | 潮汐周期（秒） |

演示节点参数定义在 `config/params.yaml` 的 `rov_demo_node` 段：`world_size`、`num_obstacles`、`obstacle_seed`、`start_x/start_y`、`goal_x/goal_y`。两类参数都只在 yaml 中定义一处，launch 只加载该文件；非法参数会在启动时报错并以非零退出码结束。

## 已知限制

- 热力图采用 \(R=\exp(-\alpha d^2)\) 并在影响半径处截断，边界处风险约 0.55 直接归零，代价场存在台阶，A* 可能沿截断边界行进。若要彻底消除，可将核函数改为归一化的 \(R=\max(\exp(-\alpha d^2)-\exp(-\alpha r^2),0)/(1-\exp(-\alpha r^2))\)，但会改变封禁半径与现有演示数据。
- 启发函数已改为米制欧氏距离乘以 `length_weight`，与 g 值的长度项同量纲；但顺流边代价最低可到 0（低于长度项），h 仍可能高估实际剩余代价，不满足可采纳性，A* 返回近优解而非全局最优。
- 使用合成演示数据（随机障碍物 + 解析洋流场），未接入真实海底声呐或多波束数据。
- 洋流场为稳态解析模型；潮汐模式有周期变化，规划时按当前时间计算，但未做真实海况下的验证。
- 路径为全局一次性规划，演示节点每 5 秒触发一次重规划作为示意，未实现随洋流变化的连续动态重规划。
