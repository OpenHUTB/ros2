# parking_path_planner

垂直车位自动泊车的**路径规划**模块：用 Hybrid A\* 在车位周围搜索一条"前进摆位 + 倒车入库"的可行轨迹，输出 `nav_msgs/Path` 与可视化标记，可在 RViz2 中直接查看。

规划器只用 Python 标准库实现（不依赖 numpy），**没有 ROS2 环境也能离线跑出轨迹图**，便于在没有仿真器的机器上复现结果。

效果：起点在车道内，先前进摆位，再一把倒车入库，全程 15.60 m、换挡 1 次，终点误差 0.25 m 以内。

## 运行环境

| 项目 | 要求 |
|------|------|
| 操作系统 | Ubuntu 22.04（Windows / macOS 可只跑离线模式） |
| ROS | ROS 2 Humble |
| Python | 3.10（仅用标准库） |
| 可视化 | RViz2 |

```bash
sudo apt update
sudo apt install ros-humble-nav-msgs ros-humble-geometry-msgs \
                 ros-humble-visualization-msgs ros-humble-std-srvs
```

## 目录结构

```
parking_path_planner/
├── main.py                        # 模块入口：有 ROS 启动节点，无 ROS 自动离线
├── package.xml
├── setup.py / setup.cfg
├── launch/main.launch.py          # launch 启动入口
├── rviz/parking_path_planner.rviz # RViz2 配置
├── parking_path_planner/
│   ├── planner.py                 # Hybrid A* 与几何工具
│   ├── ros_node.py                # ROS2 节点
│   └── viz.py                     # SVG 轨迹渲染
└── tests/test_planner.py          # 不变量测试
```

## 运行步骤

### 1. 编译

```bash
cd ~/ros2_ws/src
git clone https://github.com/OpenHUTB/ros2.git
cd ~/ros2_ws
colcon build --packages-select parking_path_planner
source install/setup.bash
```

### 2. 启动（推荐）

```bash
ros2 launch parking_path_planner main.launch.py
```

会同时拉起规划节点和 RViz2，自动完成一次规划。RViz2 中可以看到：绿色车位框、红色目标位姿箭头、蓝色前进段与橙色倒车段。

也可以用模块入口启动（等价）：

```bash
cd src/ground/parking_path_planner
python3 main.py
```

### 3. 无 ROS 环境（离线模式）

```bash
python3 main.py --offline --output results
```

会在 `results/` 下生成 `parking_path.svg`（轨迹图）与 `parking_waypoints.json`（含每个路径点的坐标、朝向与挡位 D/R）。

### 4. 验证

```bash
# 查看发布的路径（poses 数组）
ros2 topic echo /parking_path --field poses

# 重新规划
ros2 service call /plan_parking std_srvs/srv/Trigger {}

# 单元测试（无需 ROS）
python3 -m unittest discover -s tests -v
```

## 参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| `frame_id` | `map` | 坐标系 |
| `slot_width` | `2.70` | 车位宽（m） |
| `slot_depth` | `5.50` | 车位深（m） |
| `slot_center_x` / `slot_center_y` | `0.0` / `2.75` | 车位中心（世界坐标） |
| `slot_heading_deg` | `0.0` | 车位朝向（度） |
| `start_x` / `start_y` / `start_yaw_deg` | `-7.0` / `-4.2` / `0.0` | 起始位姿 |
| `wheelbase` | `2.70` | 轴距（m） |
| `vehicle_width` / `vehicle_length` | `1.80` / `4.50` | 车体尺寸（m） |
| `max_steer_deg` | `35.0` | 前轮最大转角（度） |
| `step` | `0.40` | 搜索单步弧长（m） |
| `goal_xy_tolerance` | `0.35` | 目标位置容差（m） |
| `neighbors_parked` | `true` | 车位两侧是否视为已停车 |
| `auto_plan` | `true` | 启动后是否自动规划一次 |

示例：

```bash
ros2 launch parking_path_planner main.launch.py start_x:=-6.0 start_y:=-3.6 rviz:=false
```

## 话题与服务

| 名称 | 类型 | 方向 |
|------|------|------|
| `/parking_path` | `nav_msgs/Path` | 发布 |
| `/parking_markers` | `visualization_msgs/MarkerArray` | 发布 |
| `/plan_parking` | `std_srvs/Trigger` | 服务 |

## 实现要点

* **Hybrid A\***：状态为 `(x, y, yaw)`，按 0.25 m / 5° 离散去重；动作原语为 5 档转角 × {前进, 倒车}，单步弧长 0.40 m，用自行车模型积分。这样得到的轨迹天然满足最小转弯半径，并且会自动产生换挡序列。
* **精确碰撞检测**：车体矩形做 Sutherland-Hodgman 半平面裁剪后判断多边形面积是否为空，而不是采样点检查，因此不会漏掉"车身斜插进邻车位"的情况。
* **换挡惩罚**：代价函数中加入 1.2 的换挡惩罚，抑制无意义的前后挪动，实测能把 9 次换挡的抖动轨迹收敛成 1 次。

## 已知限制

* 只规划轨迹，**不含跟踪控制**：把 `/parking_path` 交给控制器（纯跟踪 / MPC）执行后，终点误差还能再小一些。
* 目标位姿容差为 0.35 m / 10°，规划终点与车位中心存在约 0.25 m 偏差，由控制部分修正。
* 车位两侧按"已停车"处理；若测试场地空旷，可设 `neighbors_parked:=false`。
