# rooftop_landing

**屋顶降落**：无人机用机载 LiDAR 检测水平屋顶平面，在屋顶轮廓上取**最大内切圆圆心**
（离屋顶边缘最远的点）作为安全降落点，自主飞抵其上方并垂直降落在屋顶上。

本模块不做仿真器连接：连仿真器、NED↔ENU 坐标换算、速度限幅与指令超时保护
全部由 [`carlair_ros_bridge`](../carlair_ros_bridge/README.md) 承担，本模块只订阅
感知数据、下发目标点与速度，因此与键盘控制、规划器等模块共用同一条
`/uav/cmd_vel` 通道，互不耦合。

---

## 1. 运行环境

本项目沿用作业环境的「**模拟器在宿主 Windows、ROS 在客户机 Ubuntu**」分工。

| 位置 | 项目 | 版本 |
|---|---|---|
| 宿主 | 操作系统 | Windows 11 |
| 宿主 | 模拟器 | AirSim 1.8.1（OpenHUTB 低空模拟器发行版） |
| 客户机 | 操作系统 | Ubuntu 20.04.6 LTS |
| 客户机 | ROS | Noetic（ROS 1）1.16.0 |
| 客户机 | 构建工具 | catkin 0.8.10 |
| 客户机 | Python | 3.8（系统自带即可） |
| 客户机 | Python 依赖 | **numpy**（`pip3 install numpy`） |

> 本模块**不需要** OpenCV、PCL、octomap 等重型依赖：点云解析与几何运算全部用 numpy 实现。

---

## 2. 依赖安装

```bash
# 客户机（Ubuntu 虚拟机）
sudo apt update
sudo apt install -y python3-numpy        # 或 pip3 install numpy
```

前置的模拟器启动、防火墙放行、`carlair_ros_bridge` 编译等步骤，见仓库文档
[空域载具环境配置](../../../docs/air/setup_and_connect.md) 与
[无人机与 ROS 的桥接模块](../../../docs/air/carlair_ros_bridge.md)。

---

## 3. 编译

```bash
# 把本包放进 catkin 工作空间
cd ~/uav_ws/src            # 没有工作空间就先 mkdir -p ~/uav_ws/src && cd ~/uav_ws/src && catkin_init_workspace
cp -r <本包> air/          # 目录结构为 src/air/rooftop_landing
cd ~/uav_ws
catkin_make
source devel/setup.bash
```

编译成功的标志是日志最后出现 `Built target`，且没有红色报错。

---

## 4. 运行

### 4.1 宿主侧先启动仿真器

按 [无人机与 ROS 的桥接模块](../../../docs/air/carlair_ros_bridge.md) 的 **2.1 节**
（`SetupEnv.bat` → `TestEnv.bat` → `StartCarlaAir.bat Town01 --no-traffic --quality Low`）
启动 CarlaAir 仿真器，等终端输出 `CarlaAir is ready.`，此时 `41451` 端口就绪。
**仿真器窗口不要最小化**，否则 AirSim 取相机会阻塞。

### 4.2 客户机侧启动桥接

```bash
source /opt/ros/noetic/setup.bash
source ~/uav_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch
```

自检打印 `自检结果: 5/5 通过` 说明点云、里程计、相机链路都通了。

### 4.3 启动屋顶降落（作业要求的 launch 启动）

```bash
# 另开一个终端
source ~/uav_ws/devel/setup.bash
roslaunch rooftop_landing main.launch
```

节点会依次输出：

```
[INFO] [rooftop_landing] 已启动，状态=DETECT，高度约定 z_up=False
[INFO] [rooftop_landing] odom 原始高度 z=-1.20 -> 换算高度 1.20 m（z_up=False）
[INFO] [rooftop_landing] 点云 9xxx 点，检测到 3 个候选水平面
[INFO] [rooftop_landing] 选中屋顶：高度 15.xx m，面积 28x.x m^2，降落点 (20.1, 15.2)，内切圆半径 8.7 m
[INFO] [rooftop_landing] 下发目标点 (20.10, 15.20, -20.30)，即屋顶上方 5.0 m
[INFO] [rooftop_landing] 状态 APPROACH -> DESCEND
[INFO] [rooftop_landing] 下降中：离屋顶 3.42 m
[INFO] [rooftop_landing] 已降落：离屋顶 0.21 m <= 容差 0.35 m
[INFO] [rooftop_landing] 状态 DESCEND -> LANDED
```

### 4.4 前台直接运行（等价方式）

```bash
rosrun rooftop_landing main.py
```

### 4.5 不开 RViz / 手动触发

```bash
roslaunch rooftop_landing main.launch use_rviz:=false
roslaunch rooftop_landing main.launch autostart:=false
# 另开终端手动开始一次任务
rostopic pub -1 /rooftop/start std_msgs/Bool "data: true"
```

---

## 5. 参数

参数文件 `config/rooftop_landing.yaml`，全部可用 roslaunch 参数覆盖。

| 参数 | 默认 | 说明 |
|---|---|---|
| `frame/z_up` | `false` | 高度分量是否「上为正」。**实测桥接层为下为正**，故默认 false |
| `perception/max_tilt_deg` | 15.0 | 平面水平性阈值（度） |
| `perception/plane_dist_thresh` | 0.15 | RANSAC 内点距离阈值 (m) |
| `perception/min_plane_inliers` | 120 | 一个平面最少点数 |
| `perception/min_roof_area` | 8.0 | 屋顶最小面积 (m²) |
| `perception/min_height` | 0.5 | 平面高度下限 (m)，用于滤掉地面 |
| `perception/min_landing_radius` | 0.6 | 降落点最小安全半径 (m) |
| `perception/prefer` | `highest` | 目标屋顶策略：`highest` / `closest` / `largest` |
| `approach/safe_altitude` | 5.0 | 高于屋顶的安全高度 (m) |
| `approach/goal_timeout` | 60.0 | 等待 `GOAL_DONE` 超时 (s) |
| `descend/speed` | 0.5 | 垂直下降速度 (m/s) |
| `descend/tolerance` | 0.35 | 判定触地的高度容差 (m) |
| `descend/timeout` | 60.0 | 下降超时 (s) |

示例：

```bash
roslaunch rooftop_landing main.launch safe_altitude:=6.0 prefer:=closest
roslaunch rooftop_landing main.launch z_up:=true          # 若高度是 ENU 上为正
```

---

## 6. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/lidar/points` | `sensor_msgs/PointCloud2` | 激光点云（世界系，来自桥接层） |
| 订阅 | `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度 |
| 订阅 | `/uav/status` | `std_msgs/String` | 桥接状态机 `READY/HOVER/VELOCITY/GOAL/GOAL_DONE` |
| 订阅 | `/rooftop/start` | `std_msgs/Bool` | 手动开始一次任务（`autostart:=false` 时用） |
| 发布 | `/uav/goal` | `geometry_msgs/Point` | 屋顶降落点上方的目标点 |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 降落阶段的垂直速度指令（机体系） |
| 发布 | `/rooftop/landing_point` | `geometry_msgs/Point` | 选中的降落点（便于外部记录） |
| 发布 | `/rooftop/markers` | `visualization_msgs/MarkerArray` | 屋顶轮廓线与降落点可视化 |

---

## 7. 算法说明

```
点云(/lidar/points)
   │  ① 解析 PointCloud2 → (N,3)，滤掉 NaN/Inf       frames.pointcloud2_to_xyz
   ▼
   │  ② RANSAC 拟合平面 + 内点最小二乘精修          rooftop_perception.fit_plane_ransac
   ▼
   │  ③ 只保留水平面（倾角 ≤ max_tilt_deg）          plane_tilt_deg
   ▼
   │  ④ 平面内点投影 → 2D 凸包作为屋顶轮廓          convex_hull_2d
   ▼
   │  ⑤ polylabel 求最大内切圆圆心 = 降落点          polylabel
   ▼
   │  ⑥ 按高度/半径打分选最优屋顶                   select_landing_target
   ▼
目标点(/uav/goal) → 桥接层飞抵悬停 → /uav/status = GOAL_DONE
   ▼
垂直速度(/uav/cmd_vel) → 高度接近屋顶 → LANDED
```

**为什么用最大内切圆圆心而不是质心**：屋顶往往不是规则矩形，质心可能落在天窗、
通风管或靠近边缘的位置；内切圆圆心是**离屋顶边界最远**的点，四周余量最大，
是几何上最安全的落点。这也是 Mapbox 的 polylabel 算法要解决的问题。

---

## 8. 本地测试（无需 ROS / 仿真器 / GPU）

以下命令均在**仓库根目录**执行：

```bash
python3 tests/test_rooftop_landing_local.py
```

覆盖 48 项：PointCloud2 解析（含缺字段、空数据、NaN 过滤）、ENU/NED 换算、
RANSAC 平面拟合精度、凸包与面积、polylabel 圆心与半径、点–多边形有符号距离、
**合成城市场景端到端感知**（地面 + 楼房 + 墙面 + 离群点 → 应挑出 15 m 屋顶且
落点接近中心）、以及工程文件约定（入口命名、launch 参数、文件名全英文等）。

预期输出结尾：

```
结果：48/48 通过
```

### 8.2 闭环仿真冒烟测试（有 ROS 即可，不需要仿真器）

`tests/test_rooftop_landing_ros_sim.py` 自带一个**假桥接层**：它按 20 Hz 发 `/uav/odom`
（NED，下为正）、按 10 Hz 发 `/lidar/points`（8 m 高的 8×8 m 屋顶 + 地面），收到
`/uav/goal` 后瞬移到目标点上方并回报 `GOAL_DONE`，再把 `/uav/cmd_vel.linear.z`
积分进高度 —— 于是节点能真的从 15 m 一层层降到屋顶，整条链路闭环。

```bash
# 终端 1
source ~/uav_ws/devel/setup.bash
roslaunch rooftop_landing main.launch use_rviz:=false

# 终端 2
python3 tests/test_rooftop_landing_ros_sim.py
```

预期结尾（实测输出）：

```
[bridge] 合成点云 2097 点（屋顶 1089 点 z=8.0 m，8x8 m；地面 1008 点 z=0）
[node] odom 原始高度 z=-15.00 -> 换算高度 15.00 m（z_up=False）
[node] 点云 2097 点，检测到 1 个候选水平面
[node] 选中屋顶：高度 8.00 m，面积 64.0 m^2，降落点 (8.00, 0.00)，内切圆半径 4.00 m
[node] 下发目标点 (8.00, 0.00, -13.00)，即屋顶上方 5.0 m
[node] 状态 DETECT -> APPROACH
[node] 状态 APPROACH -> DESCEND
[node] 下降中：离屋顶 7.00 m
...
[node] 已降落：离屋顶 0.35 m <= 容差 0.35 m
[node] 状态 DESCEND -> LANDED

出现过 LANDED : 是
出现过 APPROACH: 是
出现过 DESCEND : 是
```

---

## 9. 常见问题

| 现象 | 原因与处理 |
|---|---|
| `未找到合格屋顶，继续搜索` | 点云太少或阈值过严。先 `rostopic echo -n1 /lidar/points/width` 确认有数据；再放宽 `perception/min_plane_inliers`、`min_roof_area` |
| 降落点选到了地面而不是屋顶 | `min_height` 太小。实测地面为 0 m，把 `perception/min_height` 提到 1.0 以上 |
| 目标点下发后无人机不动 | 桥接层没在跑或 `/uav/status` 不是 `GOAL_DONE`；先确认 `roslaunch carlair_ros_bridge main.launch` 已启动 |
| 无人机往下飞变成了往上飞 | 高度约定反了。看日志里 `odom 原始高度` 的正负，然后 `z_up:=true` 或 `false` |
| `接近超时，进入 ABORT` | 目标点太远或桥接限速。调大 `approach/goal_timeout` 或把 `safe_altitude` 调小 |
| 一直停在 `DESCEND` | 高度容差太严。调大 `descend/tolerance`；确认 `landing_alt` 与实际屋顶高度一致 |
| 用 `rosrun ... _frame/z_up:=false` 覆盖不生效 | **实测**：`rosrun` 传这种**嵌套私有参数**不会覆盖 `frame/z_up`，节点仍按默认 `z_up=true` 运行，于是高度取反、`remain` 变成负数并瞬间误判 `LANDED`。请统一用 `roslaunch rooftop_landing main.launch` 启动，参数走 `config/rooftop_landing.yaml` |
| 屋顶高度被识别成负数 | `frame/z_up` **只作用于 `/uav/odom` 与 `/uav/goal`**，点云始终按 ENU（z 向上为正）解析。若桥接层发出的点云是 NED，需要先在桥接层把点云转成 ENU |

---

## 附：大模型使用声明

本模块代码与文档在编写过程中使用了 AI 大模型辅助（需求分析、算法方案讨论、代码与文档起草）。
全部内容已通过两类测试验证：本地回归测试（`tests/test_rooftop_landing_local.py`，48 项全部通过）
与 ROS 闭环冒烟测试（`tests/test_rooftop_landing_ros_sim.py`，在 ROS Noetic 上完整跑通
`DETECT → APPROACH → DESCEND → LANDED`）；作者对提交内容的正确性与完整性负全部责任。
