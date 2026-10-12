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
[INFO] [rooftop_landing] 已启动，状态=DETECT，高度约定 z_up=True
[INFO] [rooftop_landing] odom 原始高度 z=1.20 -> 换算高度 1.20 m（z_up=True）
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
| `frame/z_up` | `true` | 高度分量是否「上为正」。**实测 carlair_ros_bridge 为 ENU（上为正）**，故默认 true；换用别的桥接层且方向不符时再改 |
| `frame/world` | `world` | marker 与状态话题的 `frame_id` |
| `perception/accumulate_frames` | 3 | 累积多少帧点云后再做一次检测 |
| `perception/max_tilt_deg` | 15.0 | 平面水平性阈值（度） |
| `perception/plane_dist_thresh` | 0.15 | RANSAC 内点距离阈值 (m) |
| `perception/min_plane_inliers` | 120 | 一个平面最少点数 |
| `perception/min_roof_area` | 8.0 | 屋顶最小面积 (m²) |
| `perception/min_height_mode` | `relative` | 屋顶高度判定：`relative`（以检出的最低水平面＝地面为基准）/ `absolute`（用 `min_height` 作绝对 ENU 阈值） |
| `perception/ground_margin` | 1.0 | `relative` 模式：比检出地面至少高这么多米才算屋顶 |
| `perception/min_height` | 0.5 | `absolute` 模式的高度下限 (m)；`relative` 模式下忽略 |
| `perception/max_planes` | 5 | 最多提取几个候选水平面（城市视野里前几个可能全是地面/墙，可提到 15） |
| `perception/min_landing_radius` | 0.6 | 降落点最小安全半径 (m)。**硬约束**：一个候选都不满足就继续搜索，不会退回不安全的候选 |
| `perception/prefer` | `highest` | 目标屋顶策略：`highest` / `closest` / `largest` |
| `perception/cluster_cell` | 1.0 | 连通域聚类栅格边长 (m)：拆开平面上互不相连的点群 |
| `perception/cluster_min_points` | 2 | 一个栅格至少几个点才算被占据（滤掉零星噪点） |
| `approach/safe_altitude` | 5.0 | 高于屋顶的安全高度 (m) |
| `approach/goal_timeout` | 60.0 | 等待 `GOAL_DONE` 超时 (s) |
| `approach/verify_goal_coords` | `true` | `GOAL_DONE` 带坐标时，核对它是不是**本节点下发的那个目标点**（分辨别人的到达状态，见 §9） |
| `approach/goal_match_tol` | 0.5 | 上面那个核对的位置容差 (m) |
| `approach/goal_retry_period` | 5.0 | 目标点被桥接忽略（已有别的任务在执行）时的重发间隔 (s)，`0` = 不重发 |
| `descend/speed` | 0.5 | 垂直下降速度 (m/s) |
| `descend/tolerance` | 0.35 | 判定触地的高度容差 (m) |
| `descend/timeout` | 60.0 | 下降超时 (s) |
| `mission/autostart` | `true` | 启动后是否自动开始；`false` 则等 `/rooftop/start` |
| `mission/start_topic` | `/rooftop/start` | 手动启动话题（`std_msgs/Bool`） |
| `mission/tick_rate_hz` | 10.0 | 主循环频率 (Hz)；下降阶段的 `cmd_vel` 也按此频率发布 |
| `topic/lidar` | `/lidar/points` | 输入点云，需与桥接层一致 |
| `topic/odom` | `/uav/odom` | 输入位姿 |
| `topic/status` | `/uav/status` | 输入桥接层状态（`GOAL_DONE` 等） |
| `topic/cmd_vel` | `/uav/cmd_vel` | 输出速度指令 |
| `topic/goal` | `/uav/goal` | 输出目标点 |
| `visualization/publish_markers` | `true` | 是否发布 RViz marker |
| `visualization/marker_topic` | `/rooftop/markers` | 屋顶轮廓与降落点的 marker 话题 |
| `visualization/landing_topic` | `/rooftop/landing_point` | 选中降落点的话题 |

示例：

```bash
roslaunch rooftop_landing main.launch safe_altitude:=6.0 prefer:=closest
roslaunch rooftop_landing main.launch z_up:=false         # 若桥接层高度是 NED 下为正

# 换地图时最常现场调整的感知阈值 —— 都已暴露成 launch 参数，不必改 yaml：
roslaunch rooftop_landing main.launch max_planes:=15 min_plane_inliers:=40
roslaunch rooftop_landing main.launch min_roof_area:=2.0 cluster_min_points:=1
roslaunch rooftop_landing main.launch min_height_mode:=absolute min_height:=1.0
```

---

## 6. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/lidar/points` | `sensor_msgs/PointCloud2` | 激光点云（世界系，来自桥接层） |
| 订阅 | `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度 |
| 订阅 | `/uav/status` | `std_msgs/String` | 桥接状态机 `READY/HOVER/VELOCITY/GOAL/GOAL_DONE`。注意 `GOAL` / `GOAL_DONE` 后面**还带三个坐标**（如 `GOAL_DONE -1.43 0.14 -20.08`），本模块只比较**第一个词** |
| 订阅 | `/rooftop/start` | `std_msgs/Bool` | 手动开始一次任务（`autostart:=false` 时用） |
| 发布 | `/uav/goal` | `geometry_msgs/Point` | 屋顶降落点上方的目标点 |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 降落阶段的垂直速度指令（机体系） |
| 发布 | `/rooftop/landing_point` | `geometry_msgs/Point` | 选中的降落点，z = 屋顶高度（上为正），便于外部记录 |
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
   │  ④ 平面上互不相连的点群拆开（连通域聚类）        split_clusters_2d
   │     滤掉零星噪声点；相邻的两栋楼各自成为候选
   ▼
   │  ⑤ 每个连通块投影 → 2D 凸包作为屋顶轮廓         convex_hull_2d
   ▼
   │  ⑥ polylabel 求最大内切圆圆心 = 降落点          polylabel
   ▼
   │  ⑦ 按高度/半径打分选最优屋顶                   select_landing_target
   ▼
目标点(/uav/goal) → 桥接层飞抵悬停 → /uav/status = GOAL_DONE
   ▼
垂直速度(/uav/cmd_vel) → 高度接近屋顶 → LANDED
```

**为什么要先做连通域聚类**：平面内点里常混有零星离群点 —— 远处的杂点、以及
**邻近楼顶**的点，都会落在同一个高度带里被 RANSAC 收进内点。而凸包对离群点
极端敏感：实测 6900 个点里混进 4 个同高度的远处噪点（占 0.06%），就把 216 m²
的屋顶撑成 1175 m²，「最大内切圆圆心」偏了 22 m，会落到楼外的街上。
先按 1 m 栅格做八邻域连通域标记、丢掉点数不足的格子，再对每个连通块单独求凸包
即可解决；副产品是**相邻但不相连的两栋楼会被拆成两个独立候选屋顶**，
使 `prefer=closest/largest` 能真正在多栋楼之间做选择。

**为什么用最大内切圆圆心而不是质心**：屋顶往往不是规则矩形，质心可能落在天窗、
通风管或靠近边缘的位置；内切圆圆心是**离屋顶边界最远**的点，四周余量最大，
是几何上最安全的落点。这也是 Mapbox 的 polylabel 算法要解决的问题。

---

## 8. 本地测试（无需 ROS / 仿真器 / GPU）

以下命令均在**仓库根目录**执行：

```bash
python3 tests/test_rooftop_landing_local.py
```

覆盖 93 项：PointCloud2 解析（含缺字段、空数据、NaN 过滤）、ENU/NED 换算、
RANSAC 平面拟合精度、凸包与面积、polylabel 圆心与半径、点–多边形有符号距离、
**合成城市场景端到端感知**（地面 + 楼房 + 墙面 + 离群点 → 应挑出 15 m 屋顶且
落点接近中心）、**换地图的高度基准回归**（地图原点在地面之上约 25 m 时，
默认的相对地面基准仍要挑出屋顶、且不会把地面当屋顶）、**安全半径硬约束**、
**桥接状态解析与目标点认领**，以及工程文件约定（入口命名、launch 参数、文件名全英文等）。

预期输出结尾：

```
结果：93/93 通过
```

### 8.2 闭环仿真冒烟测试（假桥接层，**不含仿真器**）

`tests/test_rooftop_landing_ros_sim.py` 自带一个**假桥接层**：它按 20 Hz 发 `/uav/odom`
（ENU，上为正）、按 10 Hz 发 `/lidar/points`（8 m 高的 8×8 m 屋顶 + 地面），收到
`/uav/goal` 后瞬移到目标点上方并回报 `"GOAL_DONE <x> <y> <z>"`，再把
`/uav/cmd_vel.linear.z` 积分进高度 —— 于是节点能真的从 15 m 一层层降到屋顶，
整条链路闭环。

> ⚠️ 它**只证明「本节点 + 桥接协议」这一层是通的，不等价于真实链路**：
> 点云是合成的、位置是瞬移过去的，没有仿真器、没有真实传感器噪声。
> 真机验证见 §8.3。
>
> 这里的 `GOAL_DONE` **必须带三个坐标**（与 `carlair_ros_bridge` 的
> `cmd_vel_sub.py` 完全一致）。早期版本只发一个裸词 `GOAL_DONE`，
> 于是掩盖了节点用全串比较 `self.status == "GOAL_DONE"` 的 bug：
> 假桥接测试通过、真机必挂（详见 §9 第一行）。

```bash
# 终端 1
source ~/uav_ws/devel/setup.bash
roslaunch rooftop_landing main.launch use_rviz:=false

# 终端 2
python3 tests/test_rooftop_landing_ros_sim.py
```

预期结尾：

```
[bridge] 合成点云 2097 点（屋顶 1089 点 z=8.0 m，8x8 m；地面 1008 点 z=0）
[node] odom 原始高度 z=15.00 -> 换算高度 15.00 m（z_up=True）
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

### 8.3 真机链路验证（仿真器 + 真实桥接层 + 真实点云）

在「模拟器跑在宿主 Windows、ROS 跑在客户机 Ubuntu」的真实分工下，用仓库的
`carlair_ros_bridge` 走了一遍真实链路：

| 项 | 值 |
|---|---|
| 客户机 | Ubuntu 20.04.6 + ROS Noetic 1.16.0，`192.168.49.128` |
| 宿主机 | OpenHUTB / Cosys-AirSim（UE 5.5，Blocks），`192.168.49.1:41451` |
| 桥接层自检 | `自检结果: 5/5 通过`（点云 / 里程计 / 相机链路都通） |
| 真实点云 | LiDAR 16 线 × 10 Hz，**4165 点/帧**（累积 3 帧约 1.2 万点） |
| 本模块 | **`launch/main.launch` 默认参数，未改 `config/rooftop_landing.yaml`** |

![真机运行终端记录](../../img/air/rooftop_landing/real_run_console.png)

拿到的结果：`点云 8082 点，检测到 1 个候选水平面` →
`选中屋顶：高度 -0.19 m，面积 18.7 m^2，降落点 (3.84, 18.09)，内切圆半径 0.85 m` →
`下发目标点 (3.84, 18.09, 4.81)` → 桥接回报 `GOAL_DONE 3.84 18.09 4.81` →
`状态 DETECT -> APPROACH` → `状态 APPROACH -> DESCEND`。

**图里最值得看的一行是 `高度基准：相对地面 -12.65 m + 1.0 m → 丢弃低于 -11.65 m 的平面`**：
这台模拟器的地图原点在**地面之上约 13 m**，所有平面高度都是负的十几米。若按旧版把
`min_height` 写死成 `0.5`（隐含「地面在 z≈0」），结果是**每一个候选平面都被滤掉**、
日志停在「检测到 0 个候选水平面」、无人机一动不动 —— 这正是 §5 把高度判定改成
**相对检出地面**的原因；改成相对量后，默认参数在这张地图上直接可用。

> ⚠️ **这次真机验证没有跑到 `LANDED`，我们不把它记成成功。**
> 节点进入 `DESCEND` 后，仿真器 RPC 长时间无响应（该次运行宿主机提交内存接近上限，
> UE 进程持续换页），`/uav/odom` 停止更新、速度指令不再生效，节点按设计在
> `descend/timeout` 后进入 `ABORT`（安全悬停）。
> 即：**感知 → 选点 → 下发目标点 → 认领到达状态**这几段已在真机上验证；
> **下降到底**这一段目前只在 §8.2 的闭环测试里验证过，真机上还欠一次干净复现。

> 另外，真机这张地图上点云较稀疏（4165 点/帧），若现场屋顶更小更碎，可以按 §5
> 用 launch 参数直接放宽阈值，例如
> `roslaunch rooftop_landing main.launch min_plane_inliers:=40 max_planes:=15 min_roof_area:=2.0`。

**顺带发现的两个上游问题（不属于本模块，仅记录）**：

1. `carlair_ros_bridge` 的 `cmd_vel_sub.py` 在**子线程**里调用 AirSim 的 `*Async` 接口。
   msgpackrpc 的 `Future.join()` 运行的是**创建 Client 那个线程**的 Tornado IOLoop，
   子线程里既没有默认事件循环（实测报 `There is no current event loop in thread
   'Thread-10'`），新建的循环也没有人驱动 ⇒ 目标点任务**必然失败或永久阻塞**
   （`move_to_position_enu` 与 `hover` 都走这条路）。本次验证是在虚拟机里把该任务
   挪回主循环执行的 —— **本地临时改动，没有提交到本 PR**。
2. 同一节点在「已有目标点任务在执行」时会**直接丢弃**新目标（只打一条 warn）。
   本模块原本只发一次目标点，一旦撞上就会被白白等掉整个 `approach/goal_timeout`；
   因此新增了 `approach/goal_retry_period`（默认 5 s）定期重发，图里那两行
   `下发目标点 …` 就是它生效的样子。

---

## 9. 常见问题

| 现象 | 原因与处理 |
|---|---|
| `检测到 0 个候选水平面`（无人机一直不动） | **高度基准不对。**每次检测都会打印一行 `高度基准：相对地面 … m + … m → 丢弃低于 … m 的平面`。若这行显示「只提取到 N 个水平面，无法确定地面基准」，说明点云里水平面太少（放宽 `min_plane_inliers`），或者根本不是水平面（检查点云本身）；若基准值明显不对（把地面当成了参照），把 `perception/ground_margin` 调大，或改用 `min_height_mode:=absolute` 并手填 `min_height`（按你地图的原点高度算）。**这是实测过的坑：地图原点在地面之上约 25 m 时，固定的 `min_height=0.5` 会把每个候选面都滤掉** |
| 降落点选到了地面而不是屋顶 | 地面没被排除。先看日志的 `高度基准` 行：`relative` 模式确认 `floor_ref` 就是地面高度；`absolute` 模式把 `perception/min_height` 提到地面之上。**注意别把过滤关掉**（把 `min_height` 设成很小的负数）——那样地面会被当成屋顶，无人机会在地面上"降落" |
| 点云太少或阈值过严 | 先 `rostopic echo -n1 /lidar/points/width` 确认有数据；再放宽 `perception/min_plane_inliers`、`min_roof_area`、`max_planes` |
| 换一张地图就要手调一堆阈值 | 关键阈值都已在 `launch/main.launch` 暴露成参数（`min_height_mode` / `ground_margin` / `min_plane_inliers` / `min_roof_area` / `max_planes` / `cluster_min_points` / `min_landing_radius`），用 `roslaunch rooftop_landing main.launch 参数:=值` 现场调即可，不必改 yaml |
| 降落点落到了屋顶外面（像是在街上） | 平面上混进了远处同高度的噪点，把凸包撑开了。检查 `perception/cluster_cell`（默认 1.0 m）和 `perception/cluster_min_points`（默认 2）是不是被改得过小；点云很稀疏时可把 `cluster_cell` 调大一些 |
| 目标点下发后无人机不动 | 桥接层没在跑或 `/uav/status` 不是 `GOAL_DONE`；先确认 `roslaunch carlair_ros_bridge main.launch` 已启动。节点日志里的 `桥接状态：…` 行会原样打印收到的字符串，可用来核对格式 |
| 无人机在**错误的位置**开始下降（甚至原地下降直到超时） | 收到了**别人**的到达状态。`/uav/status` 会被桥接的所有使用者共享：手动 `rostopic pub /uav/goal`、其它模块的目标点完成了都会发 `GOAL_DONE`。本节点默认按坐标核对（`approach/verify_goal_coords` / `goal_match_tol`），对不上就忽略并在日志里打「忽略别人的到达状态」；若你的桥接**上报的是到达后的实测位置而不是目标点**，把 `goal_match_tol` 调大或关掉核对 |
| `已到达目标点` 迟迟不来（桥接日志里是「已有目标点任务在执行，忽略新目标」） | 目标点被桥接丢弃了。本节点默认每 `approach/goal_retry_period` 秒重发一次；若把该参数设成 0，就会一直等到 `goal_timeout` 才 ABORT |
| 无人机往下飞变成了往上飞 | 高度约定反了。看日志里 `odom 原始高度` 的正负：原始值为负说明桥接层是 NED，把 `frame/z_up` 改成 `false` |
| `接近超时，进入 ABORT` | 目标点太远或桥接限速。调大 `approach/goal_timeout` 或把 `safe_altitude` 调小。**若桥接日志已经打印「已到达目标点」而本节点仍超时**，说明 `/uav/status` 的格式与预期不一致：节点只比较第一个词，对照日志里的 `桥接状态：…` 行即可判断（历史上就是这里踩过坑，见 §8.2 的提示） |
| 一直停在 `DESCEND` | 高度容差太严。调大 `descend/tolerance`；确认 `landing_alt` 与实际屋顶高度一致 |
| 用 `rosrun ... _frame/z_up:=false` 覆盖不生效 | **实测**：`rosrun` 传这种**嵌套私有参数**不会覆盖 `frame/z_up`，节点只按 `config/rooftop_landing.yaml` 里的值运行。若桥接层的高度方向恰好与该值相反，就会出现高度取反、`remain` 变成负数并瞬间误判 `LANDED`。请统一用 `roslaunch rooftop_landing main.launch` 启动，参数走配置文件 |
| 屋顶高度被识别成负数 | `frame/z_up` **只作用于 `/uav/odom` 与 `/uav/goal`**，点云始终按 ENU（z 向上为正）解析。若桥接层发出的点云是 NED，需要先在桥接层把点云转成 ENU |

---

## 附：大模型使用声明

本模块代码与文档在编写过程中使用了 AI 大模型辅助（需求分析、算法方案讨论、代码与文档起草）。
全部内容已通过三类验证：**本地回归测试**（`tests/test_rooftop_landing_local.py`，93 项全部通过）、
**假桥接闭环冒烟测试**（`tests/test_rooftop_landing_ros_sim.py`，在 ROS Noetic 上完整跑通
`DETECT → APPROACH → DESCEND → LANDED`，不含仿真器）、以及**真机链路验证**
（仿真器 + `carlair_ros_bridge` + 真实点云，见 §8.3，**下降到底这一段尚未在真机上复现成功**）；
作者对提交内容的正确性与完整性负全部责任。
