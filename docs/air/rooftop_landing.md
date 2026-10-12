# 无人机屋顶降落

**屋顶降落**的 ROS 侧实现：订阅桥接层发布的激光点云，检测水平**屋顶平面**，
在屋顶轮廓上取**最大内切圆圆心**作为安全降落点，控制无人机自主飞抵其上方后
垂直降落在屋顶上。

源码位于 [`src/air/rooftop_landing`](https://github.com/OpenHUTB/ros2/tree/master/src/air/rooftop_landing)，
模块的完整说明（运行步骤、参数、话题、常见问题）见
[`README.md`](https://github.com/OpenHUTB/ros2/blob/master/src/air/rooftop_landing/README.md)。

---

## 1. 它解决什么问题

无人机要「落在屋顶上」，比落在平坦地面难得多：

* 屋顶面积有限，**落点必须远离边缘**，否则容易撞到女儿墙或直接掉下去；
* 屋顶上可能有天窗、通风管、水箱等**障碍**，地面点云里还有大片「地面」平面的干扰；
* 需要先**识别出哪个平面才是屋顶**，再决定落在哪里。

本模块的做法是：

1. 用 **RANSAC** 从点云里拟合平面，只保留接近**水平**的面；
2. 用平面高度与面积把**屋顶**和**地面**区分开；
3. 先把平面上互不相连的点群拆开（连通域聚类，避免少数远处的噪点把轮廓撑大），
   再逐块投影成 2D 轮廓（凸包），用 **polylabel** 求**最大内切圆圆心**
   —— 即离屋顶边界最远的点，也就是几何上最安全的落点；
4. 按「最高 / 最近 / 最宽敞」策略选出目标屋顶，下发目标点飞抵其上方，
   再以恒定小速度垂直下降，直到高度接近屋顶。

---

## 2. 架构位置

本模块**不直接连仿真器**，只与桥接层交互：

```
Windows 宿主                                    Ubuntu 客户机
┌──────────────────┐                        ┌────────────────────────────┐
│ CarlaAir / AirSim│◀── TCP 41451 ─────────▶│ carlair_ros_bridge         │
│  无人机 + LiDAR  │                        │  /uav/odom  /lidar/points  │
└──────────────────┘                        │  /uav/cmd_vel  /uav/goal   │
                                            └────────────┬───────────────┘
                                                         │ 订阅 / 发布
                                            ┌────────────▼───────────────┐
                                            │ rooftop_landing（本模块）  │
                                            │  检测屋顶 → 选落点 → 降落  │
                                            └────────────────────────────┘
```

连仿真器、NED↔ENU 坐标换算、速度限幅与指令超时保护都在桥接层完成，因此本模块
与键盘控制、规划器等模块**共用同一条 `/uav/cmd_vel` 通道**，互不耦合。

---

## 3. 运行

前置：按 [无人机与 ROS 的桥接模块](./carlair_ros_bridge.md) 的 **2.1 节**在宿主机启动
CarlaAir 仿真器，等终端输出 `CarlaAir is ready.`（此时 `41451` 端口就绪）。
**仿真器窗口不要最小化**，否则 AirSim 取相机会阻塞。

```bash
# 客户机侧 1：启动桥接（连仿真、发布传感器）
source ~/uav_ws/devel/setup.bash
roslaunch carlair_ros_bridge main.launch
# auto_takeoff 默认为 true：桥接层会 takeoff() 并 hover()，起飞后悬停在约 1 m

# 客户机侧 2：启动屋顶降落（作业要求的 launch 启动）
roslaunch rooftop_landing main.launch
```

可选参数：

```bash
roslaunch rooftop_landing main.launch safe_altitude:=6.0 prefer:=closest
roslaunch rooftop_landing main.launch z_up:=true          # 若高度约定为 ENU 上为正
roslaunch rooftop_landing main.launch use_rviz:=false     # 不开 RViz
roslaunch rooftop_landing main.launch autostart:=false    # 手动用 /rooftop/start 触发
```

运行时的日志会依次给出「检测到几个候选面 → 选中哪个屋顶（高度/面积/落点/内切圆半径）
→ 下发目标点 → 到达后开始下降 → 触地」的完整链路，便于逐段核对。

---

## 4. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/lidar/points` | `sensor_msgs/PointCloud2` | 激光点云 |
| 订阅 | `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度 |
| 订阅 | `/uav/status` | `std_msgs/String` | 桥接状态机 |
| 订阅 | `/rooftop/start` | `std_msgs/Bool` | 手动开始一次任务（`autostart:=false` 时用） |
| 发布 | `/uav/goal` | `geometry_msgs/Point` | 降落点上方的目标点 |
| 发布 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 降落阶段的垂直速度 |
| 发布 | `/rooftop/landing_point` | `geometry_msgs/Point` | 选中的降落点（z = 屋顶高度，上为正） |
| 发布 | `/rooftop/markers` | `visualization_msgs/MarkerArray` | 屋顶轮廓与落点可视化 |

RViz 中打开 `PointCloud2(/lidar/points)` 与 `MarkerArray(/rooftop/markers)`，
把 `Fixed Frame` 设为 `world`，即可看到点云与识别出的屋顶轮廓（橙色线框）
和降落点（绿色球体）。

---

## 5. 本地测试（无需 ROS / 仿真器）

```bash
python3 tests/test_rooftop_landing_local.py
```

55 项全部通过。其中「端到端感知」用**合成城市场景**（地面 + 楼房 + 墙面 + 离群点）
验证算法：应挑出 15 m 高的屋顶，落点接近屋顶中心，内切圆半径接近 9 m；
「凸包鲁棒性回归」另外验证了少量同高度噪点不会把轮廓撑大、相邻但不相连的
两栋楼会被拆成两个候选。

这套测试的意义在于：**在没有显卡、没有仿真器、甚至没有 ROS 的机器上也能回归**，
方便在没有条件启动整套仿真的情况下先验证算法改动。

另有 `tests/test_rooftop_landing_ros_sim.py`：自带一个假桥接层，
只需要 ROS 即可跑完 `DETECT → APPROACH → DESCEND → LANDED` 全流程，不需要仿真器。
