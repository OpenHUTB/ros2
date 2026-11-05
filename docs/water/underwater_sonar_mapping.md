# 水域载具 - 水下声呐三维测深与避障 ROS 2 封装模块

## 作业概述与背景
本模块基于水下声呐探测原理，在 ROS 2 环境下实现了一个水下三维地形测绘与避障仿真节点。节点模拟了声呐探测海底起伏以及识别障碍物的过程，实时生成包含障碍物信息的三维海底地形图，为水下航行器的自主避障提供环境感知基础。

## 二、运行环境
* 操作系统：Ubuntu 22.04（虚拟机）
* ROS 2 版本：Humble
* 依赖库：rclpy, matplotlib, numpy

## 核心代码文件

本模块包含以下核心文件，均已提交至仓库：

```text
src/
└── underwater_sonar_mapping/
    ├── underwater_sonar_mapping/
    │   ├── main.py              # 三维声呐测绘与障碍物可视化节点（原有）
    │   ├── terrain.py           # 海床高度场 + 障碍物 + 水平射线求交
    │   ├── sonar.py             # 多波束扫描声呐模型（波束布局、量程、噪声、丢包）
    │   ├── vfh.py               # VFH+ 极坐标直方图、环形平滑、谷底选择、降速
    │   └── sonar_vfh_node.py    # 声呐驱动的反应式避障节点
    ├── test/
    │   ├── test_terrain_sonar.py  # 22 项：海床 / 射线 / 声呐
    │   ├── test_vfh.py            # 17 项：直方图 / 平滑 / 选向 / 滞环
    │   └── test_closed_loop.py    # 6 项：闭环到点与避碰
    ├── setup.py                 # ROS 2 功能包配置（含 sonar_vfh 入口）
    └── launch/
        ├── main.launch.py       # 三维测绘可视化
        └── sonar_vfh.launch.py  # VFH 避障
```
## 运行步骤

1. 新建标准工作空间并进入 src 目录：
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```
2. 从 ros2 仓库拷贝模块代码到工作空间：                   
("cp -r ~/OpenHUTB/ros2/src/....._mapping ~/ros2_ws/src/"中OPenHUTB是文件夹名，一般克隆ros2不会自带OPenHUTB文件夹，输入改指令时可以去掉"/OpenHUTB"部分)
```bash
cp -r ~/OpenHUTB/ros2/src/underwater_sonar_mapping ~/ros2_ws/src/
cd ~/ros2_ws
```
3. 编译工作空间并部署环境：
```bash
colcon build --packages-select underwater_sonar_mapping
source install/setup.bash
```
4. 运行水下声呐测绘节点：
```bash
ros2 launch underwater_sonar_mapping main.launch.py
```
5.运行效果（截图）
![运行效果](../img/underwater_sonar_3d_mapping_run.png)

效果说明：节点成功启动，基于模拟声呐数据生成了三维海底地形，并清晰标注了水下沉船/暗礁等障碍物位置，验证了声呐三维测绘与避障感知算法的可行性。

## 声呐驱动的 VFH+ 反应式避障

上面的可视化节点只做静态测绘，不产生控制指令。以下模块补上「感知 → 决策 → 控制」闭环：扫描声呐探测到障碍后，用 VFH+（Vector Field Histogram）选出可通行的方向，并把速度指令发给 `/cmd_vel`。

### 算法流程

1. **声呐扫描**：以 72 束、360° 的水平扫描声呐在当前深度上对海床高度场与方块障碍物做射线求交，得到每束的回波距离（可配置量程、噪声与丢包率）。
2. **极坐标障碍密度**：回波按方位落入 72 个扇区，并按**车辆半径 + 安全余量**膨胀每束的角宽，使车辆挤不过去的窄缝仍被判为阻塞；密度随距离衰减（`(1 - d/d_max)²`）。
3. **环形低通平滑**：用 `[1,2,4,2,1]` 卷积核做循环卷积，消除单束噪声造成的尖峰。
4. **谷底选择**：取密度低于阈值的连续扇区作为候选谷底，选其中最接近目标方位的方向，并加入**滞环**（默认 15°）——只有另一条谷底明显更优时才切换，避免在两个相似缝隙间来回摆动。
5. **降速**：按**所选方向上的净空**（而非全局最近回波）线性降速，避免"侧方贴障但前方开阔"时误停。

### 话题接口

| 话题 | 类型 | 方向 | 说明 |
| --- | --- | --- | --- |
| `/sonar/scan` | `sensor_msgs/LaserScan` | 发布 | 一次扫描（`frame_id: sonar_link`） |
| `/vfh/histogram` | `std_msgs/Float32MultiArray` | 发布 | 平滑后的极坐标障碍密度 |
| `/cmd_vel` | `geometry_msgs/Twist` | 发布 | 避障速度指令 |
| `/odom` | `nav_msgs/Odometry` | 发布 | 仿真位姿（可替换为真实定位） |
| `/vfh/status` | `std_msgs/String` | 发布 | JSON：模式、目标距离、转向角、谷宽、净空 |

节点内置运动积分，**不依赖物理引擎即可独立演示**；接入真实 ROV 时把 `/cmd_vel` 与 `/odom` 换成外部来源即可，算法部分无需改动。

### 主要参数

| 参数 | 默认值 | 说明 |
| --- | --- | --- |
| `goal_x` / `goal_y` | `20.0` / `0.0` | 目标点 |
| `depth` | `6.0` | 巡航深度（NED，向下为正） |
| `v_max` | `1.2` | 最大前向速度，m/s |
| `beams` / `fov_deg` | `72` / `360` | 声呐束数与视场 |
| `max_range` | `30.0` | 声呐量程，m |
| `range_sigma` / `dropout` | `0.02` / `0.0` | 测距噪声标准差与丢包率 |
| `robot_radius` / `safety_margin` | `0.6` / `0.5` | 膨胀半径，m |
| `threshold` | `0.35` | 谷底密度阈值 |
| `min_valley_deg` | `15.0` | 谷底最小角宽，过窄视为不可通行 |
| `hysteresis_deg` | `15.0` | 切换谷底时的滞环角 |

### 运行

```bash
colcon build --packages-select underwater_sonar_mapping
source install/setup.bash
ros2 launch underwater_sonar_mapping sonar_vfh.launch.py
```

或在另一个终端直接运行节点并修改目标：

```bash
ros2 run underwater_sonar_mapping sonar_vfh --ros-args \
  -p goal_x:=25.0 -p goal_y:=6.0 -p v_max:=1.5
```

在 RViz 中订阅 `/sonar/scan`（LaserScan）与 `/odom` 可看到回波与轨迹；用 `ros2 topic echo /vfh/status` 观察避障状态。

### 验证结果

单元测试共 38 项，均不需要 ROS 或仿真器：

```bash
python3 test/test_terrain_sonar.py   # 15 项：海床、射线求交、声呐布局与噪声
python3 test/test_vfh.py             # 17 项：直方图、平滑、选向、滞环、降速
python3 test/test_closed_loop.py     # 6 项：闭环到点与避碰
```

闭环仿真（三块障碍物，起点 `(0,0)`、目标 `(20,0)`）：17.5 s、175 个控制周期到达目标，终点 `(19.23, -0.17)`，全程未进入任何障碍物，与最近障碍的最小间隙约 0.7 m。

> 说明：本模块的声呐是**水平扫描**模型（用于避障），与上方三维测绘节点的俯视测深几何不同，两者互补。

