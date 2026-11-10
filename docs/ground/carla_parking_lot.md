# CARLA 停车场场景生成

## 概述
在 CARLA 0.9.16 仿真器的 Town04 地图中，通过 Python API 自动生成一个带车位线的停车场场景，包含 6 个垂直停车位，并自动将镜头移动到停车场上方俯视。

## 环境要求
- CARLA 0.9.16（服务端运行于 Windows 有 GPU 的宿主机）
- Ubuntu 22.04 + ROS 2 Humble + Python 3.10（运行客户端脚本）
- Python 依赖：`carla==0.9.16`

## 安装步骤

### 1. 下载安装 Windows 版 CARLA
从官方 Release 页面下载 CARLA 0.9.16 Windows 版：
https://github.com/carla-simulator/carla/releases/tag/0.9.16

下载 `CARLA_0.9.16.zip`，解压到任意目录（记为 CARLA 安装目录）。

### 2. 启动 CARLA 服务端
进入 CARLA 安装目录后运行：
```powershell
CarlaUE4.exe -quality-level=Low
```

### 3. 配置 Windows 防火墙

在 Windows Defender 防火墙中添加入站规则，放行 TCP 2000 和 UDP 2000 端口。

### 4. 获取代码并编译 ROS2 包

在 Ubuntu 客户端创建工作空间，并将仓库克隆到 src 目录：

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/OpenHUTB/ros2.git
```

然后编译停车场模块：

```
cd ~/ros2_ws
colcon build --packages-select carla_parking_lot
source install/setup.bash
```

## 运行

```
# 默认连接 192.168.43.1:2000
ros2 launch carla_parking_lot parking_lot.launch.py

# 指定 CARLA 服务器 IP
ros2 launch carla_parking_lot parking_lot.launch.py host:=192.168.1.100
```

## 运行效果

- 自动切换到 Town04 地图
- 在停车场位置绘制 6 个绿色车位线框
- 标注 P1-P6 车位编号
- 镜头自动跳到停车场上方俯视

## 技术说明

- 使用 `world.debug.draw_line()` 绘制车位边界线
- 使用 `world.debug.draw_string()` 标注车位编号
- 使用 `world.get_spectator()` 获取并移动镜头
- 车位坐标通过在 CARLA 窗口中飞镜头手动读取获得

## 自主实现：不依赖 CARLA 的倒车入库规划器

上面的流程门槛不低：要一台有 GPU 的 Windows 机器跑 CARLA 服务端，要 Ubuntu 22.04 +
ROS 2 Humble + Python 3.10 跑客户端，还要放行 2000 端口。如果只是想回答「这台车能不能
停进这个车位、会不会蹭到旁边的车、要倒多少米」这类问题，其实不需要整个仿真器。

`src/ground/reverse_parking_planner` 就是把这部分**只跟几何有关**的逻辑单独拿出来做成了
一个 ROS 1 包：给定车位尺寸、车辆几何和转弯半径，算出一次倒车入库的完整路径，在终端
打印断言结果和字符俯视图，并把路径发布成 `nav_msgs/Path`。它不订阅任何话题，不依赖
CARLA，也没有图形界面，`roslaunch` 一跑就能出结果。

### 目录结构

```
src/ground/reverse_parking_planner/
├── CMakeLists.txt
├── package.xml
├── README.md
├── config/parking_planner.yaml
├── launch/main.launch
└── scripts/
    ├── main.py                  # 入口
    ├── parking_planner_node.py  # 读参数、算路径、发消息
    └── parking_core.py          # 几何核心（纯标准库，可离线测）
```

### 路径怎么算出来

全程倒车，所以路径只有两段，长度恒为 `R·pi/2 + entry_length`：

1. 目标位姿 `T` 在车位正中，车头朝向车道 —— 倒车入库停好之后车头是朝外的；
2. 倒车行驶方向与车头相反，从 `T` 沿行驶方向反向退 `entry_length` 米得到 `J`，
   此时车头已经对准车位，只剩最后一小段直线；
3. 圆心 `C` 取在 `J` 行车方向的右侧 `turn_radius` 米处，起点 `S = C − R·travel`；
4. 圆弧从 `S` 右转 90° 到 `J`，再从 `J` 倒车 `entry_length` 米到 `T`。

规划前会先校验 `turn_radius` 是否小于车辆最小转弯半径
`wheelbase / tan(max_steer)`（默认 `2.70 / tan(0.55) ≈ 4.40 m`）。小于就直接抛
`ParkingError`，不会给出一个物理上开不出来的路径。

默认参数下的关键量：路径长 `5·π/2 + 3 = 10.854 m`，转角 90°，起点 `(5.000, −5.750)`，
终点 `(0.000, 2.250)`，采样 110 个点。

### 参数

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `slot/width` / `slot/length` | `2.50` / `5.30` | 车位宽 / 长（m） |
| `slot/x` / `slot/y` / `slot/yaw` | `0.0` / `0.0` / `−1.5707963` | 车位中心与车头朝向 |
| `vehicle/wheelbase` | `2.70` | 轴距（m） |
| `vehicle/length` / `vehicle/width` | `4.50` / `1.80` | 车长 / 车宽（m） |
| `vehicle/max_steer` | `0.55` | 最大前轮转角（rad） |
| `plan/turn_radius` | `5.00` | 规划用转弯半径（m） |
| `plan/entry_length` | `3.00` | 最后一段倒车直线长度（m） |
| `plan/step` | `0.10` | 采样步长（m） |
| `check/tolerance` | `0.10` | 终点位置容差（m） |
| `check/clearance` | `0.00` | 与邻车间距下限（m） |
| `check/max_path` | `12.00` | 允许的最长倒车距离（m） |
| `rate/publish_hz` | `10.0` | 逐点回放频率（Hz） |
| `topic/plan` / `topic/report` / `topic/point` | `/parking/plan` 等 | 输出话题 |

### 话题

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/parking/plan` | `nav_msgs/Path` | 完整路径，latched，可直接在 RViz 里看 |
| `/parking/report` | `std_msgs/String` | 报告全文（断言 + 俯视图），latched |
| `/parking/plan_point` | `geometry_msgs/PoseStamped` | 按 `rate/publish_hz` 逐点回放 |

### 运行

```bash
catkin_make
source devel/setup.bash
roslaunch reverse_parking_planner main.launch
```

默认参数下的真实输出：

```
倒车入库规划报告
车辆：长 4.50 m，宽 1.80 m，轴距 2.70 m，最小转弯半径 4.40 m
车位：垂直车位 2.50 m x 5.30 m（宽 x 长），转弯半径 5.00 m，直线段 3.00 m
起点：Pose(x=5.000, y=-5.750, yaw=3.142)
终点：Pose(x=-0.000, y=2.250, yaw=-1.571)
路径：倒车 10.85 m，转角 90.0 度，采样 110 个
最小间距：0.494 m
断言：4 通过 / 0 失败（共 4 条）
  [PASS] 终点精度  偏差 0.000 m，容差 0.10 m
  [PASS] 无碰撞  与邻车最小间距 0.494 m，下限 0.00 m
  [PASS] 曲率上限  曲率 0.200 1/m，车辆上限 0.227 1/m
  [PASS] 路径长度  倒车 10.85 m，上限 12.00 m
结论：PASSED
```

反例：把车位缩到比车还窄，`无碰撞` 会立刻失败。

```bash
roslaunch reverse_parking_planner main.launch slot_width:=1.7
```

```
车位：垂直车位 1.70 m x 5.30 m（宽 x 长），转弯半径 5.00 m，直线段 3.00 m
断言：3 通过 / 1 失败（共 4 条）
  [FAIL] 无碰撞  与邻车最小间距 0.000 m，下限 0.00 m
结论：FAILED
```

### 离线测试

```bash
python3 tests/test_reverse_parking_planner_local.py
```

只依赖标准库，不需要 ROS、CARLA 或显示器，覆盖参数读取、路径几何、碰撞检测、四种断言、
报告格式、俯视图，以及 `package.xml` / `CMakeLists.txt` / `main.launch` 的结构约定。

### 与 CARLA 方案的分工

| | `reverse_parking_planner` | 本页上面的 CARLA 流程 |
| --- | --- | --- |
| 依赖 | ROS 1 + Python（无 GPU） | CARLA 0.9.16 + ROS 2 Humble + Python 3.10 |
| 输出 | 终端报告 + 路径话题 | 在 Town04 里画出车位线框 |
| 用途 | 规划逻辑的离线回归 | 场景可视化演示 |

两者不冲突：CARLA 负责「看起来对不对」，这个包负责「数值上停不停得进去」。

### 常见问题

* **`ParkingError: 转弯半径 3.000 m 小于车辆最小转弯半径 4.404 m`**
  `plan/turn_radius` 比车能转的最小半径还小。调大 `turn_radius`，或调大
  `vehicle/max_steer`（例如 `0.70`）。
* **`[FAIL] 无碰撞`**
  车位太窄或直线段太短，车身切入时会碰到邻车。加大 `slot_width` 或 `entry_length`。
* **Melodic 上日志里出现 `UnicodeEncodeError`**
  给 rospy 打非 ASCII 日志必须传 byte str，节点里已用 `_log()` 统一转换；如果你在别的
  节点里自己写 `rospy.loginfo("中文 %s", x)`，在 Python 2.7 下会踩同样的坑。

所使用具体的大模型声明：使用大模型辅助代码编写、测试编写与文档整理，提交内容由本人负责。
