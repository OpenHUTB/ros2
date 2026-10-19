# 路径点发布器

本文介绍 CARLA ROS Bridge 中的**路径点发布器**：它由 CARLA 地图导出道路中心线，
以话题形式发布给其它节点使用，属于 `carla_ros_bridge` 的既有功能。

## 参考

* [Carla 路径点发布器](https://openhutb.github.io/doc/carla_waypoint/)

## 自主实现：带动态目标点切换的路径点发布器

本文的 waypoint publisher 来自 `carla_ros_bridge`：它把地图导出的道路中心线**整条**
发出来，车辆开向哪个点、什么时候换目标，都要由另一个节点自己算。下面是在同一页
基础上**自主实现**的替代方案 —— 一个**不依赖 ros-bridge、也不依赖 CARLA** 的 ROS 包，
除了发布路径点，还把「当前目标点」单独发布出来，并支持运行期间**动态目标点切换**。

代码位于
[`src/ground/waypoint_publisher`](https://github.com/OpenHUTB/ros2/tree/master/src/ground/waypoint_publisher)。

### 与 ros-bridge 的 waypoint publisher 的区别

| 对比项 | 本文示例（ros-bridge） | 本模块 |
| --- | --- | --- |
| 依赖 | 必须编译并运行 `carla_ros_bridge` | **不需要**，只依赖 rospy / geometry_msgs / nav_msgs / std_msgs |
| 发布内容 | 整条中心线路径点 | 路径点 **+ 当前目标点 + 当前下标** |
| 目标点管理 | 无（由订阅方自己决定） | `sequential` / `loop` / `manual` 三种推进模式 |
| 动态切换 | 无 | 按序号跳转 / 手动下一个 / 就近吸附 |
| 到达判定 | 无 | 订阅 `/odom`，进入 `target_tolerance` 半径即推进 |
| 无 CARLA 时 | 必须连仿真器 | 内置模拟车辆，`mock_enabled:=true` 即可跑通 |

### 话题接口

| 方向 | 话题 | 消息类型 | 说明 |
| --- | --- | --- | --- |
| 发布 | `/waypoints` | `geometry_msgs/PoseArray` | 全部路径点（latched，晚连上的节点也能收到） |
| 发布 | `/waypoint/target` | `geometry_msgs/PoseStamped` | 当前目标点 |
| 发布 | `/waypoint/current_index` | `std_msgs/Int32` | 当前目标点下标（0 起算），变化时才发 |
| 发布 | `/odom` | `nav_msgs/Odometry` | 仅 `mock/enabled: true` 时由本模块模拟发布 |
| 订阅 | `/waypoint/set_index` | `std_msgs/Int32` | 切换到第 N 个点（**1 起算**，越界忽略） |
| 订阅 | `/waypoint/next` | `std_msgs/Empty` | 手动前进一个 |
| 订阅 | `/waypoint/snap_nearest` | `std_msgs/Empty` | 吸附到离当前位置最近的点 |
| 订阅 | `/odom` | `nav_msgs/Odometry` | 真实 / 仿真位姿 |

### 参数

| 参数名 | 默认值 | 单位 | 说明 |
| --- | --- | --- | --- |
| `rate/publish_hz` | `10.0` | Hz | 目标点 / 状态的发布频率 |
| `frame/id` | `map` | — | 路径点坐标系 |
| `control/mode` | `sequential` | — | `sequential` / `loop` / `manual` |
| `control/target_tolerance` | `1.0` | m | 到达判定阈值；`<= 0` 关闭自动推进 |
| `control/start_index` | `0` | — | 初始目标点下标（超范围自动夹住） |
| `control/autostart` | `true` | — | 启动后立即开始 |
| `route/file` | `""` | — | 路线文件路径；留空用内置示例路线 |
| `mock/enabled` | `false` | — | `true` 时自带模拟车辆并发布 `/odom` |
| `mock/speed` | `2.0` | m/s | 模拟车辆速度 |

完整清单（含话题重映射）见模块内的 `config/waypoint_publisher.yaml`。

### 运行

```bash
# 默认：内置示例路线（10 m 见方环路、8 个点）
roslaunch waypoint_publisher main.launch

# 让目标点自动依次切换（内置模拟车辆，不需要 CARLA）
roslaunch waypoint_publisher main.launch mock_enabled:=true

# 循环模式 + 到达阈值 0.5 m
roslaunch waypoint_publisher main.launch mock_enabled:=true mode:=loop target_tolerance:=0.5
```

### 动态切换目标点

运行期间在任意终端执行（下标 **1 起算**）：

```bash
rostopic pub -1 /waypoint/set_index std_msgs/Int32 "data: 3"   # 跳到第 3 个
rostopic pub -1 /waypoint/next std_msgs/Empty "{}"             # 下一个
rostopic pub -1 /waypoint/snap_nearest std_msgs/Empty "{}"     # 就近吸附
```

`sequential` 模式走完最后一个点后停止推进并在终端提示；这时可以随时用 `set_index`
重新指定目标，或改用 `mode:=loop` 循环跑。

### 路线文件

纯文本，每行一个点 `x y yaw`（`yaw` 可省略，默认 0），`#` 之后是注释：

```
# x y yaw
0.0   0.0   0.0
5.0   0.0   0.0
10.0  10.0  0.0
```

随包提供 `config/demo_route.txt` 作为格式样例（与内置示例路线一致）。用纯文本而不是
YAML，是为了让「读路线」不依赖 PyYAML，手改和离线测试都更省事。

### 离线测试

核心逻辑（路线解析、目标点推进、到达判定、模拟车辆步进、四元数互转）都在
`scripts/waypoint_core.py`，该文件**不导入 rospy**，因此可以在没有 ROS、没有 CARLA、
没有图形界面的机器上回归：

```bash
cd <仓库根目录>
python3 tests/test_waypoint_publisher_local.py   # Noetic / 任意 Python 3
python2 tests/test_waypoint_publisher_local.py   # Melodic / Python 2.7
```

全部通过时打印 `全部通过` 并以 0 退出。

### 运行效果

`roslaunch waypoint_publisher main.launch mock_enabled:=true` 启动后，终端会按到达
顺序打印目标点切换过程（内置路线的 8 个点依次推进，`sequential` 模式走完后停止）：

```
[INFO] 路径点发布器已启动：8 个路径点，模式=sequential，到达阈值=1.00m，频率=10.0Hz
[INFO] 内置模拟车辆已启用：/odom 由本节点发布，速度 2.0 m/s
[INFO] 到达第 1 个路径点 → 目标切换为第 2 个
[INFO] 到达第 2 个路径点 → 目标切换为第 3 个
...
[INFO] 已到达最后一个路径点，路线结束（可发 /waypoint/set_index 重新指定）
```

在 rviz 里可以同时显示 `PoseArray`（整条路线）与 `PoseStamped`（当前目标点），
目标点会随着推进一格一格跳过去。

## 拓展：自行实现「给定轨迹的神经网络跟踪控制」

本文的路径点发布器只负责**发布**路径点，车辆如何沿路径点行驶由其它节点决定。
下面是在该功能基础上**自主实现**的一套「传感器感知 + 给定轨迹跟踪」控制，
代码位于
[`src/ground/carla_perception_control`](https://github.com/OpenHUTB/ros2/tree/master/src/ground/carla_perception_control)，
详细介绍（计算原理、源码解析、性能评价）见
[传感器感知与给定轨迹跟踪（神经网络版）](./carla_perception_control.md)。

### 与本文示例的区别

| 对比项 | 本文示例（路径点发布器） | 本拓展模块 |
|---|---|---|
| 技术路线 | `carla_ros_bridge` 的 waypoint publisher 发布路径点话题 | CARLA Python API 直连 + 自研感知/控制节点 |
| 是否依赖 ros-bridge | 必须编译并运行 ros-bridge | **不需要**，仅需 `carla` Python 客户端 |
| 功能范围 | 只发布路径点，不含控制 | 完整闭环：传感器 → 感知 → 控制 → 车辆 |
| 感知环节 | 无 | **感知神经网络**：RGB + 深度 + 雷达 → 障碍方向 |
| 控制环节 | 由外部节点自行订阅并控制 | **控制神经网络**：状态 → 转向角 |
| 算法性质 | 地图几何查询 | 两个可训练神经网络（纯 numpy 反向传播） |
| 无 CARLA / 无图形界面时 | 无法运行 | `--headless --demo` 离线自证并导出曲线图 |

### 复用本文的配置步骤

CARLA 服务端的启动、宿主机 IP 与端口 2000 的查看、虚拟机网络设置等步骤
**与已有示例完全一致，此处不再重复**，请参考
[设置并连接到 Carla 模拟器](../set_up_and_connect_to_carla.md) 的
「启动 Carla 服务器」与「使用 Carla 客户端启动 Ego Vehicle」小节；
连接失败的排查见同一篇的「常见问题」小节。

!!! warning "不要从头到尾照做已有示例"
    已有示例中的「设置 Carla ROS Bridge」「使用 rviz 进行可视化」等小节属于
    **ros-bridge 路线**（需要 catkin 编译 ros-bridge）。本拓展模块**不依赖 ros-bridge**，
    这些步骤可以跳过。

### 本拓展新增的内容

1. **感知神经网络**：三路传感器（RGB 相机、深度相机、32 线激光雷达）压缩为 4 维归一化特征，
   由 MLP 分类器（4→12→3）输出障碍方向（无目标 / 偏左 / 偏右）。
2. **控制神经网络**：状态 (航向差, 前视距离) → 转向角，MLP 策略网络（2→32→1），
   tanh 输出保证转向落在 \([-1,1]\)。
3. **几何监督标签**：控制网络的训练标签由纯跟踪几何律解析生成，
   免去人工标注。
4. **前视追踪 + 路点加密 + 垂距误差**：把稀疏给定轨迹加密到 2 m 间距，
   用前视点选取目标，用点到折线垂距度量横向误差。
5. **弯道自适应减速 + 终点停车**：按航向差自动降油门，进入终点 5 m 内刹车。
6. **离线取证模式**：`--headless --demo` 无需 CARLA、无需图形界面即可跑通全链路，
   并导出损失/误差曲线 PNG。

### 运行本拓展模块

除本文所需环境外，只需补装 CARLA 0.9.16 的 Python 客户端
（神经网络为纯 numpy 实现，无需 TensorFlow/PyTorch）：

```shell
pip3 install <CARLA>/PythonAPI/carla/dist/carla-0.9.16-cp310-cp310-manylinux_2_31_x86_64.whl
```

离线训练两个神经网络（**不需要 CARLA**）：

```shell
python3 src/ground/carla_perception_control/main.py --mode train --epochs 300
```

在线感知 + 沿给定轨迹跟踪（`--host` 的填法与本文一致：填宿主机 IP）。
**省略 `--waypoints` 即使用模块内置的 `DEMO_ROUTE`**
（26 个航点，已逐点校验在车道上，含两个约 90° 弯）：

```shell
python3 src/ground/carla_perception_control/main.py --mode run \
        --host 172.21.108.47 --sim_time 90 --save_dir ~/shots
```

!!! warning "给定轨迹务必落在车道上、且弯道处加密"
    航点之间是直线连接：航点若在路面外，车会开出路面撞停；
    转弯处航点间距过大会让车来不及转弯（前视距离仅 6 m）。
    详见 [传感器感知与给定轨迹跟踪（神经网络版）](./carla_perception_control.md) 5.7 节。

使用 launch 启动（ROS 2 / ROS 1）：

```shell
# ROS 2
ros2 launch carla_perception_control main.launch.py host:=172.21.108.47
# ROS 1
roslaunch carla_perception_control main.launch host:=172.21.108.47
```

### 实测结果

离线取证模式实测（`--epochs 300 --sim_time 90`，无需 CARLA）：

| 指标 | 数值 |
|---|---|
| 感知 NN 训练集准确率 | 0.995 |
| 控制 NN 训练集 MSE | 0.00303 |
| 横向误差 RMSE | **0.249 m**（含两个约 90° 弯） |

在线连接 CARLA 服务端实测（`--mode run --sim_time 90`，Town05）：

| 指标 | 数值 |
|---|---|
| 横向误差 RMSE | **1.012 m** |
| 是否到达终点 | 是（t=28.9 s） |
| 相机帧数 / 导出截图 | 577 帧 / 28 张 |

![横向误差随时间的收敛曲线](../img/ground/carla_lateral_error.png)

![控制神经网络输出的转向指令](../img/ground/carla_steer_cmd.png)

完整的调优过程与性能评价见
[传感器感知与给定轨迹跟踪（神经网络版）](./carla_perception_control.md) 第 6 节。
