# uav_mujoco_sim

MuJoCo 3.12 四旋翼仿真底座，把物理仿真与多路传感器以 **标准 ROS2 Humble 话题** 暴露出来。

本包是其余五个无人机模块（神经残差控制、神经 VIO/SLAM、语义建图导航、深度图端到端竞速、
神经安全过滤器）的共同地基，并单独完成任务①：**通过键盘对无人机进行运动控制**。

> **不使用 Gazebo。** 仿真器为 MuJoCo 3.12.0，纯 CPU 运行，不需要独立显卡。

---

## 1. 为什么是 MuJoCo + 双进程

ROS2 Humble 的 `rclpy` 只有 **Python 3.8** 绑定，而 MuJoCo 3.12.0 要求 **Python ≥ 3.10**，
两者无法共存于同一解释器，因此拆成两个进程：

```
┌──────────────────────────────┐          ┌───────────────────────────────┐
│  仿真进程 (Python ≥3.10)      │  TCP     │  ROS2 进程 (Python 3.8)        │
│  MuJoCo 3.12 物理 + 传感器    │◀────────▶│  uav_sim_bridge               │
│  main.py --server            │  41460   │  /uav/odom  /camera/* /lidar  │
└──────────────────────────────┘          └───────────────────────────────┘
```

架构与既有的 `carlair_ros_bridge` 一致（仿真器一边、ROS 一边、中间走 TCP）。

**关键设计**：话题契约与 `carlair_ros_bridge` **逐字段对齐**，上游算法模块
（`uav_ppo_nav`、`air_learning`、`octree_uav_3d_pathfinding` 等）无需改动即可切换；
把仿真器换成 AirSim/CarlaAir 时算法代码也不动。

---

## 2. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 发布 | `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度（ENU：x 东、y 北、z 上） |
| 发布 | `/tf` | `tf2_msgs/TFMessage` | `world → base_link` |
| 发布 | `/camera/image_raw` | `sensor_msgs/Image` | 前视彩色图 `bgr8` |
| 发布 | `/camera/depth` | `sensor_msgs/Image` | 深度图 `32FC1`，单位米，量程 0.2~10 m |
| 发布 | `/camera/seg` | `sensor_msgs/Image` | 语义分割伪彩色 `bgr8`（默认关闭） |
| 发布 | `/lidar/points` | `sensor_msgs/PointCloud2` | XYZ float32，世界系 ENU |
| 发布 | `/uav/status` | `std_msgs/String` | `READY / HOVER / VELOCITY / GOAL / GOAL_DONE / ROTOR` |
| 订阅 | `/uav/cmd_vel` | `geometry_msgs/Twist` | 速度指令，默认机体系（x 前、y 左、z 上） |
| 订阅 | `/uav/goal` | `geometry_msgs/Point` | 目标点（ENU 世界系） |

**安全保护**：速度自动限幅；超过 `cmd_timeout`（默认 0.5 s）没有新指令即自动悬停。

---

## 3. 运行环境

| 项 | 要求 |
|---|---|
| ROS | ROS2 **Humble**（Ubuntu 20.04/22.04 均可） |
| 仿真侧 Python | **≥ 3.10**（MuJoCo 3.12.0 的硬性下限） |
| 显卡 | **不需要独立显卡**。核显/纯 CPU 即可（Intel 核显实测 ~85 FPS @160×120 RGB） |
| 内存 | ≥ 4 GB |
| 其他 | 不需要 Gazebo、不需要 Unreal、不需要 CUDA |

实测性能（Intel 核显笔记本，640×480 相机，72 线雷达）：

| 项目 | 数值 |
|---|---|
| 物理仿真 | 约 3400 Hz 控制回路（要求 ≥100 Hz） |
| RGB 320×240 | 12 ms/帧（82 Hz） |
| 深度 320×240 | 21 ms/帧（49 Hz） |
| 语义分割 320×240 | 17 ms/帧（59 Hz） |
| 72 线激光雷达 | 1.9 ms/帧（519 Hz） |
| 四路传感器合计 | 47 ms/帧（21 Hz） |

---

## 4. 安装

### 4.1 仿真侧（一次性）

```bash
# 装 uv（只装到 ~/.local/bin，不动系统 Python）
curl -LsSf https://astral.sh/uv/install.sh | sh
export PATH="$HOME/.local/bin:$PATH"

# 建一个 Python 3.12 环境并装 MuJoCo
uv venv ~/uav_ws/.venv --python 3.12
uv pip install --python ~/uav_ws/.venv/bin/python "mujoco==3.12.0" numpy
```

### 4.2 ROS 侧

```bash
source /opt/ros/humble/setup.bash
mkdir -p ~/uav_ws/ros2_ws/src && cd ~/uav_ws/ros2_ws
# 把本包放进 src/ 后：
colcon build --symlink-install --packages-select uav_mujoco_sim
source install/setup.bash
```

---

## 5. 运行步骤

### 5.1 先跑自检

不需要 ROS 与显示器，用于确认环境与代码正确：

```bash
cd ~/uav_ws/ros2_ws/src/uav_mujoco_sim
~/uav_ws/.venv/bin/python main.py --selftest
```

期望输出 `通过 20/20`。

### 5.2 一键启动（ROS2 launch）

```bash
source /opt/ros/humble/setup.bash
source ~/uav_ws/ros2_ws/install/setup.bash
export DISPLAY=:0            # 无显示环境见 5.5

ros2 launch uav_mujoco_sim main.launch.py
```

launch 会自动：探测仿真侧 Python → 启动 MuJoCo 服务端 → 启动桥接节点。
键盘遥控节点默认不启动（见 5.3）。

常用参数：

```bash
ros2 launch uav_mujoco_sim main.launch.py scene:=urban_block
ros2 launch uav_mujoco_sim main.launch.py keyboard:=true     # 键盘遥控，见 5.3
ros2 launch uav_mujoco_sim main.launch.py quality:=full     # 开阴影，录动图用
ros2 launch uav_mujoco_sim main.launch.py headless:=true    # 不渲染，纯控制/规划
```

### 5.3 键盘遥控（任务①）

键盘节点要独占一个**前台**终端，因此**默认不随 launch 启动**：终端只把输入交给前台
进程组，launch 拉起的子进程不在其中，读终端拿到 `EIO`，按键没反应也不报错。

遥控时另开一个终端（终端 1 跑 5.2 的 launch，终端 2 跑键盘节点）：

```bash
source /opt/ros/humble/setup.bash
source ~/uav_ws/ros2_ws/install/setup.bash
ros2 run uav_mujoco_sim keyboard
```

该终端要让程序留在前台——别用管道、重定向、`nohup`、`&` 启动。

按键：

| 按键 | 动作 | 按键 | 动作 |
|---|---|---|---|
| `W` / `S` | 前进 / 后退 | `A` / `D` | 左移 / 右移 |
| `R` / `F` | 上升 / 下降 | `Q` / `E` | 右转 / 左转 |
| 松开所有键 | 悬停 | `ESC` / `Ctrl-C` | 退出 |

### 5.4 不用 ROS 的独立演示

只看效果或复现时**不需要 ROS**：

```bash
~/uav_ws/.venv/bin/python main.py --demo --scene obstacle_field --viewer
```

### 5.5 无显示环境（服务器 / 无 X 的容器）

```bash
export MUJOCO_GL=osmesa      # 需先 apt install libosmesa6-dev
# 或用虚拟显示：
xvfb-run -a python main.py --selftest
```

有 X 的虚拟机（本课程虚拟机即如此）用：

```bash
export DISPLAY=:0
export MUJOCO_GL=glfw
```

---

## 6. 验证话题是否正常

```bash
ros2 topic list
ros2 topic hz /uav/odom                       # 期望 ≈50 Hz
ros2 topic echo /uav/status --once
ros2 topic pub --once /uav/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 1.0}}"                        # 让无人机前进 1 m/s
ros2 topic pub --once /uav/goal geometry_msgs/msg/Point \
  "{x: 5.0, y: 0.0, z: 2.0}"                  # 飞向目标点
```

---

## 7. 目录结构

```
uav_mujoco_sim/
├── main.py                  # 统一入口（作业要求入口以 main 开头）
├── launch/main.launch.py    # ros2 launch 入口
├── package.xml              # ament_python 包定义
├── setup.py / setup.cfg
├── config/sim.yaml          # 桥接参数（launch 可覆盖）
├── models/quadrotor.xml     # 四旋翼模型（唯一的机体定义）
├── resource/uav_mujoco_sim  # ament 资源索引
├── uav_mujoco_sim/
│   ├── cli.py               # 入口实现（main.py 的薄封装调用它）
│   ├── selftest.py          # 内置自检
│   ├── demo.py              # 无 ROS 演示 / 录制 / 回放
│   ├── core/                # ← 纯 Python，无 ROS、无 TCP
│   │   ├── quadrotor.py     #   模型、混控、几何控制器
│   │   ├── scenes.py        #   场景库（5 个场景）
│   │   ├── sensors.py       #   相机/深度/语义分割/雷达/IMU
│   │   ├── teleop.py        #   按键映射（ROS 节点与演示共用）
│   │   ├── terminal.py      #   终端原始按键读取
│   │   └── paths.py         #   资源定位（源码布局 & 安装布局）
│   ├── sim/                 # ← TCP 仿真服务端与客户端
│   │   ├── protocol.py      #   线协议（JSON 头 + 二进制载荷）
│   │   ├── server.py        #   MuJoCo 服务端
│   │   └── client.py        #   客户端
│   └── ros/                 # ← ROS2 节点层（Python 3.8 兼容）
│       ├── bridge_node.py   #   桥接节点
│       ├── keyboard_node.py #   键盘遥控节点（任务①）
│       └── conversions.py   #   numpy ↔ ROS 消息
├── data/sample/             # 少量示例数据（离线复现用）
└── test/                    # 冒烟测试
```

`core/` 与 `sim/` 不依赖 ROS，无 ROS 的机器也能复现全部算法与仿真结果。

---

## 8. 场景

| 场景 | 内容 | 用途 |
|---|---|---|
| `open_ground` | 开阔平地 | 控制类实验、悬停与轨迹跟踪 |
| `obstacle_field` | 20 个随机障碍（种子可调） | 避障、导航、安全过滤器 |
| `corridor` | 室内走廊 + 立柱 | 视觉里程计、SLAM |
| `gate_track` | 8 个穿越门 | 端到端竞速 |
| `urban_block` | 14 栋建筑 + 10 棵树 + 6 辆车 | 语义建图与语义导航 |

场景几何的**名前缀**决定语义分割类别（`building_*` → 建筑、`tree_*` → 植被、
`car_*` → 车辆…）；新增场景遵守命名规则即可自动生效。

---

## 9. 已知限制

- 仿真侧与 ROS 侧须能互相访问 TCP 端口。仿真在 Windows 宿主机、ROS 在虚拟机时，
  把 `config/sim.yaml` 的 `sim/host` 改成宿主机的虚拟网段地址
  （VMware NAT 常见 `192.168.94.1`，VirtualBox NAT 固定 `10.0.2.2`）。
- 默认 `quality:=fast` 关闭阴影/反射/天空盒，只影响观感，不影响深度与语义分割，
  但采数据与演示应使用同一 quality。
- 深度图量程默认 10 m，超出置 0（模拟无效像素）。
