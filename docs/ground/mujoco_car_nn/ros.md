# ROS 部署

同一套 MuJoCo 仿真与策略推理被封装为 ROS 1（Noetic）和 ROS 2（Humble）两个包，
满足"所有程序必须支持 launch 启动，且在 Ubuntu 20.04 ROS Noetic / Humble 上运行通过"的要求。

## 1. 工作空间布局

```text
catkin_ws/src/
├── car_nn_sim/          # 本仓库 src/ground/mujoco_car_nn/ros1/car_nn_sim
└── ROS/                 # 本仓库根目录（提供 mujoco_car_nn Python 包）
```

其中 Python 包位于 `src/ground/mujoco_car_nn/src/mujoco_car_nn`。

节点启动时按下面的顺序查找 `mujoco_car_nn` 包，找到第一个就把它加入 `sys.path`：

1. 环境变量 `CAR_NN_SRC`（指向包含 `mujoco_car_nn/` 的目录）；
2. 源文件所在位置按 `realpath` 解析后的相对路径（`colcon build --symlink-install` 时有效）；
3. 当前目录下的 `src/ground/mujoco_car_nn/src`、`src`。

```python
here = os.path.dirname(os.path.realpath(__file__))
candidates = [os.environ.get("CAR_NN_SRC"), os.path.join(here, "..", "..", "..", "src"), ...]
```

三种情况都可以正常工作：

| 场景 | 做法 |
|---|---|
| 源码树直接运行（`python -m car_nn_sim.sim_node`） | 无需配置，按相对路径找到 |
| `colcon build --symlink-install` 后运行 | 无需配置，`realpath` 会指回源码 |
| 普通 `colcon build`（文件被复制到 `install/`） | 设置 `export CAR_NN_SRC=<repo>/src/ground/mujoco_car_nn/src`，或 `pip install -e <repo>/src/ground/mujoco_car_nn/src` |

找不到包时节点会打印上述提示并退出，而不是抛出 `ModuleNotFoundError`。

## 2. ROS 1（Noetic）

### 2.1 依赖

```bash
sudo apt install ros-noetic-teleop-twist-keyboard ros-noetic-tf2-ros
pip3 install mujoco torch numpy pillow
```

### 2.2 编译

```bash
cd ~/catkin_ws/src
ln -s /path/to/ROS ros_car_nn
cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

### 2.3 启动

| 命令 | 功能 |
|---|---|
| `roslaunch car_nn_sim car_teleop.launch` | 键盘遥控（W/A/S/D） |
| `roslaunch car_nn_sim car_nn.launch policy:=/abs/path/cnn_dagger.pt` | 神经网络自动驾驶 |
| `roslaunch car_nn_sim car_slam.launch` | 建图 + 定位（发布 `/map` 与 TF） |

参数：

| 参数 | 默认 | 说明 |
|---|---|---|
| `~scene` | `day` | `day` / `dusk` / `fog` |
| `~policy` | 空 | 权重路径；为空则等待 `/car/cmd_vel` |
| `~rate` | 20.0 | 定时器频率 Hz（MuJoCo 渲染受限，WSL 实测约 3.5–4 Hz，见 §2.6） |
| `~offset` | 0.0 | 初始横向偏移（m） |

### 2.4 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/car/camera/image_raw` | `sensor_msgs/Image` | 发布 | 64×64 rgb8 第一视角 |
| `/car/scan` | `sensor_msgs/LaserScan` | 发布 | 360°，64 束，量程 20 m |
| `/car/odom` | `nav_msgs/Odometry` | 发布 | 真值位姿 + 速度 |
| `/car/cmd_vel` | `geometry_msgs/Twist` | 订阅 | `linear.x` m/s → 油门，`angular.z` → 转向 |
| `/map` | `nav_msgs/OccupancyGrid` | 发布 | 占据栅格（latched） |
| TF `odom→base_link` | `tf2` | 发布 | 仿真位姿（`sim_node`） |
| TF `base_link→camera_link` | `tf2` | 发布 | 相机安装位置 (1.45, 0, 1.40)（静态） |
| TF `base_link→laser_link` | `tf2` | 发布 | 雷达安装位置 (0, 0, 0.60)（静态） |
| TF `map→base_link` | `tf2` | 发布 | SLAM 估计位姿（`slam_node`） |

`sim_node` 与 `slam_node` 同时运行时，`map→base_link→{camera_link, laser_link}`
组成完整的 TF 链，RViz 的 Fixed Frame 设为 `map` 后可直接显示 `/car/scan`。

## 3. ROS 2（Humble）

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
ln -s <repo>/src/ground/mujoco_car_nn/ros2/car_nn_sim car_nn_sim
cd ~/ros2_ws
colcon build --symlink-install --packages-select car_nn_sim
source install/setup.bash
```

启动：

```bash
ros2 launch car_nn_sim car_teleop.launch.py
ros2 launch car_nn_sim car_nn.launch.py policy:=/abs/path/cnn_dagger.pt scene:=dusk
ros2 launch car_nn_sim car_slam.launch.py
```

ROS 2 节点用 `create_timer(1/rate)` 驱动仿真，话题名与 ROS 1 完全一致，便于对比。

> **实际频率**：`~rate` 只是定时器的标称值。每次回调要做一次 MuJoCo 渲染 + 64 束射线，
> 在 WSL2 的 EGL 软件渲染下实测 `/car/camera/image_raw` 约 **3.8 Hz**、`/car/scan` 约
> **3.4 Hz**（`src/ground/mujoco_car_nn/evidence/10_ros2/ros2_run.log`）；Windows 原生
> MuJoCo 可接近 20 Hz。做数据对齐时请以 `ros2 topic hz` 的实测值为准。

### 3.1 Docker（免安装 ROS）

```bash
docker build -t car_nn .
docker run --rm -it car_nn
# 容器内：
source /opt/ros/humble/setup.bash && source /workspace/ros2_ws/install/setup.bash
ros2 launch car_nn_sim car_teleop.launch.py
```

镜像基于 `ros:humble-ros-base`，安装 CPU 版 `torch`/`mujoco` 并设置 `MUJOCO_GL=egl`，
无显示器也能离屏渲染；`colcon build` 已在构建阶段完成。

## 4. 在 RViz 中观察

1. `Fixed Frame` 设为 `map`；
2. 添加 `LaserScan`（topic `/car/scan`）、`Image`（`/car/camera/image_raw`）、
   `Map`（`/map`）、`Odometry`（`/car/odom`）；
3. 建图时可以看到红色雷达点沿护栏铺开、灰色未知区域逐渐被白色自由空间填满。

## 5. 无 ROS 的等价运行

课程检查时如果没有 ROS 环境，直接用 `main.py` 即可得到同样的功能与指标：

```bash
python main.py eval --model runs/cnn_dagger.pt --laps 2 --record
python main.py slam --laps 1
```

ROS 节点的**运行时冒烟测试**（不需要安装 ROS）：

```bash
python -m pytest tests/test_ros_nodes.py -q
```

该测试用 `rospy`/`rclpy` 桩模块真实实例化节点、跑 3 个控制周期，校验
图像字节数、雷达 64 束、`odom` frame 以及 `cmd_vel` 能驱动车辆加速，
可以在没有 ROS 的 Windows 上提前发现消息字段错误。

## 6. 文档构建

文档使用课程提供的 **hutb-doc**（OpenHUTB 维护的 mkdocs 分支）构建，已实测兼容：

```bash
pip install hutb-doc mkdocs-material pymdown-extensions
mkdocs build --strict          # 输出 site/，可直接发布 GitHub Pages
```

`docs/report.docx` 由 `scripts/build_report.py` 调用 pandoc 生成（目录、章节编号、
公式、表格、图片齐全），作用等价于 Markdown2Anything 的 Markdown → Word 导出。

## 7. 常见问题

??? question "`ImportError: No module named mujoco_car_nn`"
    确认 `catkin_ws/src/ROS` 软链接存在；节点是按 `ros1/car_nn_sim/scripts/../../../src`
    推算仓库位置的。若目录结构不同，设置 `PYTHONPATH`。

??? question "虚拟机里 MuJoCo 报 GL 错误"
    课程虚拟机是 Ubuntu 20.04，装有 Mesa。若离屏渲染失败，安装 `libosmesa6` 并
    `export MUJOCO_GL=osmesa`；有桌面会话时默认 GLFW 即可。

??? question "launch 后车子不动"
    `car_teleop.launch` 需要把焦点放在 teleop 终端上按键；`car_nn.launch` 需要
    `policy` 参数指向存在的 `.pt` 文件，否则节点只发布传感器、等待 `cmd_vel`。
