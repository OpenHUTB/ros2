# dave_underwater_sim

DAVE（DAVE Aquatic Virtual Environment）水下机器人仿真环境的一键部署与演示模块。

- 上游项目：<https://github.com/Field-Robotics-Lab/dave>（Apache-2.0）
- 技术栈：Ubuntu 20.04 + ROS Noetic + Gazebo 11（DAVE 官方推荐组合）
- 模块目标：在课堂/实验环境中快速复现水下 ROV（REXROV）+ Oberon7 机械臂的仿真，
  并提供机械臂操控、水下相机采集与截图脚本。

## 1. 模块组成

| 文件 | 说明 |
|---|---|
| `main.sh` | 一键入口：检查工作区并启动演示 |
| `install_dave.sh` | 安装脚本：前置检查 + apt 依赖 + 按固定 commit 下载 DAVE 及依赖 + catkin 构建演示所需包 |
| `run_demo.sh` | 演示启动：`demo` / `valve` / `plug`，支持 `headless` |
| `scripts/dave_arm_move.py` | 发布 Oberon7 六关节 effort 指令与夹爪开合指令，演示机械臂动作 |
| `scripts/dave_camseq.py` | 连续保存水下相机帧（用于生成 GIF） |
| `scripts/dave_save_camera.py` | 保存单帧水下相机图像 |

## 2. 前置条件

`install_dave.sh` 只安装额外 apt 依赖并构建 DAVE 演示包，**不安装 ROS 本体**，请先准备：

- Ubuntu 20.04（22.04 请使用 DAVE 官方 Docker 方案）；
- **ROS Noetic desktop-full**（`/opt/ros/noetic/setup.bash` 存在）+ **Gazebo 11**
  （官方安装：<http://wiki.ros.org/noetic/Installation/Ubuntu>）；
- 4 GB 以上内存（VMware 实测 4 GB 可运行；带界面软件渲染实时因子约 0.3~0.5）；
- 约 5 GB 磁盘空间（源码 + 编译产物）；
- 安装脚本使用 `curl` 下载 tarball、构建时需 `catkin_tools`/`rosdep`（脚本会自动安装缺失的 apt 包）。

## 3. 依赖的外部仓库（由 install 脚本按固定 commit 下载）

`dave`、`uuv_simulator`、`uuv_manipulators`、`rexrov2`、`eca_a9`、`ds_msgs`、
`ds_sim`、`dockwater`，下载地址已固定到实测可用的 commit（见 `install_dave.sh` 中
`dl` 调用），目录结构为：

```
~/uuv_ws/src/
├── dave/                    # 仿真世界/模型/插件/demo 启动
├── uuv_simulator/           # UUV 动力学、控制器、传感器插件
├── uuv_manipulators/        # Oberon7 机械臂描述与控制
├── rexrov2/                 # REXROV 机器人描述
├── eca_a9/  ds_msgs/  ds_sim/  dockwater/
```

## 4. 使用

```bash
# 首次安装（约 10~20 分钟，视网络情况；构建 44 个包）
bash install_dave.sh

# 启动演示
bash main.sh                 # 基础场景：REXROV + 机械臂 + 海浪海底世界
bash main.sh valve           # BOP 防喷器面板作业场景（对应参考视频）
bash main.sh demo headless   # 无界面（gzserver），适合无 GPU 的服务器

# 演示机械臂动作（另开终端，先 source 工作区）
source /opt/ros/noetic/setup.bash
source ~/uuv_ws/devel/setup.bash
python3 scripts/dave_arm_move.py
python3 scripts/dave_save_camera.py 1   # 保存一帧水下相机图像到 /tmp/valve_cam_1.png
python3 scripts/dave_camseq.py          # 连续保存到 /tmp/camseq/f_*.png（制作 GIF 用）
```

如果同时装有 ROS 2，也可用 ament_python 封装启动（需要 ROS 1 + ROS 2 环境共存，
脚本内部会 source ROS 1）：

```bash
ros2 launch dave_underwater_sim main.launch.py            # demo 场景
ros2 launch dave_underwater_sim main.launch.py mode:=valve
```

## 5. 已验证的话题（VM 实测，19 个节点 / 48 个话题）

| 话题 | 类型 | 说明 |
|---|---|---|
| `/rexrov/cmd_vel` | `geometry_msgs/Twist` | ROV 速度控制 |
| `/rexrov/imu` | `sensor_msgs/Imu` | IMU |
| `/rexrov/dvl` | `uuv_sensor_ros_plugins_msgs/DVL` | DVL 速度 |
| `/rexrov/dvl_sonar0~3` | `sensor_msgs/Range` | DVL 声呐波束 |
| `/rexrov/pressure` | `sensor_msgs/FluidPressure` | 压力/深度 |
| `/rexrov/joint_states` | `sensor_msgs/JointState` | 机械臂/推进器关节状态 |
| `/rexrov/oberon7/<joint>/controller/command` | `std_msgs/Float64` | 机械臂 effort 指令 |
| `/rexrov/oberon7/finger_left_joint/controller/command` 等 | `std_msgs/Float64` | 夹爪指令 |
| `/rexrov/rexrov/camera/camera_image` | `sensor_msgs/Image` | 水下相机（768×492） |

## 6. 已知限制

- 无独立 GPU 时带界面的仿真实时因子约 0.30~0.45（软件渲染，1920×1080 下更低）。
- 无摇杆环境下机械臂用 effort 指令控制，动作幅度有限；完整的插拔/面板作业建议使用
  Logitech F310 摇杆与 `oberon7_moveit`/`dave_nodes` 的 MoveIt 流程。
- `valve` 场景的 launch 会同时拉起 RViz 与 `rqt_image_view`（多窗口），无界面环境请用
  `demo headless` 或直接 `roslaunch` 时设 `gui:=false`。
- `plug` 场景未在本环境实测（需要 `plug_and_socket_description` 等额外模型，已包含在构建
  列表中），如不可用请以 `demo`/`valve` 为准。
- 本模块只依赖上游 DAVE 源码，不修改其核心代码；`install_dave.sh` 用 codeload
  tarball（固定 commit）下载以绕开 GitHub 直连不稳定的问题。

## 7. 大模型使用声明

本模块的安装脚本、运行脚本与文档由 AI 助手辅助整理，所有命令与实验数据均在本仓库
环境（Ubuntu 20.04 + ROS Noetic + Gazebo 11）中实际运行验证，作者已核对并对内容负责。
