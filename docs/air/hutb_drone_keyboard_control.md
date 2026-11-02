# (作业) 蒋宇鑫 2535030012 基于 OpenHUTB 的无人机仿真与键盘控制

## 1. 项目简介

本项目基于开源孪创（OpenHUTB）人机模拟器，实现了一个 **ROS 2 + 键盘遥控无人机** 的仿真模块：

- 通过 ROS 2 节点读取终端键盘输入，发布速度控制指令（`/cmd_vel`）；
- 桥接节点调用 OpenHUTB 的 AirSim 模式 Python API（`airsim.MultirotorClient`），将 ROS 速度指令实时转换为无人机速度控制，实现 **起飞、平移、升降、旋转、悬停** 的完整键盘操控；
- 同时发布无人机的 **定位信息**（`/drone/odom`，NED→ENU 转换）与 **碰撞检测状态**（`/drone/collision`），并在终端实时打印碰撞告警。

与仓库中已有模块的区别：

- 已有 [`uav_keyboard_control`](./uav_keyboard_control.md) 基于 **ROS 1**（roslaunch/catkin），且键盘节点与仿真器桥接（`carlair_ros_bridge`）分离为两个模块；
- 本模块面向 **ROS 2 Humble**，键盘遥控与 AirSim 桥接**合为单模块**，`main.py` 一键启动即可运行，无外部模块依赖；
- 在指令通道之外额外输出 **定位话题（NED→ENU 转换后的 Odometry）** 与 **碰撞检测话题**，便于上位机/导航算法直接消费。

模块架构如下：

```text
+------------------+   /cmd_vel    +-----------------+  AirSim RPC   +------------------+
| keyboard_teleop  | ------------> |  drone_client   | ------------> |  OpenHUTB 模拟器  |
|  (键盘遥控节点)   |    Twist      | (无人机桥接节点) |  :41451       |  (AIR 模式无人机)  |
+------------------+               +-----------------+               +------------------+
                                         |        ^
                    /drone/odom (Odometry)|        | /drone/collision (Bool)
                                         v        |
                                    定位输出与碰撞检测（由 drone_client 发布）
```

## 2. 运行环境

| 项目 | 版本/说明 |
| --- | --- |
| 操作系统 | Ubuntu 22.04 LTS |
| ROS 2 | Humble |
| OpenHUTB | v2.3.0 及以上（默认同时支持 Carla 模式和 AirSim 模式） |
| Python 客户端 | OpenHUTB 发布包自带 `hutb-*.whl`（提供 `airsim` 模块） |
| 硬件 | 建议独立显卡（NVIDIA GTX 1060 及以上），内存 16 GB |

## 3. 安装与配置

### 3.1 安装 OpenHUTB 模拟器

1. 使用官方下载工具下载并解压模拟器发行包；
2. 进入生成的 `hutb/` 目录，安装 Python 客户端：

```bash
cd hutb/PythonAPI/carla/dist/
pip install hutb-*.whl    # 根据自己的 Python 版本选择对应 whl，安装后可 import airsim
```

### 3.2 安装 ROS 2 Humble

按官方文档安装（本作业使用二进制安装即可）：

```bash
sudo apt install ros-humble-desktop
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3.3 获取本模块代码

将本模块的 `src/air/hutb_drone_keyboard_control/` 文件夹放到任意工作目录，例如：

```bash
mkdir -p ~/ros2_ws && cd ~/ros2_ws
# 把 src/air/hutb_drone_keyboard_control/ 下的 main.py、
# keyboard_teleop.py、drone_client.py 拷贝到此处
```

模块仅依赖 `rclpy`、`geometry_msgs`、`nav_msgs`、`std_msgs`（Humble 桌面版自带）与 `airsim`（3.1 步已安装），无需额外编译。

## 4. 运行步骤

共需 **两个终端**。

**终端 1：启动 OpenHUTB 模拟器（AIR 无人机模式）**

```bash
cd hutb
python config.py --map Town10HD?GAME=AIR
# 等待模拟器窗口打开，出现无人机场景后按回车继续
```

**终端 2：一键启动本模块（起飞 + 键盘遥控）**

```bash
cd ~/ros2_ws/src/hutb_drone_keyboard_control
python3 main.py
```

启动后可看到如下日志，随后无人机自动起飞进入悬停状态，按 `h` 可随时查看按键帮助：

```text
==========================================================
  基于 OpenHUTB 的无人机键盘控制仿真 (ROS 2 Humble)
  节点: keyboard_teleop + drone_client
==========================================================
[drone_client]: 正在连接 OpenHUTB 模拟器 127.0.0.1:41451 ...
[drone_client]: 连接成功, API 控制已接管
[drone_client]: 无人机自动起飞中...
[keyboard_teleop]: 键盘遥控节点已启动，话题: /cmd_vel
```

## 5. 按键说明

| 按键 | 功能 | 按键 | 功能 |
| --- | --- | --- | --- |
| `w` / `s` | 前进 / 后退 | `q` / `e` | 左旋 / 右旋 |
| `a` / `d` | 左移 / 右移 | `g` / `b` | 增大 / 减小速度步长 |
| `SPACE` | 上升 | `k` | 急停（速度清零悬停） |
| `c` | 下降 | `h` | 打印按键帮助 |

> 松开按键约 0.4 秒后速度自动衰减为 0（"松手即缓停"），持续按住按键可保持匀速运动，避免无人机失漂。

## 6. 话题说明

| 话题 | 类型 | 频率 | 说明 |
| --- | --- | --- | --- |
| `/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | 键盘速度指令（ENU：x 前、y 左、z 上、偏航逆时针为正） |
| `/drone/odom` | `nav_msgs/Odometry` | 5 Hz | 无人机定位与速度（已由模拟器 NED 转换为 ROS ENU） |
| `/drone/collision` | `std_msgs/Bool` | 1 Hz | 碰撞检测标志，`true` 表示发生碰撞，节点同时打印告警日志 |

查看话题数据：

```bash
ros2 topic echo /drone/odom
ros2 topic echo /drone/collision
ros2 topic hz /cmd_vel
```

## 7. 代码结构

```text
src/air/hutb_drone_keyboard_control/
├── main.py              # 一键启动入口: 同时拉起两个节点
├── keyboard_teleop.py   # 键盘遥控节点: termios 原始读键 -> /cmd_vel
└── drone_client.py      # 无人机桥接节点: /cmd_vel -> airsim.MultirotorClient
```

- `keyboard_teleop.py`：使用 `termios` 将终端切为 cbreak 模式读取按键，后台线程根据按键调整目标速度（含限幅与步长调节），主线程按 10 Hz 发布 `/cmd_vel`；
- `drone_client.py`：连接 OpenHUTB（AIR 模式 RPC 端口 41451），自动 `armDisarm` + `takeoffAsync` 起飞；10 Hz 控制循环将 `/cmd_vel` 经 **ENU→NED 转换** 后调用 `moveByVelocityAsync`（`YawMode(is_rate=True)` 实现角速度旋转）；指令超时 0.5 秒自动 `hoverAsync` 悬停；5 Hz 发布定位、1 Hz 检测碰撞。

## 8. 运行效果

> 以下截图/动图请在本机运行后替换（截图用 .png 且 < 3 MB，动图用 ScreenToGif 录制 .gif 且 < 10 MB）。

【运行截图 1：模拟器 AIR 模式场景 + 无人机起飞瞬间】

【运行截图 2：键盘控制无人机平移/旋转过程中的模拟器画面】

【运行截图 3：终端运行日志（连接成功、起飞、碰撞告警）】

【运行动图：完整键盘操控过程（起飞 → 前进 → 上升 → 左旋 → 下降 → 急停悬停）】

【运行截图 4：`ros2 topic echo /drone/odom` 的定位数据与 `rqt_graph` 节点关系图】

## 9. 常见问题

1. **连接失败（confirmConnection 一直等待）**：确认模拟器已用 `?GAME=AIR` 启动，且端口未被占用；可用 `--host_ip` / `host_port` 参数修改地址。
2. **按键无反应**：键盘节点只支持 Linux 终端（termios），需在真实终端中运行 `main.py`；部分终端需保持窗口焦点。
3. **无人机漂移**：松手后 0.4 秒速度才会衰减为 0，属正常减速过程；也可按 `k` 立即急停悬停。
4. **碰撞后异常**：模块默认碰撞后仅告警不强制降落，可按 `k` 悬停后继续飞行，或重启模块重新起飞。

## 10. 参考链接

- OpenHUTB 项目主页：<https://openhutb.github.io/>
- OpenHUTB 空地一体仿真文档：<https://openhutb.github.io/air_doc/dev/Carla_Air/>
- MultirotorClient API 文档：<https://openhutb.github.io/doc/python_api/>
- ROS/ROS2 示例仓库（本作业提交仓库）：<https://github.com/OpenHUTB/ros2>
- hello_drone 官方示例：<https://github.com/OpenHUTB/air/blob/main/PythonClient/multirotor/hello_drone.py>

## 11. 大模型使用声明

本文档与代码在编写过程中使用了大模型工具（WorkBuddy）辅助生成，作者已逐行核对全部内容，理解其原理并实际运行验证，对提交内容负全部责任。
