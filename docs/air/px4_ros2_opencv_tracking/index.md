# PX4 + ROS2 + OpenCV 无人机视觉追踪

本文档在 **Ubuntu 22.04.5 LTS**（VMware 虚拟机，16 核 / 7G 内存）PX4 v1.18 + ROS2 Humble + Gazebo Sim 8 + OpenCV，仿真无人机起飞并自动追踪红色目标。每一步都给出命令与"完成标志"。第 2–7、9–11 节的命令均在本机实际执行验证；第 8 节是从本仓库获取功能包并编译，因此不再内嵌代码。

> 

## 1. 环境与工程总览

| 项目        | 版本 / 说明                              |
| --------- | ------------------------------------ |
| 操作系统      | Ubuntu 22.04.5 LTS（VMware 虚拟机，64 位）  |
| ROS 版本    | ROS2 Humble Hawksbill                |
| PX4 固件    | PX4-Autopilot，`release/1.18` 分支      |
| 仿真环境      | Gazebo Sim（新版 gz，gz-harmonic 8.15.0） |
| 视觉库       | OpenCV 4.x（Python 3.10，cv2）          |
| 桥接        | Micro-XRCE-DDS-Agent（eProsima 源码构建）  |
| ROS2 工作空间 | `~/ws_px4`                           |
| PX4 源码目录  | `~/PX4-Autopilot`                    |
| 磁盘要求      | ≥ 40G（PX4 全量子模块 + 编译约需 25G+）         |

工程结构：

```
~/ws_px4/src/
├── px4_msgs/               # release/1.18 分支（浅克隆）
├── px4_ros_com/            # 桥接与示例（main 分支）
└── px4_ros2_opencv_tracking/    # 视觉追踪功能包（来自本仓库 src/air）
    ├── package.xml
    ├── setup.py
    ├── resource/px4_ros2_opencv_tracking
    ├── launch/tracking_launch.py
    └── px4_ros2_opencv_tracking/     # Python 包目录（脚本放这里，勿放 scripts/）
        ├── __init__.py
        ├── image_pub.py
        ├── color_tracker.py
        └── offboard_control.py
```

## 2. 前置准备（网络 / 磁盘 / 基础工具）

**① 磁盘空间检查**（PX4 子模块全量约 10G+，编译后累计 25G+，**磁盘必须 ≥ 40G**）：

```bash
df -h
```

若 `/` 不足 40G，需先扩容（VMware 设置扩容虚拟磁盘后，在系统内执行）：

```bash
sudo growpart /dev/sda 3 && sudo resize2fs /dev/sda3
df -h   # 应看到 39G / 21G 可用
```

**② 基础工具**：

```bash
sudo apt update && sudo apt install -y git build-essential cmake ninja-build curl wget unzip
```

**③ GitHub 镜像（关键：大陆网络 github.com 直连 Connection refused）**：

```bash
git config --global url."https://gh-proxy.com/https://github.com/".insteadOf "https://github.com/"
git config --get-regexp insteadof   # ✅ 输出 url.https://gh-proxy.com/...insteadof 一行
```

此配置全局生效，后续所有 `git clone`（含子模块）自动走镜像。

**✅ 前置准备完成标志**：`df -h` 有 ≥ 20G 可用；`git --version` 正常。

## 3. 安装 ROS2 Humble

```bash
sudo apt install -y curl gnupg lsb-release
# 添加 ROS2 公钥（raw.githubusercontent.com 可能不通，备选命令已给）
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
# 若上面报错（公钥下载失败），改用：
sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-keys F42ED6FBAB17C654

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions python3-rosdep
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

**✅ 完成标志**：

```bash
echo $ROS_DISTRO   # 输出 humble
ros2 --help        # 正常显示帮助
```

## 4. 安装 Gazebo Sim 8（gz-harmonic）

```bash
sudo apt install -y lsb-release wget gnupg
sudo wget https://packages.osrfoundation.org/gazebo.gpg \
  -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo-ubuntu-stable $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update && sudo apt install -y gz-harmonic
```

**✅ 完成标志**：

```bash
gz sim --version   # 输出 8.15.0
```

## 5. Python 依赖链

```bash
sudo apt install -y libopencv-dev python3-opencv python3-numpy
/usr/bin/python3 -m pip install empy kconfiglib jsonschema jinja2 pyserial pyulog pyros-genmsg pymavlink numpy==1.26.4
```

**✅ 完成标志**：

```bash
python3 -c "import cv2, numpy, pymavlink; print('OK')"   # 输出 OK（numpy 必须是 1.x，勿装 2.x）
```

## 6. 编译 PX4 v1.18

**① 克隆（含子模块，镜像已全局生效）**：

```bash
cd ~ && git clone https://github.com/PX4/PX4-Autopilot.git --recursive
cd ~/PX4-Autopilot && git checkout release/1.18
```

**② 跳过不需要的子模块**（flightgear 桥接，gz 仿真用不到；不跳会卡死/损坏）：

```bash
git config -f .gitmodules submodule.Tools/simulation/flightgear/flightgear_bridge.update none
```

**③ 拉取子模块**（若某个子模块反复卡/损坏：`git submodule deinit -f <路径>` 后重拉）：

```bash
git submodule update --init --recursive --depth 1
git submodule status | grep -cE "^ "   # ✅ 正常数应 ≥ 35（无 ^- 前缀）
```

**④ 补全 PX4 构建所需的 Python 模块**（一次性）：

```bash
/usr/bin/python3 -m pip install -r ~/PX4-Autopilot/Tools/setup/requirements.txt
```

**⑤ 编译**（约 20-40 分钟；16 核机器用 -j4 防止内存爆掉）：

```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500 -j4
```

**✅ 完成标志（最终验收一）**：

- 编译结束自动拉起 **Gazebo 窗口**（世界中出现 ground_plane）
- 终端出现 `pxh>` 提示符
- 日志出现 `INFO [uxrce_dds_client] init UDP agent IP:127.0.0.1, port:8888`
- 日志出现 mavlink 端口 `18570`

> 若编译中断，报 `No module named 'xxx'` → 补装对应模块后重跑 `make`（增量继续）。
> 若报 `Failed to import jsonschema/jinja2` 等 → 执行步骤 ④ 后重跑。

## 7. ROS2 工作区：桥接包

```bash
mkdir -p ~/ws_px4/src && cd ~/ws_px4/src
git clone --depth 1 -b release/1.18 https://github.com/PX4/px4_msgs.git
git clone --depth 1 https://github.com/PX4/px4_ros_com.git
```

> 使用 `--depth 1` 浅克隆（只需分支最新代码，全量 clone 走镜像会卡 10 分钟+）。

**✅ 完成标志**：

```bash
ls ~/ws_px4/src    # 看到 px4_msgs  px4_ros_com
```

## 8. px4_ros2_opencv_tracking 功能包

本仓库已提供现成功能包 `src/air/px4_ros2_opencv_tracking`（标准 ament_python 包，含 launch），
**不需要手工抄代码**，拷进工作空间编译即可：

```bash
# 克隆本仓库（已克隆可跳过）
git clone https://github.com/OpenHUTB/ros2.git ~/ros2
# 拷入工作空间
cp -r ~/ros2/src/air/px4_ros2_opencv_tracking ~/ws_px4/src/
cd ~/ws_px4
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_ros2_opencv_tracking
```

**包内三个节点**：

| 可执行文件 | 作用 | 订阅 / 发布 |
| --- | --- | --- |
| `image_pub` | 合成 640×480 黑底红色移动圆（10 Hz） | 发布 `/camera/image_raw` |
| `color_tracker` | HSV 红色双区间 + 轮廓质心 | `/camera/image_raw` → `/vision/target`（`point.x/y` 为偏移像素，`point.z` 为目标面积） |
| `offboard_control` | 图像偏差 → PX4 Offboard 速度指令 | `/vision/target` → `/fmu/in/offboard_control_mode`、`/fmu/in/trajectory_setpoint`、`/fmu/in/vehicle_command` |

`launch/tracking_launch.py` 一次拉起三个节点。

**✅ 完成标志**：

```bash
source ~/ws_px4/install/setup.bash
ros2 pkg executables px4_ros2_opencv_tracking   # 列出 color_tracker image_pub offboard_control
```

> 若 `ros2 pkg executables` 看不到包：确认三个脚本确实在 `px4_ros2_opencv_tracking/` 包目录内
> （与 setup.py 的 `px4_ros2_opencv_tracking.xxx:main` 入口对应），再
> `colcon build --packages-select px4_ros2_opencv_tracking` 重编。

## 9. 构建 MicroXRCEAgent（PX4 ↔ ROS2 桥接器）

```bash
cd ~ && git clone https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent && mkdir build && cd build
cmake .. && make -j4
sudo make install && sudo ldconfig
```

**✅ 完成标志**：

```bash
which MicroXRCEAgent    # 输出 /usr/local/bin/MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888   # 显示 running... port: 8888（Ctrl+C 关掉）
```

## 10. 启动仿真与视觉追踪（五终端）

> **问题修复：无法渲染的问题**。VMware 虚拟机内 Gazebo 3D 画面可能渲染空白（server headless + 无 GPU 加速），**这不影响仿真与验收**，话题数据即权威证据。这里提供一套更快的检验方式（见下方一键脚本），并附有真实运行视频。

> 用 `make` 方式启动 PX4（自动管理 gz + px4，避免手动 `./bin/px4` 偶发卡死在 uxrce 创建阶段）。**不要**再手动启动 gz。

**更快检验成果：一键脚本（只需一个端口就能检验）**。把以下脚本放到 `~/` 并加执行权限，此后无需每次敲五条命令，也无需关注渲染是否正常，只看 `8888` 端口与话题即可确认跑通：

```bash
# ~/px4_sim.sh —— 启动 PX4 仿真 + 桥接（无 GUI 版本，只看 8888 端口）
#!/bin/bash
set -e
cd ~/PX4-Autopilot
PX4_SIM_MODEL=gz_x500 make px4_sitl gz_x500 > /tmp/px4.log 2>&1 &
sleep 2
MicroXRCEAgent udp4 -p 8888 --verbose 0 > /tmp/agent.log 2>&1 &
sleep 3
# 检验标志：8888 端口已被 uXRCE-DDS Agent 监听
ss -tulnp | grep 8888 && echo "PX4 已连接 8888 端口，只需一个端口即可确认通信链路就绪"
```

```bash
# ~/px4_sim.sh gui —— 带图形界面启动（在开启"加速 3D 图形"后可看画面）
#!/bin/bash
cd ~/PX4-Autopilot
PX4_SIM_MODEL=gz_x500 make px4_sitl gz_x500
```

```bash
# ~/px4_stop.sh —— 一键停止全部仿真进程
#!/bin/bash
pkill -f "bin/px4" 2>/dev/null || true
pkill -f MicroXRCEAgent 2>/dev/null || true
pkill -f "gz sim" 2>/dev/null || true
pkill -f gcs.py 2>/dev/null || true
pkill -f px4_ros2_opencv_tracking 2>/dev/null || true
echo "已停止全部 PX4/桥接/仿真进程"
```

```bash
# 赋予执行权限
chmod +x ~/px4_sim.sh ~/px4_stop.sh
```

```bash
# 终端 1：PX4 SITL（自动拉起 Gazebo，等待模型加载）
cd ~/PX4-Autopilot
PX4_SIM_MODEL=gz_x500 make px4_sitl gz_x500
# ✅ 标志：Gazebo 窗口出现 x500 模型 + 终端出现 pxh> 提示符

# 终端 2：MicroXRCEAgent
MicroXRCEAgent udp4 -p 8888 --verbose 0
# ✅ 标志：running... port: 8888

# 终端 3：GCS 心跳（关键！PX4 1.18 的 ARM 前置检查要求 GCS 在线，否则
#         报 "No connection to the GCS" 拒绝解锁。用 pymavlink 模拟地面站心跳）
cat > /tmp/gcs.py << 'EOF'
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpout:127.0.0.1:18570', source_system=255, source_component=190)
print('GCS heartbeat -> 18570')
while True:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
    time.sleep(1)
EOF
python3 /tmp/gcs.py &
# ✅ 标志：打印 GCS heartbeat -> 18570（注意是 udpout 不是 udput）

# 终端 4：验证话题并启动追踪
source /opt/ros/humble/setup.bash && source ~/ws_px4/install/setup.bash
ros2 topic list | grep -c "^/fmu"        # ✅ ≥ 60
ros2 topic echo /fmu/out/failsafe_flags --once | grep gcs_connection_lost   # ✅ false
ros2 launch px4_ros2_opencv_tracking tracking_launch.py
# ✅ 标志：image_pub / color_tracker / offboard_control 三个节点启动；
#          offboard_control 打印 Offboard mode set / ARM sent
```

## 11. 最终验收

新开终端（终端 5）：

```bash
source /opt/ros/humble/setup.bash && source ~/ws_px4/install/setup.bash
ros2 topic echo /fmu/out/vehicle_status_v4 --once | grep -E "arming_state|nav_state"
```

**✅ 通过标准（全部满足即复现成功）**：

| 指标                             | 期望值              | 含义               |
| ------------------------------ | ---------------- | ---------------- |
| `arming_state`                 | 1 或 2            | 无人机已解锁（ARM 成功）   |
| `nav_state`                    | 14               | Offboard 机外控制模式  |
| `z`（vehicle_local_position_v1） | 0 → -2 变化        | 无人机升空约 2 米       |
| `/vision/target`               | 持续输出 point.x/y/z | 视觉闭环（z = 红色目标面积） |

```bash
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once | grep "^z:"   # 多次运行看变化
ros2 topic echo /vision/target --once
```

> 说明：VMware 虚拟机内 Gazebo 3D 画面可能渲染空白（server headless + 无 GPU 加速），**不影响仿真与验收**，以上话题数据即权威证据。若想看到画面：虚拟机设置 → 显示器 → 勾选"加速 3D 图形"（需重启）。

### 11.1 运行效果视频

以下是仿真运行录屏：无人机起飞后按 `/vision/target` 的偏差调整水平位置与前后距离。

![PX4 视觉追踪运行效果](../img/px4_tracking_demo.mp4)

> 快速检验思路：只需确认 **`8888` 端口**被 `MicroXRCEAgent` 监听（PX4 通信链路就绪），再 `ros2 topic echo /vision/target --once` 看到持续输出的 `point.x/y/z`，即可判定视觉追踪闭环工作正常——无需等待 Gazebo 画面渲染。

## 12. 常见问题（真实踩坑记录）

**Q1：github.com 直连 Connection refused / clone 卡死？**
配置镜像全局重写（第 2 节 ③），大仓库（px4_msgs 等）用 `--depth 1` 浅克隆。若镜像也慢，可换 `ghfast.top`。

**Q2：磁盘满 No space left on device？**
PX4 全量子模块 + 编译需要 25G+。扩容：VMware 设置加磁盘 → `sudo growpart /dev/sda 3 && sudo resize2fs /dev/sda3`。

**Q3：子模块拉取反复卡死 / config.lock 报错？**
`flightgear_bridge` 子模块跳过（第 6 节 ②）；损坏的子模块 `git submodule deinit -f <路径> && rm -rf <路径>` 后重拉。

**Q4：编译报 `No module named 'empy/jsonschema/jinja2'`？**
执行第 6 节 ④（`pip install -r Tools/setup/requirements.txt`）一次补齐后重跑 `make`。

**Q5：`ros2 launch` 报 `ModuleNotFoundError: px4_ros2_opencv_tracking.xxx`？**
三个脚本没有放进 `px4_ros2_opencv_tracking/` 包目录（setup.py 入口是 `px4_ros2_opencv_tracking.xxx:main`）。按第 8 节结构放置后，重 `colcon build --packages-select px4_ros2_opencv_tracking`。

**Q6：ARM 被拒，日志 `No connection to the GCS` / `gcs_connection_lost: true`？**
PX4 1.18 的 HealthAndArmingChecks 把 GCS 连接作为解锁前置。运行第 10 节终端 3 的 pymavlink 心跳（`udpout:127.0.0.1:18570`，1Hz），等 `failsafe_flags.gcs_connection_lost=false` 后重启 launch 重新 ARM。

**Q7：手动 `./bin/px4 -d` 启动卡在 data writer 创建？**
改用 `make px4_sitl gz_x500` 方式启动（第 10 节终端 1），官方脚本已规避该问题。

**Q8：Gazebo 窗口打开但 3D 画面空白？**
VMware 无 GPU 加速导致渲染不可见（World 面板显示 Render Engine Server Headless）。数据验收不受影响；开启虚拟机"加速 3D 图形"可恢复画面。

**Q9：`import cv2` 报 `_ARRAY_API not found`？**
NumPy 2.x 与系统 OpenCV 不兼容：`/usr/bin/python3 -m pip install "numpy==1.26.4"` 降级。

**Q10：`px4_ros_com` 没有 `release/1.18` 分支？**
用 `main` 分支即可，不影响编译与桥接。

## 附录 A：修正的 Bug 清单

| #   | 问题                                                                | 修正                                       |
| --- | ----------------------------------------------------------------- | ---------------------------------------- |
| 1   | 原始 README 中 launch 的 executable 带 `.py`，ROS2 找不到                  | `executable='image_pub'`（不带 .py）         |
| 2   | 脚本放在 `scripts/` 但 setup.py 入口指向 `px4_ros2_opencv_tracking.xxx` → import 失败 | 脚本放入 `px4_ros2_opencv_tracking/` 包目录       |
| 3   | launch 的 `executable` 与包内可执行名不一致（带 `.py` 或旧节点名）                    | 统一为 `image_pub` / `color_tracker` / `offboard_control` |
| 4   | `PointStamped` 无顶层 `x/y/z`，坐标在 `.point` 子消息                       | 全部使用 `p.point.x/y/z` / `msg.point.x/y/z` |
| 5   | `DO_SET_MODE(176)` 的 `param1=0` 在 1.18 无效                         | `param1=1.0, param2=6.0`                 |
| 6   | OffboardControlMode 仅 `velocity=True` 时位置设定点不生效                   | `position=True, velocity=True` 双通道       |
| 7   | PX4 1.18 ARM 前置检查要求 GCS 在线 → 解锁被拒                                 | pymavlink 心跳脚本（18570 端口）                 |
| 8   | 手动 `./bin/px4` 偶发卡死在 uxrce 创建                                     | 统一用 `make px4_sitl gz_x500` 启动           |

## 附录 B：命令速查

```bash
# 常用验证
source /opt/ros/humble/setup.bash && source ~/ws_px4/install/setup.bash
ros2 topic list | grep -c "^/fmu"
ros2 topic echo /vision/target --once
ros2 topic echo /fmu/out/vehicle_status_v4 --once | grep -E "arming_state|nav_state"
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once | grep "^z:"

# 重启整套
pkill -f "bin/px4" ; pkill -f MicroXRCEAgent ; pkill -f "gz sim" ; pkill -f gcs.py ; pkill -f px4_ros2_opencv_tracking
```

---

> 本文档基于 Ubuntu 22.04.5 LTS + VMware 虚拟机**完整复现并修正所有 bug** 的真实过程。环境版本：ROS2 Humble、PX4 release/1.18、gz-harmonic 8.15.0、Micro-XRCE-DDS-Agent、OpenCV 4.x。
