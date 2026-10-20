# PX4 + ROS2 + OpenCV 无人机视觉追踪

本文档在 **Ubuntu 22.04.5 LTS**（VMware 虚拟机，16 核 / 7G 内存）PX4 v1.18 + ROS2 Humble + Gazebo Sim 8 + OpenCV，仿真无人机起飞并自动追踪红色目标。每一步都给出命令与"完成标志"，全部命令均已实际验证。

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
├── px4_msgs/              # release/1.18 分支（浅克隆）
├── px4_ros_com/           # 桥接与示例（main 分支）
└── vision_tracking/       # 视觉追踪功能包（修正版）
    ├── package.xml
    ├── setup.py
    ├── resource/vision_tracking
    ├── launch/vision_tracking.launch.py
    └── vision_tracking/            # Python 包目录（脚本放这里，勿放 scripts/）
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

## 8. vision_tracking 功能包（修正版完整代码）

> 修正点：① 三个核心脚本放在 `vision_tracking/` **包目录内**（与 setup.py 的 `vision_tracking.xxx:main` 入口匹配，否则 import 失败）；② launch 的 `executable` **不带 .py**；③ `PointStamped` 坐标在 `.point` 子消息；④ `DO_SET_MODE` 的 `param1=1.0`；⑤ OffboardControlMode 需 `position+velocity` 双通道。

**① 目录与基础文件**：

```bash
mkdir -p ~/ws_px4/src/vision_tracking/{launch,resource,vision_tracking}
```

**② package.xml**：

```bash
cat > ~/ws_px4/src/vision_tracking/package.xml << 'EOF'
<?xml version="1.0"?>
<package format="3">
  <name>vision_tracking</name>
  <version>0.0.1</version>
  <description>PX4 vision-based target tracking with OpenCV and offboard control</description>
  <maintainer email="user@todo.todo">user</maintainer>
  <license>Apache-2.0</license>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>sensor_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>cv_bridge</exec_depend>
  <exec_depend>px4_msgs</exec_depend>
  <exec_depend>python3-opencv</exec_depend>
  <exec_depend>python3-numpy</exec_depend>
  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
EOF
```

**③ setup.py**：

```bash
cat > ~/ws_px4/src/vision_tracking/setup.py << 'EOF'
from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'vision_tracking'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='PX4 vision target tracking',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'image_pub = vision_tracking.image_pub:main',
            'color_tracker = vision_tracking.color_tracker:main',
            'offboard_control = vision_tracking.offboard_control:main',
        ],
    },
)
EOF
```

**④ resource 与包初始化**：

```bash
echo "vision_tracking" > ~/ws_px4/src/vision_tracking/resource/vision_tracking
touch ~/ws_px4/src/vision_tracking/vision_tracking/__init__.py
```

**⑤ launch（executable 不带 .py）**：

```bash
cat > ~/ws_px4/src/vision_tracking/launch/vision_tracking.launch.py << 'EOF'
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(package='vision_tracking', executable='image_pub', name='image_pub', output='screen'),
        Node(package='vision_tracking', executable='color_tracker', name='color_tracker', output='screen'),
        Node(package='vision_tracking', executable='offboard_control', name='offboard_control', output='screen'),
    ])
EOF
```

**⑥ image_pub.py**（合成 640x480 黑底红色移动圆，10Hz）：

```bash
cat > ~/ws_px4/src/vision_tracking/vision_tracking/image_pub.py << 'EOF'
#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class ImagePub(Node):
    def __init__(self):
        super().__init__('image_pub')
        self.pub = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()
        self.t = 0.0
        self.create_timer(0.1, self.publish_image)

    def publish_image(self):
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cx = int(320 + 200 * np.sin(self.t * 0.8))
        cy = int(240 + 150 * np.sin(self.t * 0.5 + 1.0))
        cv2.circle(img, (cx, cy), 35, (0, 0, 255), -1)
        self.t += 0.1
        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImagePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

**⑦ color_tracker.py**（HSV 红双区间 + 轮廓质心，发布到 /vision/target；**坐标在 p.point.x/y/z**）：

```bash
cat > ~/ws_px4/src/vision_tracking/vision_tracking/color_tracker.py << 'EOF'
#!/usr/bin/env python3
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge


class ColorTracker(Node):
    def __init__(self):
        super().__init__('color_tracker')
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_cb, 10)
        self.pub = self.create_publisher(PointStamped, '/vision/target', 10)
        self.bridge = CvBridge()
        self.h, self.w = 480, 640
        self.min_area = 300.0

    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception:
            return
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
        mask2 = cv2.inRange(hsv, (160, 100, 100), (180, 255, 255))
        mask = cv2.bitwise_or(mask1, mask2)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        p = PointStamped()
        p.header.stamp = self.get_clock().now().to_msg()
        p.header.frame_id = 'camera'
        p.point.z = 0.0
        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > self.min_area:
                M = cv2.moments(c)
                if M['m00'] > 0:
                    p.point.x = float(M['m10'] / M['m00'] - self.w / 2.0)
                    p.point.y = float(M['m01'] / M['m00'] - self.h / 2.0)
                    p.point.z = float(area)
        self.pub.publish(p)


def main(args=None):
    rclpy.init(args=args)
    node = ColorTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

**⑧ offboard_control.py**（修正版：OffboardControlMode position+velocity 双通道；DO_SET_MODE param1=1.0、param2=6.0；ARM param1=1.0；分步状态机）：

```bash
cat > ~/ws_px4/src/vision_tracking/vision_tracking/offboard_control.py << 'EOF'
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class OffboardControl(Node):
    K_Y = 0.12
    K_Z = 0.08
    K_D = 0.02
    DESIRED_AREA = 15000.0
    MAX_V = 1.5
    TAKEOFF_ALT = -2.0

    def __init__(self):
        super().__init__('offboard_control')
        self.off_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.cmd_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.create_subscription(PointStamped, '/vision/target', self.target_cb, 10)
        self.dx = self.dy = self.area = 0.0
        self.start = self.get_clock().now()
        self._armed = False
        self.create_timer(0.1, self.control)

    def target_cb(self, msg):
        self.dx = msg.point.x
        self.dy = msg.point.y
        self.area = msg.point.z

    def send_cmd(self, cmd, p1, p2):
        vc = VehicleCommand()
        vc.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        vc.command = cmd
        vc.param1 = p1
        vc.param2 = p2
        vc.target_system = 1
        vc.target_component = 1
        vc.source_system = 1
        vc.source_component = 1
        self.cmd_pub.publish(vc)

    def control(self):
        now = self.get_clock().now()
        t = (now - self.start).nanoseconds * 1e-9

        om = OffboardControlMode()
        om.timestamp = int(now.nanoseconds / 1000)
        om.position = True
        om.velocity = True
        self.off_pub.publish(om)

        ts = TrajectorySetpoint()
        ts.timestamp = int(now.nanoseconds / 1000)
        ts.position = [float('nan'), float('nan'), self.TAKEOFF_ALT]
        vx = max(-self.MAX_V, min(self.MAX_V, self.K_D * (self.DESIRED_AREA - self.area)))
        vy = max(-self.MAX_V, min(self.MAX_V, -self.K_Y * self.dx))
        vz = max(-self.MAX_V, min(self.MAX_V, -self.K_Z * self.dy))
        ts.velocity = [vx, vy, vz]
        ts.yaw = 0.0
        self.traj_pub.publish(ts)

        if 2.0 < t < 2.2:
            self.send_cmd(176, 1.0, 6.0)
            self.get_logger().info('Offboard mode set')
        if t > 2.5 and not self._armed:
            self.send_cmd(400, 1.0, 0.0)
            self._armed = True
            self.get_logger().info('ARM sent')


def main(args=None):
    rclpy.init(args=args)
    node = OffboardControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
EOF
```

**⑨ 编译**：

```bash
cd ~/ws_px4
source /opt/ros/humble/setup.bash
export PATH=/usr/bin:$PATH
colcon build --merge-install
```

> 必须加 `--merge-install`（默认布局下 ament_index 找不到 vision_tracking 包）。

**✅ 完成标志**：

```bash
source ~/ws_px4/install/setup.bash
ros2 pkg executables vision_tracking   # 列出 image_pub / color_tracker / offboard_control
python3 -c "import vision_tracking.color_tracker; print('IMPORT OK')"   # 输出 IMPORT OK
```

若 `IMPORT OK` 失败：确认脚本确实在 `vision_tracking/` 包目录内（不是 scripts/），然后 `colcon build --merge-install --packages-select vision_tracking` 重编。

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

> 用 `make` 方式启动 PX4（自动管理 gz + px4，避免手动 `./bin/px4` 偶发卡死在 uxrce 创建阶段）。**不要**再手动启动 gz。

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
ros2 launch vision_tracking vision_tracking.launch.py
# ✅ 标志：3 个节点启动；offboard_control 打印 Offboard mode set / ARM sent
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

## 12. 常见问题（真实踩坑记录）

**Q1：github.com 直连 Connection refused / clone 卡死？**
配置镜像全局重写（第 2 节 ③），大仓库（px4_msgs 等）用 `--depth 1` 浅克隆。若镜像也慢，可换 `ghfast.top`。

**Q2：磁盘满 No space left on device？**
PX4 全量子模块 + 编译需要 25G+。扩容：VMware 设置加磁盘 → `sudo growpart /dev/sda 3 && sudo resize2fs /dev/sda3`。

**Q3：子模块拉取反复卡死 / config.lock 报错？**
`flightgear_bridge` 子模块跳过（第 6 节 ②）；损坏的子模块 `git submodule deinit -f <路径> && rm -rf <路径>` 后重拉。

**Q4：编译报 `No module named 'empy/jsonschema/jinja2'`？**
执行第 6 节 ④（`pip install -r Tools/setup/requirements.txt`）一次补齐后重跑 `make`。

**Q5：`ros2 launch` 报 `ModuleNotFoundError: vision_tracking.xxx`？**
脚本没有放进 `vision_tracking/` 包目录（setup.py 入口 `vision_tracking.xxx:main` 找不到模块）。把三个 .py 放到包目录（第 8 节结构），重 `colcon build --merge-install --packages-select vision_tracking`。

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
| 2   | 脚本放在 `scripts/` 但 setup.py 入口指向 `vision_tracking.xxx` → import 失败 | 脚本放入 `vision_tracking/` 包目录              |
| 3   | `colcon build` 默认布局导致包不可见                                         | 必须 `--merge-install`                     |
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
pkill -f "bin/px4" ; pkill -f MicroXRCEAgent ; pkill -f "gz sim" ; pkill -f gcs.py ; pkill -f vision_tracking
```

---

> 本文档基于 Ubuntu 22.04.5 LTS + VMware 虚拟机**完整复现并修正所有 bug** 的真实过程。环境版本：ROS2 Humble、PX4 release/1.18、gz-harmonic 8.15.0、Micro-XRCE-DDS-Agent、OpenCV 4.x。
