# px4_ros2_opencv_tracking

PX4 SITL（`gz_x500`）+ ROS 2 Humble + OpenCV 的**视觉检测与 Offboard 速度控制演示**。

> ⚠️ **重要说明（请先读）**
> - 本模块的 `/camera/image_raw` 来自 `image_pub` 节点**程序合成的图像**（黑底红色圆），
>   **不是** Gazebo 相机画面。`gz_x500` 机型默认没有相机，因此图像与无人机在仿真中的
>   运动无关，本模块演示的是「OpenCV 检测 → Offboard 速度指令 → PX4 位置响应」这条
>   控制/通信链路，**不是严格意义的视觉闭环伺服**。
> - 如需真正的视觉闭环，请使用带相机的机型（如 `gz_x500_mono_cam` / `gz_x500_depth`），
>   用 `ros_gz_bridge` 把 Gazebo 相机话题桥接到 `/camera/image_raw`，并在 Gazebo 世界中
>   放置红色目标物。

## 目录结构

```
px4_ros2_opencv_tracking/
├── package.xml
├── setup.py / setup.cfg
├── launch/tracking_launch.py
└── px4_ros2_opencv_tracking/
    ├── image_pub.py         # 合成图像源（640x480, 10 Hz）
    ├── color_tracker.py     # HSV 红色检测 -> /vision/target + /vision/debug_image
    └── offboard_control.py  # 状态机：WAIT_TARGET -> SET_OFFBOARD -> ARM -> TAKEOFF -> TRACK
```

## 依赖

- Ubuntu 22.04 + ROS 2 Humble
- PX4-Autopilot `release/1.18`（含 Gazebo Sim 8 / gz_x500）
- `px4_msgs`（release/1.18）已在本工作空间构建。**注意：`px4_msgs` 不是 rosdep 二进制包**
  （PX4 未发布 release，`rosdep install` 无法解析），必须按下方「构建」步骤从 PX4 源码 clone 后
  与工作空间一起 `colcon build`；因此 `package.xml` 中不声明该 `exec_depend`。
- `Micro-XRCE-DDS-Agent`（`udp4 -p 8888`）
- Python 3.10、OpenCV、numpy、cv_bridge、pymavlink（GCS 心跳用）

## 构建

```bash
# 1) 先构建桥接消息包（新工作区必须）
mkdir -p ~/ws_px4/src && cd ~/ws_px4/src
git clone --depth 1 -b release/1.18 https://github.com/PX4/px4_msgs.git
git clone --depth 1 https://github.com/PX4/px4_ros_com.git
# 2) 拷入本功能包
cp -r <repo>/src/air/px4_ros2_opencv_tracking ~/ws_px4/src/
cd ~/ws_px4
source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs px4_ros_com px4_ros2_opencv_tracking
source install/setup.bash
ros2 pkg executables px4_ros2_opencv_tracking
# 期望输出：
#   px4_ros2_opencv_tracking color_tracker
#   px4_ros2_opencv_tracking image_pub
#   px4_ros2_opencv_tracking offboard_control
```

## 运行（4 个终端）

```bash
# 终端 1：PX4 SITL + Gazebo
cd ~/PX4-Autopilot && PX4_SIM_MODEL=gz_x500 make px4_sitl gz_x500

# 终端 2：uXRCE-DDS 桥接
MicroXRCEAgent udp4 -p 8888

# 终端 3：GCS 心跳（PX4 1.18 解锁前置检查要求 GCS 在线）
cat > /tmp/gcs.py <<'PY'
import time
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpout:127.0.0.1:18570', source_system=255, source_component=190)
while True:
    m.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, mavutil.mavlink.MAV_STATE_ACTIVE)
    time.sleep(1)
PY
python3 /tmp/gcs.py

# 终端 4：启动本模块
source /opt/ros/humble/setup.bash && source ~/ws_px4/install/setup.bash
ros2 launch px4_ros2_opencv_tracking tracking_launch.py
```

## 验收命令

```bash
source /opt/ros/humble/setup.bash && source ~/ws_px4/install/setup.bash

ros2 topic list | grep -c '^/fmu'                                  # >= 60
ros2 topic echo /fmu/out/vehicle_status_v4 --once | grep -E 'arming_state|^nav_state:'
#   arming_state: 2        # 2 = ARMED（注意 1 = DISARMED）
#   nav_state: 14          # 14 = OFFBOARD
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once | grep -E '^\s*(x|y|z):'
#   z ≈ -2.0 ± 0.3         # 稳定悬停在离地 2 m
ros2 topic echo /vision/target --once
ros2 topic hz /vision/target                                       # ~10 Hz
ros2 run rqt_image_view rqt_image_view                             # 选 /vision/debug_image 查看检测效果
```

## 参数（launch 中可改）

| 参数 | 默认 | 说明 |
| --- | --- | --- |
| `image_pub.radius_px` | 35 | 合成红圆半径（`contourArea` 实测约 3750 px²） |
| `image_pub.rate_hz` | 10.0 | 图像发布频率 |
| `color_tracker.min_area` | 300.0 | 有效目标最小面积 px² |
| `offboard_control.takeoff_alt` | -2.0 | 悬停高度（NED，向下为正） |
| `offboard_control.desired_area` | 3750.0 | 期望目标面积（距离保持目标）；换相机/分辨率后需重新标定 |
| `offboard_control.max_v` | 1.0 | 水平速度限幅 m/s |
| `offboard_control.k_y` / `k_d` / `k_alt` | 0.12 / 0.02 / 0.8 | 居中 / 距离 / 高度比例增益 |
| `offboard_control.target_timeout` | 3.0 | 目标丢失悬停超时 s |

> `desired_area` 标定方法：目标静止在期望距离时读一次 `/vision/target` 的 `point.z`，
> 把该值填入 `desired_area`；若使用真实相机，还需按相机内参和目真实尺寸换算。

## 已知限制

1. 图像为合成输入（开环），不能证明"无人机看到并追踪了仿真世界中的目标"；
2. 高度/位置依赖 PX4 EKF 估计，实机测试前务必做 SITL 参数与 failsafe 检查；
3. 安全收尾：节点退出时会发送 LAND，但仍建议在 QGC/MAVLink 端监控；
4. 本模块按 **PX4 `release/1.18`** 的 `vehicle_status_v4` / `vehicle_local_position_v1` 等话题名实现
   （话题后缀来自 px4_msgs 的 `MESSAGE_VERSION`）。**其他 PX4 版本需先核对实际话题名**，
   必要时修改 `offboard_control.py` 中的订阅话题。
