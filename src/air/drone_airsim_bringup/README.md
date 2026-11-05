# drone_airsim_bringup

AirSim 无人机接入包（ROS Noetic）。属于 **CarlaAir-Drone / 空地一体无人机自主飞行仿真实验** 的阶段 1，对应 PR1。

## 这个包解决什么问题

CarlaAir 把 CARLA 和 AirSim 放在同一个 UE4 进程里，但 AirSim 只提供 RPC 接口，没有现成的 ROS 话题。这个包把无人机的位姿、相机和速度控制接到 ROS 上，后面的感知、SLAM 和端到端模块都从这里取数据。

## 功能

- 连接 CarlaAir 的 AirSim（默认 `192.168.189.1:41451`）；
- 发布 `/drone/pose`（NED 世界坐标，`geometry_msgs/PoseStamped`）；
- 发布 `/drone/camera/image_raw`（`sensor_msgs/Image`，bgr8）；
- 订阅 `/drone/cmd_vel`（NED 速度，`geometry_msgs/Twist`）；
- 提供 `/drone/takeoff`、`/drone/land` 服务（`std_srvs/Empty`）。

## 目录结构

```text
src/air/drone_airsim_bringup/
├── CMakeLists.txt
├── package.xml
├── README.md
├── ROADMAP.md
├── config/
│   └── settings.json
├── launch/
│   └── drone_bringup.launch
└── scripts/
    └── main.py
```

## 环境

- Windows：CarlaAir v0.1.7，AirSim 1.8.1，端口 41451；
- Ubuntu：20.04.6 LTS，ROS Noetic，Python 3.8；
- 虚拟机：`D:\ROS2004\Ubuntu2004.vmx`，用户 `ncut`。

## 相机配置（必须做）

AirSim 的分辨率由服务端的 `settings.json` 决定，不配置就默认 256×144。仓库里的 `config/settings.json` 是实测用的配置（相机 1280×960）。

注意：`CarlaAir.ps1` 每次启动都会用 `<CarlaAir>\AirSimConfig\settings.json` 覆盖 `%USERPROFILE%\Documents\AirSim\settings.json`，所以只改 Documents 会被覆盖。推荐先备份，再替换 CarlaAir 的模板：

```powershell
copy "<CarlaAir>\AirSimConfig\settings.json" "<CarlaAir>\AirSimConfig\settings.json.bak"
copy config\settings.json "<CarlaAir>\AirSimConfig\settings.json"
```

然后启动 CarlaAir。如果直接运行 `CarlaUE4.exe`，才需要复制到 `%USERPROFILE%\Documents\AirSim\settings.json`。VM 里不需要放这个文件。

## 依赖安装

ROS 侧：

```bash
sudo apt install ros-noetic-sensor-msgs ros-noetic-geometry-msgs
```

AirSim Python 客户端是这一步最容易卡住的地方。PyPI 上 `airsim==1.8.1` 的 sdist 在 Python 3.8 下会报 `metadata-generation-failed`，直接 `pip install` 装不上。可用的做法是从一个已经装好 AirSim 1.8.1 的环境复制 `airsim` 包，并补齐它依赖的旧版库：

```bash
python3 -m pip install --user msgpack-python==0.5.6 tornado==4.5.3
# 把 airsim/ 和 msgpackrpc/ 复制到 ~/.local/lib/python3.8/site-packages/
python3 -c "import airsim; print(airsim.__file__)"
```

注意 `msgpack` 必须是 0.5.x，1.x 会报 `Packer.__init__() got an unexpected keyword argument 'encoding'`；`tornado` 必须是 4.5.x，6.x 没有 `tornado.platform.auto`。

## 编译

```bash
source /opt/ros/noetic/setup.bash
cd /opt/drone_ws
catkin_make
source devel/setup.bash
```

## 运行

先确认 Windows 上 CarlaAir 已启动，2000 和 41451 都在监听：

```powershell
Get-NetTCPConnection -State Listen -LocalPort 2000,41451
```

VM 里：

```bash
source /opt/ros/noetic/setup.bash
source /opt/drone_ws/devel/setup.bash
roslaunch drone_airsim_bringup drone_bringup.launch host:=192.168.189.1 port:=41451
```

## 验证

```bash
rostopic list | grep drone
rostopic hz /drone/camera/image_raw
rostopic echo -n1 /drone/pose
rosservice call /drone/takeoff
rosservice call /drone/land
```

预期输出：

- 话题：`/drone/pose`、`/drone/camera/image_raw`、`/drone/cmd_vel`；
- 图像：配好 `settings.json` 后 1280×960；不配则 256×144；
- 位姿示例：

```yaml
header:
  frame_id: "world_ned"
pose:
  position:
    x: -0.0519
    y: 0.0530
    z: 29.2545
```

- `takeoff` / `land` 返回空响应，无报错；
- 起降期间 `/drone/pose` 和相机继续更新（不再长时间持锁）。

## 本次 review 后的修改

- 去掉 `cv_bridge`，直接构造 `sensor_msgs/Image`，避免 OpenCV 5 下 `KeyError: 16`；
- 相机异常改为捕获 `Exception` 并打印 `%r`，日志里能看到异常类型；
- 增加 `config/settings.json`，固定 1280×960；
- 包路径改为 `src/air/drone_airsim_bringup`；
- `README.md` / `ROADMAP.md` 用 UTF-8 重新写入，修复乱码；
- 起降不再在 `join()` 期间持锁，改为异步命令 + 分段等待。

## 已知问题

1. **端口冲突**：CarlaAir 和标准 CARLA 都占用 2000 端口，不能同时启动。做无人机时只启动 CarlaAir。
2. **图像频率**：1280×960 约 2.7 Hz，256×144 可到 5 Hz；主要受 AirSim RPC 和 VM 限制。
3. **坐标系**：当前位姿按 AirSim NED 发布（`frame_id: world_ned`），ROS 侧默认 ENU，后续在感知/控制节点里做转换。
4. **线程安全**：所有 AirSim 调用必须走同一把锁，不要在多个回调里直接并发调用。
5. **OpenCV 版本**：本包不再依赖 `cv_bridge`/OpenCV，换 OpenCV 5 不会影响相机话题。

## 整体任务进度

见 [ROADMAP.md](ROADMAP.md)。当前处于阶段 1 的 PR1，后续 PR2（深度/IMU）、PR3（键盘遥控）会在这个包上继续加。
