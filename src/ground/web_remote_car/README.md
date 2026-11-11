# ROS Noetic 网页远程遥控小车

在 Gazebo 仿真环境中搭建搭载 RGB 相机的差速小车模型，通过 `rosbridge_server` 建立网页与 ROS 的 WebSocket 通信，`web_video_server` 推送相机图像流；网页端支持 W/A/S/D 键遥控小车，并实时查看车载相机画面。

## 运行环境

- 操作系统：Ubuntu 20.04.6 LTS
- ROS：Noetic（1.17.4）
- Python：3.8.10
- 仿真器：Gazebo 11
- 依赖包：`ros-noetic-rosbridge-server`、`ros-noetic-web-video-server`、`xacro`、`joint_state_publisher`、`robot_state_publisher`、`gazebo_ros`

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-web-video-server
```

## 构建

```bash
mkdir -p ~/ros1_ws/src
cp -r <仓库路径>/src/ground/web_remote_car ~/ros1_ws/src/
cd ~/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 启动

一条命令拉起全部服务（仿真、小车模型、rosbridge、视频流、前端页面）：

```bash
roslaunch web_remote_car start_all.launch
```

前端静态页面由 `scripts/web_server.py` 节点托管（已写进 `launch/web_bridge.launch`），监听 `0.0.0.0:8000`，启动日志会打印：

```text
[INFO] 本机访问：http://127.0.0.1:8000/
[INFO] 局域网访问：http://192.168.1.102:8000/
```

浏览器直接打开日志里的地址即可，不需要再另开终端跑 `python3 -m http.server`。8000 被占用时用 `roslaunch web_remote_car start_all.launch frontend_port:=8001` 换端口，`bridge_port`（默认 9090）、`video_port`（默认 8080）同样可以命令行覆盖；改了视频端口要在地址里带 `?video=<IP>:<端口>`。页面里的 rosbridge 地址与视频流地址由当前访问的主机名自动拼接，换机器访问不需要改代码。

只想单独调试前端、不起仿真时，仍可手动起静态服务（必须在 `www` 目录）：

```bash
cd ~/ros1_ws/src/web_remote_car/www
python3 -m http.server 8000
```

## 操作

控制按键：W 前进、S 后退、A 左转、D 右转、空格停止，方向键等效。

### 局域网其他电脑访问

1. 虚拟机网络用桥接模式，NAT 模式下局域网其他机器访问不到虚拟机。
2. `launch/web_bridge.launch` 已给 rosbridge 配 `address=0.0.0.0`，启动后核对 `netstat -tulpn | grep 9090`，必须是 `0.0.0.0:9090`，只有 `127.0.0.1:9090` 说明没监听外网。
3. 放行端口：

```bash
sudo ufw allow 9090/tcp
sudo ufw allow 8080/tcp
sudo ufw allow 8000/tcp
```

其他电脑打开 `http://<虚拟机IP>:8000/`，自动连 `ws://<虚拟机IP>:9090`。网页与 rosbridge 不在同一台机器时，用 `?bridge=<IP>:9090` 指定。

前端页面不引用任何公网 CDN 资源：`www/index.html` 用浏览器原生 `WebSocket` 直接收发 rosbridge 的 JSON 协议帧（`{"op":"advertise"}` / `{"op":"publish"}`），因此只有内网、连不上外网的机器也能正常连接；旧版本从 `cdn.jsdelivr.net` 加载 `roslib.js`，取不到时页面会一直停在“正在连接”。浏览器如果还显示旧页面，按 `Ctrl+F5` 强制刷新（`scripts/web_server.py` 已加 `Cache-Control: no-store`）。

## 端口与话题

| 项目 | 值 | 位置 |
| --- | --- | --- |
| rosbridge WebSocket | 9090 | `launch/web_bridge.launch` |
| web_video_server 图像流 | 8080 | `launch/web_bridge.launch` |
| 前端静态页面 | 8000 | `launch/web_bridge.launch` 的 `frontend_port`，由 `scripts/web_server.py` 提供 |
| 相机图像话题 | `/front_camera/image_raw` | `urdf/car_cam.xacro`、`www/index.html` |
| 速度控制话题 | `/cmd_vel` | `www/index.html` |
| 线速度 / 角速度 | 0.3 m/s / 0.4 rad/s | `www/index.html` 中的 `SPEED` / `TURN_SPEED` |

端口、话题在对应文件里是直接写死的常量，修改后需要同步改前端 `www/index.html` 里的 `BRIDGE_PORT`、`VIDEO_PORT`、`CAMERA_TOPIC`，重启 launch 后生效。

## 验证

```bash
rostopic hz /front_camera/image_raw     # 相机图像，实测约 30 Hz
rostopic echo /cmd_vel                  # 按键时才有消息，按需发布
```

## 已知限制

1. 仅支持单用户控制，多端同时下发速度指令会互相覆盖。
2. 只有远程手动遥控，没有自主避障、路径规划、自动巡航。
3. 相机只做原始图像推流，不含图像识别、目标检测处理。
4. 模型为纯仿真差速车，未在实体硬件上部署验证。
5. 前端没有断线自动重连，WebSocket 断开后需要手动刷新页面。

## 大模型辅助开发声明

本模块的 Xacro 模型调试、Launch 文件编写、前端遥控页面代码、文档撰写与 Git 提交流程排错使用大模型辅助完成；控制方案、仿真模型结构与整体功能设计由作者完成，作者对全部内容负责。
