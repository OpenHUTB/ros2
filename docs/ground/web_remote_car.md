# 网页远程遥控小车

在 Gazebo 仿真环境中搭建搭载 RGB 相机的差速小车模型，通过 `rosbridge_server` 实现 ROS 与前端 JS 的 WebSocket 通信，`web_video_server` 推送相机图像流；网页端支持键盘控制小车运动，带按键按下高亮反馈，同步展示小车相机视角画面。

源码位置：`src/ground/web_remote_car`

## 1. 项目目标

- 在 Gazebo 中搭建搭载 RGB 相机的差速小车 Xacro 模型
- 使用 `rosbridge_server` 建立 ROS 与网页前端的 WebSocket 通信
- 使用 `web_video_server` 发布相机图像流
- 网页前端实现 W/A/S/D 键盘遥控小车，按键高亮反馈
- 网页实时渲染相机画面，实现远程可视化监控

## 2. 数据流

```text
键盘按键事件 -> JS -> rosbridge_server -> /cmd_vel -> 小车运动

相机图像话题 /front_camera/image_raw -> web_video_server -> HTTP 图像流 -> 网页展示
```

## 3. 运行环境

| 项目 | 说明 |
| --- | --- |
| 操作系统 | Ubuntu 20.04.6 LTS |
| ROS | Noetic（1.17.4） |
| Python | 3.8.10 |
| 仿真器 | Gazebo 11 |
| 依赖 | `ros-noetic-rosbridge-server`、`ros-noetic-web-video-server`、`xacro`、`joint_state_publisher`、`robot_state_publisher`、`gazebo_ros` |

依赖安装：

```bash
sudo apt install ros-noetic-rosbridge-server ros-noetic-web-video-server
```

本项目依赖 Gazebo 仿真环境，不需要真实硬件小车与实体相机。

## 4. 构建

```bash
mkdir -p ~/ros1_ws/src
cp -r <仓库路径>/src/ground/web_remote_car ~/ros1_ws/src/
cd ~/ros1_ws
source /opt/ros/noetic/setup.bash
catkin_make
source devel/setup.bash
```

## 5. 启动

终端 1，启动 Gazebo 仿真、小车模型生成、`rosbridge_server`、`web_video_server`：

```bash
roslaunch web_remote_car start_all.launch
```

终端 2，前端页面是纯静态 HTML，launch 里不会自动起网页服务，需要单独用 Python 自带的 http 服务托管：

```bash
cd ~/ros1_ws/src/web_remote_car/www
python3 -m http.server 8000
```

## 6. 查看结果

新开终端查看话题：

```bash
source /opt/ros/noetic/setup.bash
source ~/ros1_ws/devel/setup.bash
rostopic echo /cmd_vel
rostopic hz /front_camera/image_raw
```

浏览器打开 `http://127.0.0.1:8000`，默认控制按键：W 前进、S 后退、A 左转、D 右转、空格停止，方向键等效。

页面里的 rosbridge 地址和视频流地址由当前访问的主机名自动拼接，浏览器地址填哪台机器的 IP，前端就连哪台机器，不需要改代码。

默认端口与话题：

| 话题 / 端口 | 说明 | 定义位置 |
| --- | --- | --- |
| `/cmd_vel` | 小车速度控制指令 | `www/index.html` |
| `/front_camera/image_raw` | 相机原始图像 | `urdf/car_cam.xacro` |
| 9090 | rosbridge WebSocket 端口 | `launch/web_bridge.launch` |
| 8080 | web_video_server 服务端口 | `launch/web_bridge.launch` |
| 8000 | 前端静态页面端口 | `python3 -m http.server 8000`，仅该命令运行时存在 |

控制与相机画面演示：

![网页遥控小车相机视角 1](../img/camera_demo1.gif)

![网页遥控小车相机视角 2](../img/camera_demo2.gif)

## 7. 参数调整

参数直接写在各文件里，没有单独的 yaml 配置文件：

| 参数 | 默认值 | 位置 |
| --- | --- | --- |
| rosbridge 端口 | 9090 | `launch/web_bridge.launch` 中 `<param name="port" value="9090"/>` |
| 视频流端口 | 8080 | `launch/web_bridge.launch` 中 `web_video_server` 的 `port` |
| 相机话题 | `/front_camera/image_raw` | `urdf/car_cam.xacro` 的 `imageTopicName` |
| 线速度 | 0.3 m/s | `www/index.html` 中 `SPEED` |
| 角速度 | 0.4 rad/s | `www/index.html` 中 `TURN_SPEED` |

修改端口或相机话题后，需要同步修改前端 `www/index.html` 里的 `BRIDGE_PORT`、`VIDEO_PORT`、`CAMERA_TOPIC` 三个常量，重启 launch 生效。

## 8. 实测结果

测试环境：Ubuntu 20.04.6 LTS、ROS Noetic 1.17.4、Python 3.8.10、Gazebo 11。

- `catkin_make` 构建成功
- Gazebo 正常加载小车模型，`/front_camera/image_raw` 正常发布
- 网页端按键指令正常下发到 `/cmd_vel`，小车按 W/A/S/D 平顺运动
- `web_video_server` 图像流在网页端正常渲染

话题实测频率：

| 话题 | 实测频率 |
| --- | --- |
| `/front_camera/image_raw` | 约 30 Hz |
| `/cmd_vel` | 按键触发，按需发布 |

## 9. 常见问题

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| 启动报找不到 `rosbridge_server` / `web_video_server` | 未安装对应包 | `sudo apt install ros-noetic-rosbridge-server ros-noetic-web-video-server` |
| 浏览器打不开 `http://127.0.0.1:8000` | 没起静态文件服务 | 在 `www` 目录执行 `python3 -m http.server 8000` |
| 网页提示连接断开 | rosbridge 未运行或端口被占用 | `rostopic list` 确认 ROS 正常，检查 9090 端口 |
| 网页看不到相机画面 | 视频流话题与 xacro 中的话题不一致 | 核对 `urdf/car_cam.xacro` 的 `imageTopicName` 与 `index.html` 的 `CAMERA_TOPIC` |
| 按键控制小车不动 | `/cmd_vel` 没有消息或差速插件未加载 | `rostopic echo /cmd_vel` 排查，检查 xacro 中 `libgazebo_ros_diff_drive.so` 插件 |

## 10. 已知限制

1. 仅支持单用户控制，多端同时下发速度指令会互相覆盖。
2. 只有远程手动遥控，没有自主避障、路径规划、自动巡航。
3. 相机只做原始图像推流，不含图像识别、目标检测处理。
4. 模型为纯仿真差速车，未在实体硬件上部署验证。
5. 前端没有断线自动重连，WebSocket 断开后需要手动刷新页面。

## 11. 小结

本项目基于 ROS Noetic 搭建网页远程监控遥控系统，`rosbridge` 打通 ROS 后端与前端，`web_video_server` 完成图像推流。适合仿真小车远程操控、车载相机可视化场景，可以在此基础上扩展雷达数据可视化、自动避障、AEB 制动等功能。
