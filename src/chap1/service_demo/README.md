# ROS2 Service 小海龟距离控制

## 项目介绍

本示例使用 ROS 2 Service 实现对 turtlesim 小海龟的距离控制。

客户端向 `/move_forward` 服务发送需要前进的距离，服务端收到请求后，通过 `/turtle1/cmd_vel` 控制小海龟运动，同时订阅 `/turtle1/pose` 获取小海龟当前位置。

当小海龟实际移动距离达到目标距离后，服务端停止运动，并向客户端返回执行结果。

## 运行环境

- Ubuntu 22.04
- ROS 2 Humble
- Python 3
- turtlesim

## 项目结构

```text
service_demo/
├── launch/
│   └── move_forward.launch.py
├── service_demo/
│   ├── client.py
│   ├── main.py
│   └── server.py
├── README.md
├── package.xml
└── setup.py

service_interfaces/
├── srv/
│   └── MoveForward.srv
├── CMakeLists.txt
└── package.xml
