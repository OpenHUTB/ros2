# ROS2 Service 控制小海龟移动

## 功能介绍

本示例使用 ROS2 Service 实现对 turtlesim 小海龟的距离控制。

客户端向 `/move_forward` 服务发送需要前进的距离，服务端收到请求后，通过发布 `/turtle1/cmd_vel` 控制小海龟运动，同时订阅 `/turtle1/pose` 获取小海龟当前位置。

当小海龟实际移动距离达到目标距离后，服务端停止运动，并向客户端返回执行结果。

## 系统结构

```text
Service Client
    |
    | /move_forward
    | distance
    v
Service Server
    |
    | /turtle1/cmd_vel
    v
turtlesim
    |
    | /turtle1/pose
    v
Service Server
