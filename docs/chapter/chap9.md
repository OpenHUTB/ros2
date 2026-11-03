# ROS2 

以下测试都是基于 Ubuntu 20.04 ROS Humble

```shell
source /opt/ros/humble/setup.bash
ros2 topic pub /chatter std_msgs/String "data: Hello world"
# 重新打开一个终端
ros2 topic echo /chatter
```

![](../img/ros/ros2_pub_echo.png)


## 运行 talker 和 listener 示例

```shell
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

![](../img/ros/ros2_talker_listener.png)


## 练习：UAV waypoint tracking

本练习的代码、运行说明和项目图见[UAV waypoint tracking](../chap9/uav_waypoint_pid.md)。
