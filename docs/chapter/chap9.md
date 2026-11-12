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

---

## 相关实验：Gazebo 仿真相机与图像话题验证

- [Gazebo 仿真相机与图像话题验证](chap9/gazebo_camera_sim.md)
  - 环境：Ubuntu 22.04 + ROS 2 Humble + Gazebo Fortress
  - 组成：`ros_gz_sim_demos` 示例场景 + `ros_gz_bridge` 参数桥接 + RViz2 图像面板
  - 任务：跑通「仿真相机 → 桥接 → ROS 2 话题 → 可视化」链路，
    检查话题频率、分辨率、编码与相机内参是否符合预期
