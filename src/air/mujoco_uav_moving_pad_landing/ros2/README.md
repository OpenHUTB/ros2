# ROS 2 Humble 着陆控制器

本节点订阅 `/landing/features` 六维归一化特征消息，并向 `/cmd_vel` 发布限幅的 `geometry_msgs/Twist` 速度指令。Compose 会等待着陆模型生成后再启动控制节点。

```bash
colcon build --packages-select mujoco_uav_moving_pad_landing
source install/setup.bash
ros2 launch mujoco_uav_moving_pad_landing landing.launch.py
```

消息 `Float32MultiArray.data` 依次包含水平视觉误差、高度、平台水平速度和检测有效标记。Docker 环境可用 `docker compose up --build` 启动仿真与 ROS 2 控制节点。

English: The node subscribes to six normalized features on `/landing/features` and publishes bounded `geometry_msgs/Twist` commands on `/cmd_vel`.
