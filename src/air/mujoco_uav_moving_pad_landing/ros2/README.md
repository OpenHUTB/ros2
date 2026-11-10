# ROS 2 Humble 着陆控制器

本集成将六维归一化特征消息转换为有界 `geometry_msgs/Twist` 速度指令。它订阅 `/landing/features`，发布 `/cmd_vel`；相机与里程计特征由外部节点提供。

```bash
colcon build --packages-select mujoco_uav_moving_pad_landing
source install/setup.bash
ros2 launch mujoco_uav_moving_pad_landing landing.launch.py
```

消息 `Float32MultiArray.data` 需要依次包含六个有限值。长度不符或包含非有限值时，节点发布零速度。Docker 环境可用 `docker compose up --build` 构建。

English: The adapter consumes six normalized features from `/landing/features` and publishes bounded velocity commands on `/cmd_vel`. Sensor acquisition remains an external ROS 2 node.
