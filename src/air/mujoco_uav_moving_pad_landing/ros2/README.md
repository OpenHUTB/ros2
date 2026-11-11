# ROS 2 Humble 着陆控制器

ROS 2 集成由 MuJoCo 特征源和着陆控制器组成。特征源以 50 Hz 渲染 MuJoCo 场景，将图像误差、高度、平台速度和检测标记组成六维消息发布到 `/landing/features`，并订阅 `/cmd_vel` 驱动仿真位置伺服。控制器读取特征并发布限幅的 `geometry_msgs/Twist` 速度指令。

```bash
colcon build --packages-select mujoco_uav_moving_pad_landing
source install/setup.bash
ros2 launch mujoco_uav_moving_pad_landing landing.launch.py
```

启动后可分别查看输入特征和控制输出：

```bash
ros2 topic echo /landing/features
ros2 topic echo /cmd_vel
```

`Float32MultiArray.data` 顺序为水平视觉误差 X/Y、归一化高度、平台水平速度 X/Y、检测有效标记。Docker 环境可用 `docker compose up --build`；Compose 会先生成着陆模型，再启动两个 ROS 节点。

English: A 50 Hz MuJoCo feature source publishes six rendered image/pose values on `/landing/features` and applies bounded `geometry_msgs/Twist` commands from `/cmd_vel` to the simulated UAV.
