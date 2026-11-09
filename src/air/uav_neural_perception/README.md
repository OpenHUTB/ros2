# uav_neural_perception

前视 CNN 行为克隆视觉导航：`/camera/image_raw` → CNN → `/uav/cmd_vel`。**ROS2 Humble (ament_python)**。

## 安全约束

- 权重缺失或加载失败时节点**安全停止**（fatal + exit 1），不创建发布者、不进入推理循环；
- 随机权重只在离线自检显式 `allow_random=True` 时使用，部署侧固定 `False`；
- 模型输出 NaN/inf/长度异常时丢弃本帧，不下发速度；
- 合法但超界的输出经 `sanitize_action` 限幅到 `max_vx/max_vy/max_yaw`。

## 运行

```bash
# Noetic 终端：bridge
roslaunch carlair_ros_bridge bringup.launch

# Humble 终端
ros2 launch uav_neural_perception perception.launch.py
# 或
ros2 run uav_neural_perception perception
```

## 参数

`config/perception.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `model_path` | 空 | 留空用 share 目录下 `models/best_drone_model.pth` |
| `max_vx` / `max_vy` / `max_yaw` | 2.0/0.5/0.4 | 三轴限幅 |
| `gain_vx` / `gain_vy` / `gain_yaw` | 1.0 | 在线增益（限幅前施加） |
| `cmd_vel_topic` | `/uav/cmd_vel` | 速度话题 |

## 文件

```
uav_neural_perception/
  safety.py         纯逻辑：权重加载校验、输出限幅/拦截（不依赖 ROS/torch）
  model.py          CNN 模型（torch）
  preprocess.py     Image 解码 + resize + ImageNet 归一化（numpy，无 cv2/cv_bridge）
  perception_node.py rclpy 部署节点
launch/perception.launch.py
config/perception.yaml
test/test_perception_local.py
```

## 测试

```bash
python3 test/test_perception_local.py
```

18 个用例跑过（torch 相关的 1 个在 CI 没装 torch 时自动 skip）。覆盖：sanitize_action 限幅/NaN/inf/长度异常、权重缺失抛错、Image 解码（rgb8/bgr8/mono8）、normalize 往返、包结构静态断言（ament_python、无 CMakeLists、无 rospy/CvBridge）。

## 说明

当前模型只做行为克隆直线跟随，**没有主动避障/绕障**。碰撞急停由 `uav_e2e_rl` 或 bridge 层兜底。
