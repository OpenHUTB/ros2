# 前视 CNN 行为克隆视觉导航

对应作业任务（2）**神经网络感知与行为克隆**。

`/camera/image_raw` → CNN → `/uav/cmd_vel`。前视 RGB 图像直接输入卷积网络，
输出机体系速度指令，中间没有手工特征提取、没有地图、没有路径搜索。

权重缺失或加载失败时节点直接退出（fatal + exit 1），不创建发布者。
输出 NaN/inf 或长度异常时丢帧不下发；合法但超界的输出按 `max_vx/max_vy/max_yaw` 限幅。

---

## 1. 模块定位

| 维度 | 说明 |
|---|---|
| 输入 | 前视 RGB 图像（sensor_msgs/Image）|
| 输出 | 机体系速度指令（geometry_msgs/Twist → `/uav/cmd_vel`）|
| 框架 | PyTorch（CNN 行为克隆）|
| 节点 | `perception`（rclpy ament_python）|
| ROS 版本 | Humble |

当前模型只做行为克隆直线跟随，**没有主动避障/绕障**。碰撞急停由
`uav_e2e_rl` 或 bridge 层兜底。

## 2. 运行

```bash
# Noetic 终端：起 bridge
roslaunch carlair_ros_bridge bringup.launch

# Humble 终端
ros2 launch uav_neural_perception perception.launch.py
# 或
ros2 run uav_neural_perception perception
```

## 3. 参数

`config/perception.yaml`：

| 参数 | 默认 | 说明 |
|---|---|---|
| `model_path` | 空 | 留空用 share 目录下 `models/best_drone_model.pth` |
| `max_vx` / `max_vy` / `max_yaw` | 2.0 / 0.5 / 0.4 | 三轴限幅 |
| `gain_vx` / `gain_vy` / `gain_yaw` | 1.0 | 在线增益（限幅前施加）|
| `cmd_vel_topic` | `/uav/cmd_vel` | 速度话题 |

## 4. 文件结构

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

## 5. 测试

```bash
python3 test/test_perception_local.py
```

18 个用例跑过（torch 相关的 1 个在 CI 没装 torch 时自动 skip）。
