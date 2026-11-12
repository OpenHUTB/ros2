# uav_e2e_rl

端到端视觉导航：相机图像 → CNN+PPO → 速度指令。**ROS2 Humble (ament_python)**。

## 架构

```
/camera/image_raw (1,48,64 灰度) → CNN → PPO → /uav/cmd_vel
/camera/depth → 前向扇区最小深度 → 碰撞急停（训练和部署都用，纯视觉，不用 /scan）
/uav/odom → 实际前向位移 → 训练奖励
```

## 文件

```
uav_e2e_rl/
  e2e_logic.py      纯逻辑：图像预处理、深度碰撞、奖励计算（不 import rclpy/cv2，可单测）
  train_env.py      gymnasium 训练环境（rclpy 节点，参数从 ROS 读）
  deploy_node.py    部署节点，加载 PPO 权重实时推理 + 深度急停
launch/e2e_deploy.launch.py
config/e2e.yaml
test/test_e2e_logic_local.py
requirements.txt
```

## 和 Noetic bridge 的连接

`carlair_ros_bridge` 是 Noetic 包，本包是 Humble，两边通过 `ros1_bridge` 通信。三个话题名字两边一致，用 bridge 的默认映射即可，不需要额外写 mapping：

| ROS1 (bridge 发出) | 类型 | ROS2 侧用途 |
|---|---|---|
| `/camera/image_raw` | sensor_msgs/Image | 观测输入 |
| `/camera/depth` | sensor_msgs/Image (32FC1) | 碰撞判断 |
| `/uav/odom` | nav_msgs/Odometry | 训练奖励 |
| `/uav/cmd_vel` | geometry_msgs/Twist | 本包输出 |

```bash
# Noetic 终端
roslaunch carlair_ros_bridge bringup.launch
roslaunch ros1_bridge ros1_bridge.launch
# Humble 终端
ros2 topic list | grep -E "camera|uav"    # 确认四个话题都跨过来了
```

如果 bridge 的话题名和上表不一致，需要写一个 mapping yaml（格式见 ros1_bridge README）。本仓库的 bridge 是上表这套名字。

## 训练

```bash
pip install -r requirements.txt
# 训练超参都是 ROS 参数，节点名 e2e_rl_train
ros2 run uav_e2e_rl train --ros-args -p timesteps:=100000 -p n_steps:=256 -p seed:=42
# 或者用参数文件
ros2 run uav_e2e_rl train --ros-args --params-file config/e2e.yaml
```

训练环境的参数（img_w/img_h/collision_depth/max_steps 等）在 `config/e2e.yaml` 的 `e2e_rl_train` 段。

- 观测：48×64 灰度图，[0,1]，channel-first
- 动作：[linear_x ∈ [0,2] m/s, angular_z ∈ [-1,1] rad/s]
- 奖励：实际前向位移 ×0.5 − 碰撞 5.0 − 转向 0.1×|wz|
- PPO：CnnPolicy，n_steps=256，batch=64，lr=3e-4，seed=42，total_timesteps=100000
- 每个 episode 最多 `max_steps=500` 步截断（truncated）

## 部署

```bash
ros2 launch uav_e2e_rl e2e_deploy.launch.py
```

- 权重路径 `model_path` 参数留空时，用 `ament_index_python.get_package_share_directory` 定位 `share/uav_e2e_rl/models/ppo_e2e.zip`
- 第一帧图像到之前不预测（避免全黑图乱输出）
- 动作裁剪到 [0,2] / [-1,1]
- 急停：`/camera/depth` 前向扇区最小深度 < `collision_depth`（默认 0.6 m）时直接发零速，不管模型输出什么

## 参数

`config/e2e.yaml` 分两段，段名必须等于节点名：

- `e2e_rl_deploy`：`model_path`、`img_w`、`img_h`、`collision_depth`
- `e2e_rl_train`：`img_w`、`img_h`、`forward_gain`、`collision_penalty`、`yaw_penalty`、`collision_depth`、`max_steps`

## 测试

```bash
python3 test/test_e2e_logic_local.py   # 11 个用例，不依赖 ROS
```

CI 直接跑。

## 依赖

见 `requirements.txt`：numpy、stable-baselines3[extra]、gymnasium。图像处理全部用 numpy，没有 opencv 依赖。
