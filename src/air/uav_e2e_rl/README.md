# uav_e2e_rl

端到端视觉导航：相机图像 → CNN+PPO → 速度指令。**ROS2 Humble (ament_python)**，通过 ros1_bridge 接 Noetic bridge。

## 架构

```
/camera/image_raw (1,48,64 灰度) → CNN → PPO → /uav/cmd_vel
/camera/depth → 前向扇区最小深度 → 碰撞检测（纯视觉，不用 /scan）
/uav/odom → 实际前向位移 → 奖励
```

## 文件

```
uav_e2e_rl/
  e2e_logic.py      纯逻辑：图像预处理、深度碰撞、奖励计算（不 import rclpy/cv2，可单测）
  train_env.py      gymnasium 训练环境（rclpy 节点）
  deploy_node.py    部署节点，加载 PPO 权重实时推理
launch/e2e_deploy.launch.py
config/e2e.yaml
test/test_e2e_logic_local.py
requirements.txt
```

## 训练

```bash
pip install -r requirements.txt
ros2 run uav_e2e_rl train --ros-args -p timesteps:=100000
```

- 观测：48×64 灰度图，[0,1]，channel-first
- 动作：[linear_x ∈ [0,2] m/s, angular_z ∈ [-1,1] rad/s]
- 奖励：实际前向位移 ×0.5 − 碰撞 5.0 − 转向 0.1×|wz|
- PPO：CnnPolicy，n_steps=256，batch=64，lr=3e-4，seed=42，total_timesteps=100000
- 每个 episode 最多 500 步截断（truncated）

## 部署

```bash
ros2 launch uav_e2e_rl e2e_deploy.launch.py
```

- 权重路径 `model_path` 参数留空时用 `share/uav_e2e_rl/models/ppo_e2e.zip`
- 第一帧图像到之前不预测（避免全黑图乱输出）
- 动作裁剪到 [0,2] / [-1,1]

## 碰撞检测

从 `/camera/depth`（32FC1，米）取屏幕下半部中间 60% 宽度的 ROI，最小深度 < `collision_depth`（默认 0.6 m，可在 config/e2e.yaml 调）算撞车。不用激光雷达，保持纯视觉。

## 测试

```bash
python3 test/test_e2e_logic_local.py   # 11 个用例，不依赖 ROS
```

CI 直接跑。

## 依赖

见 `requirements.txt`：numpy、opencv-python、stable-baselines3[extra]、gymnasium。
