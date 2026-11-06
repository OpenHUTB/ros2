# mjc_rov_ppo_tracking

手写 PPO 强化学习 ROV 轨迹跟踪，与 PID 基线对比。

## 运行环境
- Ubuntu 20.04.6 LTS，ROS 2 Humble，Python 3.8.10
- MuJoCo 3.1.x（OSMesa），PyTorch 2.4.1+cpu

## 运行步骤
```bash
colcon build --symlink-install --packages-select mjc_rov_ppo_tracking
source install/setup.bash
bash main.sh                          # 全链路 train + test
ros2 launch mjc_rov_ppo_tracking main.launch.py
```

## 话题
| 话题 | 类型 | 说明 |
|---|---|---|
| /rov_ppo/odom | nav_msgs/Odometry | ROV 位姿 |
| /rov_ppo/cmd_vel | geometry_msgs/Twist | PPO 推力输出 |
