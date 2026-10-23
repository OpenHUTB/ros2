# mjc_rov_rl_compare

同一 ROV 任务上 PPO 与 SAC 强化学习算法对比。

## 运行环境
Ubuntu 20.04.6 LTS，ROS 2 Humble，Python 3.8.10，PyTorch 2.4.1+cpu

## 运行步骤
```bash
colcon build --symlink-install --packages-select mjc_rov_rl_compare
source install/setup.bash
bash main.sh
ros2 launch mjc_rov_rl_compare main.launch.py
```
