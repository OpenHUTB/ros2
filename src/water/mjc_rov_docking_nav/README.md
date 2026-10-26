# mjc_rov_docking_nav

水下 ROV 对接任务：声呐局部扫描 + CNN 对接标记检测 + 逼近导航。

## 运行环境
Ubuntu 20.04.6 LTS，ROS 2 Humble，Python 3.8.10，MuJoCo 3.1.x，PyTorch 2.4.1+cpu

## 运行步骤
```bash
colcon build --symlink-install --packages-select mjc_rov_docking_nav
source install/setup.bash
bash main.sh
ros2 launch mjc_rov_docking_nav main.launch.py
```
