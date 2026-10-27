# mjc_rov_sonar_cnn

侧扫声呐图像仿真与 CNN 海底目标分类（礁石 / 管道 / 残骸三分类）。

## 运行环境
- Ubuntu 20.04.6 LTS，ROS 2 Humble，Python 3.8.10
- MuJoCo 3.1.x（OSMesa 离屏渲染），PyTorch 2.4.1+cpu

## 运行步骤
```bash
colcon build --symlink-install --packages-select mjc_rov_sonar_cnn
source install/setup.bash

# 全链路：训练 + 测试
bash main.sh

# ROS 实时仿真
ros2 launch mjc_rov_sonar_cnn main.launch.py
# 另开终端
ros2 run mjc_rov_sonar_cnn classify_node
```

## 话题
| 话题 | 类型 | 说明 |
|---|---|---|
| /rov_sonar/image_raw | sensor_msgs/Image (mono8, 64x64) | 合成声呐图 |
| /rov_sonar/odom | nav_msgs/Odometry | ROV 位姿 |
| /rov_sonar/classification | std_msgs/String | CNN 分类结果 |
