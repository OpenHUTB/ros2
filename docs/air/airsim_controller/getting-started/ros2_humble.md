# ROS2 Humble 与 AirSim ROS2 示例

## 1. 安装 ROS2 Humble（Ubuntu 22.04）

```bash
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update && sudo apt install -y ros-humble-desktop python3-colcon-common-extensions
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 2. 在 Humble 上运行 AirSim 的 ROS2 示例

AirSim 自带 `ros2_wrapper`（位于 `AirSim/ros2`）：

```bash
cd ~/airsim_ws/src
git clone https://github.com/microsoft/AirSim.git -b v1.8.1
# 把 ros2_wrapper 软链进工作空间
ln -s ~/AirSim/ros2/airsim_ros_pkgs ~/airsim_ws/src/
cd ~/airsim_ws && rosdep install --from-paths src -y --ignore-src
colcon build --packages-select airsim_ros_pkgs
source install/setup.bash

# 启动 AirSim 的 ROS2 桥（发布 /depth_image, /lidar, /odom 等）
ros2 launch airsim_ros_pkgs airsim_node.launch.py
```

另开终端验证话题：

```bash
ros2 topic list | grep -E "depth|lidar|odom"
ros2 topic hz /depth_image
```

## 3. 本仓库在 Humble 下运行

本仓库核心控制节点不依赖 rospy（惰性导入），可直接以 Python 脚本运行：

```bash
python3 modules/task1_keyboard/main.py        # 键盘
python3 modules/task2_perception_control/main.py --mode avoid
```

如需在 Humble 中以 `ros2 run` 启动，可将 `ros_nodes/*.py` 包装为
`setup.py` 的 `entry_points`（标准 ROS2 Python 包写法），接口保持不变。

> 截图占位：`assets/humble_topics.png`（`ros2 topic list` 中出现 AirSim 话题）。
