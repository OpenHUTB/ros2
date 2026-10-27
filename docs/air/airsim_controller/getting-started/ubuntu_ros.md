# Ubuntu 20.04 + ROS Noetic 安装

## 1. 系统准备

```bash
sudo apt update && sudo apt upgrade
sudo apt install -y build-essential git python3-pip python3-dev
```

## 2. 安装 ROS Noetic

```bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt install -y curl
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
sudo apt update
sudo apt install -y ros-noetic-desktop-full
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init && rosdep update
```

验证：

```bash
roscore        # 能启动即成功
```

## 3. 编译 AirSim ROS 桥（可选，任务3 需要）

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
git clone https://github.com/microsoft/AirSim.git
cd AirSim && git checkout v1.8.1
# 按官方文档编译 AirSim 与 ros_wrapper
cd ~/catkin_ws && catkin_make
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc && source ~/.bashrc
```

## 4. 把本仓库放进工作空间

```bash
cd ~/catkin_ws/src
git clone https://github.com/thechaos16/airsim_controller.git
cd ~/catkin_ws && catkin_make
source devel/setup.bash
```

## 5. 安装 Python 依赖

```bash
cd ~/catkin_ws/src/airsim_controller
pip3 install -r requirements.txt
```

## 6. 一键验证（launch）

```bash
roslaunch airsim_controller task1_keyboard.launch
```

> 截图占位：`assets/noetic_running.png`（终端打印 `Connected!` 并进入键盘控制）。
