CARLA 手动控制 —— 设置并连接到 CARLA 模拟器
一、项目概述
本项目基于 CARLA 0.9.14 和 ROS 2 Humble，实现了对 CARLA 模拟器中 ego_vehicle 的手动控制。通过 carla_manual_control 节点，可以用键盘控制车辆行驶、转向、刹车和倒车。

系统架构：

Windows 宿主机：运行 CARLA 0.9.14 服务器（提供 3D 渲染和物理引擎）

Ubuntu 20.04 虚拟机：运行 ROS 2 Humble 和 CARLA ROS Bridge（通过网络连接 CARLA 服务器）

功能：

手动控制（键盘 W/A/S/D 控制车辆）

中文部署文档

二、环境准备
2.1 硬件与系统
项目	配置
宿主机	Windows 10/11
虚拟机	VMware Workstation
虚拟机系统	Ubuntu 20.04.6 LTS
虚拟机内存	8 GB 以上
虚拟机处理器	4 核以上
显卡	支持 Vulkan 的独立显卡（宿主机）
2.2 软件版本
软件	版本
CARLA	0.9.14（Windows 版 + Linux 版）
ROS 2	Humble
Python	3.8
CARLA ROS Bridge	1.3.0
三、部署流程
3.1 下载 CARLA 0.9.14
Windows 版（用于宿主机服务器）：

text
https://carla-releases.s3.us-east-005.backblazeb2.com/Windows/CARLA_0.9.14.zip
解压到 D:\CARLA_0.9.14\WindowsNoEditor\。

Linux 版（用于 Ubuntu 的 Python API）：

text
https://carla-releases.s3.us-east-005.backblazeb2.com/Linux/CARLA_0.9.14.tar.gz
在 Ubuntu 上解压：

bash
cd ~
tar -xzvf CARLA_0.9.14.tar.gz
3.2 安装 CARLA ROS Bridge
1. 创建工作空间并克隆代码：

bash
mkdir -p ~/carla-ros-bridge && cd ~/carla-ros-bridge
git clone https://gitclone.com/github.com/carla-simulator/ros-bridge.git src/ros-bridge
2. 修复子模块地址：

bash
cd ~/carla-ros-bridge/src/ros-bridge
sed -i 's|https://github.com/|https://gitclone.com/github.com/|g' .gitmodules
git submodule sync
git submodule update --init --recursive
3. 安装依赖并编译：

bash
cd ~/carla-ros-bridge
source /opt/ros/humble/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r
colcon build --packages-skip rviz_carla_plugin carla_ad_demo pcl_recorder
3.3 配置 Python API 路径
在 ~/.bashrc 末尾添加：

bash
export CARLA_ROOT=$HOME/PythonAPI
export PYTHONPATH=$CARLA_ROOT/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:$CARLA_ROOT/carla:$PYTHONPATH
让改动生效：

bash
source ~/.bashrc
验证：

bash
python3 -c "import carla; print(carla.__file__)"
3.4 修改 CARLA 版本检查
ROS Bridge 默认要求 CARLA 0.9.13，需要修改为 0.9.14：

bash
echo "0.9.14" > ~/carla-ros-bridge/src/ros-bridge/carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION
cd ~/carla-ros-bridge
colcon build --packages-select carla_ros_bridge
3.5 修改 ego_vehicle 车型
编辑 objects.json，把 ego_vehicle 的车型从 vehicle.tesla.model3 改成 vehicle.audi.a2：

bash
nano ~/carla-ros-bridge/src/ros-bridge/carla_spawn_objects/config/objects.json
找到：

json
"type": "vehicle.tesla.model3",
"id": "ego_vehicle",
改成：

json
"type": "vehicle.audi.a2",
"id": "ego_vehicle",
重新编译：

bash
cd ~/carla-ros-bridge
colcon build --packages-select carla_spawn_objects
3.6 配置 VMware 网络
1. 在 VMware 里设置桥接模式：

虚拟机 → 设置 → 网络适配器 → 桥接模式：直接连接物理网络。

勾选 “复制物理网络连接状态”。

2. 在 VMware 虚拟网络编辑器里，把 VMnet0 桥接到 WLAN 网卡：

编辑 → 虚拟网络编辑器 → 更改设置 → 选中 VMnet0 → 桥接到实际的无线网卡。

3. 在 Ubuntu 里获取 IP：

bash
sudo dhclient ens33
ip addr show ens33 | grep "inet "
4. 测试连接 Windows 上的 CARLA：

bash
nc -zv 192.168.0.3 2000
显示 succeeded 即可。

四、启动流程
4.1 启动 Windows 上的 CARLA 服务器
进入 D:\CARLA_0.9.14\WindowsNoEditor\，双击 CarlaUE4.exe。

等 1-2 分钟，让它完全加载。

确认监听 2000 端口：

cmd
netstat -ano | findstr 2000
应该只有一行 LISTENING。

4.2 启动 ROS Bridge（终端 1）
bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_ETC_DIR ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI PKG_CONFIG_PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v noetic | grep -v catkin_ws | tr '\n' ':')
export PATH=${PATH%:}
export LD_LIBRARY_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/PythonAPI
export PYTHONPATH=$CARLA_ROOT/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:$CARLA_ROOT/carla:$PYTHONPATH
ros2 launch carla_ros_bridge carla_ros_bridge.launch.py host:=192.168.0.3 port:=2000 timeout:=60
4.3 启动 spawn objects 生成 ego_vehicle（终端 2）
bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_ETC_DIR ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI PKG_CONFIG_PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v noetic | grep -v catkin_ws | tr '\n' ':')
export PATH=${PATH%:}
export LD_LIBRARY_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/PythonAPI
export PYTHONPATH=$CARLA_ROOT/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:$CARLA_ROOT/carla:$PYTHONPATH
ros2 launch carla_spawn_objects carla_spawn_objects.launch.py host:=192.168.0.3 port:=2000
等它输出 All objects spawned。

4.4 启动手动控制（终端 3）
bash
unset ROS_DISTRO ROS_VERSION ROS_PYTHON_VERSION AMENT_PREFIX_PATH CMAKE_PREFIX_PATH COLCON_PREFIX_PATH PYTHONPATH LD_LIBRARY_PATH ROS_ETC_DIR ROS_PACKAGE_PATH ROS_ROOT ROS_MASTER_URI PKG_CONFIG_PATH
export PATH=$(echo $PATH | tr ':' '\n' | grep -v noetic | grep -v catkin_ws | tr '\n' ':')
export PATH=${PATH%:}
export LD_LIBRARY_PATH=/opt/ros/humble/lib
source /opt/ros/humble/setup.bash
source ~/carla-ros-bridge/install/setup.bash
export CARLA_ROOT=$HOME/PythonAPI
export PYTHONPATH=$CARLA_ROOT/carla/dist/carla-0.9.14-py3.7-linux-x86_64.egg:$CARLA_ROOT/carla:$PYTHONPATH
ros2 run carla_manual_control carla_manual_control
4.5 开车操作
按键	功能
B	激活/关闭手动驾驶
W	油门
S	刹车（车停后再按可倒车）
A	左转
D	右转
空格	手刹
P	切换自动驾驶
H	查看帮助
五、遇到的问题与解决方案
5.1 VMware Vulkan 支持不足
问题： CARLA 在 VMware 虚拟机里运行时报 GameThread timed out waiting for RenderThread，最终段错误崩溃。

解决： 把 CARLA 服务器放在 Windows 宿主机上运行，Ubuntu 虚拟机通过网络连接。

5.2 ROS 1 Noetic 与 ROS 2 Humble 环境混用
问题： ROS Bridge 启动时报 AttributeError: type object 'type' has no attribute '_TYPE_SUPPORT'。

解决： 在每个终端启动前，执行 unset 清理所有 ROS 相关变量，然后只 source /opt/ros/humble/setup.bash。同时修复 carla-ros-bridge/install/setup.sh 里写死的 /opt/ros/noetic 路径。

5.3 版本不匹配
问题： ROS Bridge 报 CARLA python module version 0.9.13 required. Found: 0.9.14。

解决： 修改 carla_ros_bridge/src/carla_ros_bridge/CARLA_VERSION 文件，把 0.9.13 改成 0.9.14，重新编译。

5.4 依赖缺失
问题： 编译时报 ModuleNotFoundError: No module named 'derived_object_msgs'。

解决：

bash
cd ~/carla-ros-bridge/src
git clone https://gitclone.com/github.com/astuff/astuff_sensor_msgs.git
cd ~/carla-ros-bridge
colcon build --packages-select derived_object_msgs
5.5 tf2 库冲突
问题： 报 undefined symbol: _ZN3tf210BufferCore15allFramesAsDotB5cxx11Ev。

解决：

bash
sudo apt remove ros-noetic-tf2 ros-noetic-tf2-eigen ros-noetic-tf2-geometry-msgs ros-noetic-tf2-kdl ros-noetic-tf2-msgs ros-noetic-tf2-py ros-noetic-tf2-ros -y
5.6 Foxy 与 Humble 库冲突
问题： 报 undefined symbol: rcutils_expand_user。

解决：

bash
sudo mv /opt/ros/foxy/lib /opt/ros/foxy/lib.bak
5.7 VMware 桥接网络问题
问题： Ubuntu 无法连接 Windows 上的 CARLA，报 Network is unreachable。

解决： 在 VMware 虚拟网络编辑器里，把 VMnet0 桥接到实际的 WLAN 网卡。

5.8 车辆卡在护栏上
问题： 车辆生成后卡在护栏上，全油门也动不了。

解决： 用 Python 脚本把车辆瞬移到空旷的生成点：

python
import carla
client = carla.Client('192.168.0.3', 2000)
client.set_timeout(30)
world = client.get_world()
vehicles = world.get_actors().filter('vehicle.*')
for v in vehicles:
    if v.attributes.get('role_name') == 'ego_vehicle':
        spawn_points = world.get_map().get_spawn_points()
        for sp in spawn_points:
            if 0.2 < sp.location.z < 0.5:
                v.set_transform(sp)
                break
        break
        ## 运行效果

以下是实际运行截图：

![CARLA 服务器窗口](images/carla_server.jpg)
![ROS Bridge 启动日志](images/ros_bridge.jpg)
![spawn objects 输出](images/spawn_objects.jpg)
![carla_manual_control 控制窗口](images/manual_control.jpg)
六、总结
项目	状态
ROS Bridge 编译	
CARLA 服务器运行	
ego_vehicle 生成	
手动控制开车	
中文部署文档	
文档作者： jasmine055
日期： 2026-10-16

