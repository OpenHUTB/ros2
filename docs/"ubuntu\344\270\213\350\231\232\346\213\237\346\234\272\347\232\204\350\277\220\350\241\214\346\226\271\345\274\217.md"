# [Ubuntu 下虚拟机的运行方式](https://ww2.mathworks.cn/support/product/robotics/ros2-vm-installation-instructions-v9.html)

本页面介绍 Robotics System Toolbox™ 和 ROS Toolbox™ 配套虚拟机的安装说明。您可以使用 MATLAB® 和 Simulink® 配合 [Gazebo](https://gazebosim.org/) 机器人模拟器，以及与外部 [ROS](https://www.ros.org/)（Robot Operating System，机器人操作系统）和 [ROS 2](https://docs.ros.org/en/rolling/) 网络协同工作。

Ubuntu® Focal 20.04 虚拟机可运行于多种平台（Windows®、Mac 和 Linux®），其中包含以下内容：

* [ROS 2 Humble](https://docs.ros.org/en/humble/Installation.html) 桌面版安装
* [ROS Noetic](https://wiki.ros.org/noetic) 桌面版安装
* [Gazebo](https://gazebosim.org/) 机器人模拟器 11.0.0
* 用于模拟 [TurtleBot® 3](http://turtlebot3.robotis.com/) 的示例 Gazebo 世界

支持 64 位 Windows、64 位 Linux 和 64 位 Mac OS X 平台。安装说明按您的主机平台列出。

## 特定平台的安装说明

### Windows（64 位）

1. 访问 [broadcom.com](https://www.broadcom.com/)。
2. 在右上角，单击 “Support Portal”（支持门户）。
3. 通过单击 “Go To Portal”（进入门户）登录，或 “Register”（注册）一个基本的 Broadcom 账户。
4. 登录后，找到 “Software”（软件）下拉菜单。从中选择 VMware Cloud Foundation 部门，并单击 “My Downloads”（我的下载）。
5. 选择 VMware Workstation，并选择所需版本。
6. 下载并安装 VMware Player。
7. 下载包含虚拟机的[压缩包](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v9/ros_noetic_humble_gazebov11_linux_win_v1.zip)。
8. 将压缩包解压到硬盘上的某个位置。
9. 启动 VMware Player。
10. 在 VMware Player 中，按下 *Open a Virtual Machine*（打开虚拟机）。
11. 浏览到 Ubuntu 镜像所在位置，选择 `ros_noetic_foxy_gazebov11.vmx` 文件，然后按下 *OK*（确定）。
12. 虚拟机现已添加到您的库中。
13. 在 VMware Player 中，启动虚拟机。
14. 如果弹出窗口询问您是否复制或移动了虚拟机，请按下 *I copied it*（我已复制它）。

### Linux（64 位）

1. 访问 [broadcom.com](https://www.broadcom.com/)。
2. 在右上角，单击 “Support Portal”（支持门户）。
3. 通过单击 “Go To Portal”（进入门户）登录，或 “Register”（注册）一个基本的 Broadcom 账户。
4. 登录后，找到 “Software”（软件）下拉菜单。从中选择 VMware Cloud Foundation 部门，并单击 “My Downloads”（我的下载）。
5. 选择 VMware Workstation，并选择所需版本。
6. 以管理员权限执行 bundle 安装程序来安装 VMware Player。
7. 下载包含虚拟机的[压缩包](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v9/ros_noetic_humble_gazebov11_linux_win_v1.zip)。
8. 将压缩包解压到硬盘上的某个位置。
9. 启动 VMware Player。
10. 在 VMware Player 中，按下 *Open a Virtual Machine*（打开虚拟机）。
11. 浏览到 Ubuntu 镜像所在位置，选择 `ros_noetic_foxy_gazebov11.vmx` 文件，然后按下 *OK*（确定）。
12. 虚拟机现已添加到您的库中。
13. 在 VMware Player 中，启动虚拟机。
14. 如果弹出窗口询问您是否复制或移动了虚拟机，请按下 *I copied it*（我已复制它）。

### Mac OS X（64 位）

由于 VMware Player 不适用于 Mac，此平台使用 [VirtualBox®](https://download.virtualbox.org/virtualbox/6.1.26/VirtualBox-6.1.26-145957-OSX.dmg) 运行虚拟机。如果您拥有 [VMware Fusion®](https://www.vmware.com/products/fusion/fusion-evaluation.html) 的有效许可证，则可以改按 Windows 的安装说明操作。

1. 为 OS X 主机下载并安装 [VirtualBox®](https://download.virtualbox.org/virtualbox/6.1.26/VirtualBox-6.1.26-145957-OSX.dmg)（[许可证](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html)）。
2. 将[虚拟机](https://ssd.mathworks.com/supportfiles/ros/virtual_machines/v9/ros_noetic_humble_gazebov11_mac_v1.ova)下载到硬盘上的某个文件夹中。
3. 启动 VirtualBox。
4. 在 VirtualBox 中，选择 *File*（文件）菜单中的 *Import Appliance*（导入虚拟机）条目。
5. 选择您刚刚下载的文件，然后按下 *Next*（下一步）。
6. 确认虚拟机设置，然后按下 *Import*（导入）。导入过程可能需要几分钟。
7. 虚拟机现已添加到您的库中。
8. 在 VirtualBox 中，启动虚拟机。
9. 根据主机的网络配置，您可能需要调整虚拟机的网络设置。如果首次启动时虚拟机提示未找到网络接口，请按下 *Change Network Settings*（更改网络设置），并选择您主机主网络适配器的 *Name*（名称）。

## 使用虚拟机

请参阅 [ROS Toolbox 示例](/help/ros/examples.html)和 [Robotics System Toolbox 示例](/help/robotics/examples.html)，了解如何使用此虚拟机。

## 故障排查

* 要运行虚拟机，必须在 BIOS 中启用处理器的虚拟化扩展（有关更多信息，请参阅[此文章](https://www.howtogeek.com/213795/how-to-enable-intel-vt-x-in-your-computers-bios-or-uefi-firmware/)）。
* 默认情况下，虚拟机使用 2 个 CPU 核心，并最多分配 4096 MB 内存。如果您的计算机不支持这些默认设置，则必须在启动虚拟机之前修改虚拟机设置。
* 要在 MATLAB 与虚拟机之间启用 ROS 通信，您可能需要禁用防火墙或杀毒软件。ROS 会为节点任意分配端口号，因此根据您的防火墙配置，节点之间的通信可能会被阻止。

## 参考

* [ROS Noetic and ROS 2 Humble and Gazebo — MATLAB &amp; Simulink](https://ww2.mathworks.cn/support/product/robotics/ros2-vm-installation-instructions-v9.html)
* [设置并连接到 Carla 模拟器](./set_up_and_connect_to_carla.md)
