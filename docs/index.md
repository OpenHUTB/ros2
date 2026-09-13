title: 主页

# [模拟器的 ROS 文档](https://github.com/OpenHUTB/ros2)

欢迎使用 OpenHUTB 的  ROS 文档 [@macenski2022robot]。

- [简介](#list)
  - [入门](#introduction)
  - [地面载具](#ground_vehicle)
  - [空域载具](#air_vehicle)

---

## 1. 入门 <span id="list"></span>

ROS相关资料（[网盘下载地址](https://pan.baidu.com/s/1viua4SZ7tP2DtU2XlCRKPg?pwd=hutb)）：

* 教材：ROS教材.pdf
* 课件和视频：ROS资料.zip
* 安装好 ros kinetic 的虚拟机（密码：rosindustrial）Ubuntu 16.04：*.ova
* Windows虚拟机（密钥：ZF3R0-FHED2-M80TY-8QYGC-NPKYF）：*.exe
* 补充：[ubuntu下虚拟机的运行方式](ubuntu下虚拟机的运行方式.md)


### 1.1 Windows系统（通过虚拟机运行） 

[下载](https://ww2.mathworks.cn/support/product/robotics/ros2-vm-installation-instructions-v9.html)并安装好 ROS 的虚拟机。此虚拟机基于 Linux （Ubuntu 20.04 `lsb_release -a`）操作系统，并已预先配置为支持使用 ROS （ROS 1 Noetic 和 ROS 2 Humble） 构建的应用程序。


ROS每章节运行代码:

[第二章](<./Run_code_for_%20the_%20chapter/ROS理论与实践第二章代码运行.md>)


## 2. 地面载具  <span id='ground_vehicle'></span>

* [建立虚拟机和地面载具之间的连接：手动控制](./set_up_and_connect_to_carla.md)
* [生成对象](./ground/carla_spawn_objects.md)


## 3. 空域载具 <span id='air_vehicle'></span>

* [建立虚拟机和空域载具之间的连接](./air/setup_and_connect.md)

* [空域模拟器的 ROS 封装器](./air/ros_pkgs.md)

* [低空载具的 ROS 示例教程](https://openhutb.github.io/air_doc/airsim_tutorial_pkgs/)


## 4. 水域载具

* [水域载具 ROS2 接口](./water/HoloOcean.md)

___

如果对文档中的任何问题可以在 [本文档的源码仓库](https://github.com/OpenHUTB/ros2) 中的 [问题](https://github.com/OpenHUTB/ros2/issues) 页面讨论或者提交 [拉取请求](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md) 直接修改文档。

## 参考文献
