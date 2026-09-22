title: 主页

# [模拟器的 ROS 文档](https://github.com/OpenHUTB/ros2)

欢迎使用 OpenHUTB 的  ROS 文档 [@macenski2022robot]。

- [简介](#list)
  - [入门](#list)
  - [人形机器人](#humanoid)
  - [肌肉骨骼人](#musculoskeletal)
  - [地面载具](#ground_vehicle)
  - [空域载具](#air_vehicle)
  - [水域载具](#water_vehicle)

---

## 1. 入门 <span id="list"></span>

ROS 相关资料（[网盘下载地址](https://pan.baidu.com/s/1viua4SZ7tP2DtU2XlCRKPg?pwd=hutb)）：

* 教材：ROS教材.pdf
* 课件和视频：ROS资料.zip
* 安装好 ros kinetic 的虚拟机（密码：rosindustrial）Ubuntu 16.04：*.ova
* Windows虚拟机（密钥：ZF3R0-FHED2-M80TY-8QYGC-NPKYF）：*.exe
* 补充：[ubuntu下虚拟机的运行方式](ubuntu下虚拟机的运行方式.md)
* [虚拟机环境介绍](./vm_introduction.md)
* 补充：[Ubuntu 20.04 安装 ROS Noetic 详细教程](ubuntu20.04.md)



[下载](https://ww2.mathworks.cn/support/product/robotics/ros2-vm-installation-instructions-v9.html)并安装好 ROS 的虚拟机。此虚拟机基于 Linux （Ubuntu 20.04 `lsb_release -a`）操作系统，并已预先配置为支持使用 ROS （ROS 1 Noetic 和 ROS 2 Humble） 构建的应用程序。


ROS每章节运行代码:

* [第 1 章](./chapter/chap1/chap1.md) - 认识 ROS
* [第 2 章](./chapter/chap2.md) - ROS 基础
* [第 3 章](./chapter/chap3.md) - 机器人系统设计
* [第 4 章](./chapter/chap4.md) - 机器人仿真
* [第 5 章](./chapter/chap5.md) - 机器人感知
* [第 6 章](./chapter/chap6.md) - 机器人 SLAM 与自主导航
* [第 7 章](./chapter/chap7.md) - 机械臂控制
* [第 8 章](./chapter/chap8.md) - ROS 机器人综合应用
* [第 9 章](./chapter/chap9.md) - ROS 2


## 2. 人形机器人 <span id='humanoid'></span>

* [人形机器人基础](./humanoid/humanoid_basics.md)

## 3. 肌肉骨骼人 <span id='musculoskeletal'></span>

* [肌肉骨骼人基础](./musculoskeletal/musculoskeletal_basics.md)

## 4. 地面载具  <span id='ground_vehicle'></span>

* [手动控制](./set_up_and_connect_to_carla.md)
* [生成对象](./ground/carla_spawn_objects.md)
* [阿克曼控制](./ground/ackermann_control.md)
* [路径点发布器](./ground/waypoint.md)
* [自动驾驶代理](./ground/ad_agent.md)
* [自动驾驶示例](./ground/ad_demo.md)
* [ROS Scenario Runner](./ground/ros_scenario_runner.md)
* [RVIZ Carla 插件](./ground/rviz_plugin.md)
* [扭转控制](./ground/twist_to_control.md)
* [RQT 插件](./ground/rqt_plugin.md)
* [点云地图创建](./ground/pcl_recorder.md)


## 5. 空域载具 <span id='air_vehicle'></span>

* [建立虚拟机和空域载具之间的连接](./air/setup_and_connect.md)

* [空域载具的八叉树三维寻路](./air/octree_uav_3d_pathfinding/env_setup.md)

* [点云转八叉树占用地图](./air/octree_uav_3d_pathfinding/occupancy_mapping.md)

* [空域模拟器的 ROS 封装器](./air/ros_pkgs.md)

* [无人机终端键盘遥控器](./air/drone_teleop.md)

* [基于 ROS 消息解耦的无人机键盘遥控](./air/drone_ros_teleop.md)

* [低空载具的 ROS 示例教程](https://openhutb.github.io/air_doc/airsim_tutorial_pkgs/)

* [AP-CPP 主动感知覆盖路径规划](./ap_cpp_overfomo.md) —— 基于 AirSim 的无人机主动感知覆盖路径规划，在 OverFOMO 自适应覆盖规划的基础上进一步决定「该去哪里看」



## 6. 水域载具  <span id='water_vehicle'></span>

* [水域载具 ROS2 接口](./water/HoloOcean.md)
* [水下机器人物理仿真与 6-DOF 键盘运动控制](./water/rov_physical_simulation.md)

___

如果对文档中的任何问题可以在 [本文档的源码仓库](https://github.com/OpenHUTB/ros2) 中的 [问题](https://github.com/OpenHUTB/ros2/issues) 页面讨论或者提交 [拉取请求](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md) 直接修改文档。

___

## 参考文献
