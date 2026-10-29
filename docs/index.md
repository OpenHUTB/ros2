title: 主页

# [模拟器的 ROS 文档](https://github.com/OpenHUTB/ros2)

欢迎使用 OpenHUTB 的  ROS 文档 [@macenski2022robot]。

<span id="submission-guide"></span>
**提交项目之前**，请按以下指引自查，并阅读[仓库提交约定](https://github.com/OpenHUTB/ros2#注意事项)。PR 模板只列常见检查项；具体任务、模拟器和验收要求以老师及维护者说明为准。

<details>
<summary>展开项目提交与复现指引</summary>

<ol>
<li><strong>确认提交范围。</strong> 在 Files changed 中逐个检查新增、修改和删除。不要提交 build、install、log 等无关生成目录、备份或重复文件，不用个人项目内容覆盖仓库首页及公共配置。需要修改公共内容时说明原因与影响。</li>
<li><strong>整理模块与启动入口。</strong> 源码放在 src 对应模块，教程放在 docs 对应模块，文件使用英文下划线命名。提供 main.* 和 launch，核对包名、可执行名、依赖、模型路径及安装规则，完善实际包描述、维护信息和许可证。</li>
<li><strong>按文档完整运行。</strong> README 写清实际验证的系统、ROS、Python、模拟器和关键依赖版本，以及获取源码、安装、构建、加载环境、启动和停止步骤，注明执行目录。建议从新终端操作，检查键盘输入、运动方向、状态切换、退出停车和异常清理。</li>
<li><strong>准确说明验证范围。</strong> 区分节点启动、话题通信、可视化演示与完整仿真效果。固定速度发布不等于轨迹跟踪，离线测试不等于在线闭环验证。指标注明定义、单位、场景与测量方法，保留必要日志；未验证的内容和简化假设如实说明。</li>
<li><strong>检查网页与配图。</strong> 新模块在本首页和 mkdocs.yml 增加入口，运行 <code>mkdocs serve --livereload</code> 检查相关页面、链接、公式和流程图。使用真实运行截图而非拍照，示意图明确标注；PNG 小于 3 MB、GIF 小于 10 MB，大量数据提供持久下载链接。</li>
<li><strong>完成说明与复核。</strong> PR 标题和提交信息说明修改目的，使用大模型时声明具体工具并对内容负责。合并前至少一名其他人在另一台机器测试并同意，不能用作者自测替代。纯文档或局部修复按实际影响检查，不要求新增无关模块。</li>
</ol>
<p>以上结合仓库要求和复现建议整理，不作为新增评分标准。也可参阅<a href="https://github.com/OpenHUTB/doc#撰写规范">文档撰写约定</a>和<a href="https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md">贡献指南</a>。</p>

</details>

- [简介](#list)
  - [入门](#list)
  - [人形机器人](#humanoid)
  - [肌肉骨骼人](#musculoskeletal)
  - [地面载具](#ground_vehicle)
  - [禁飞区检测模块](no_fly_zone_check/index.md)
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
* [第 7 章](./chapter/chap7.md) - 阿克曼四轮小车运动控制
* [第 8 章](./chapter/chap8.md) - ROS 机器人综合应用
* [第 9 章](./chapter/chap9.md) - ROS 2

ROS 2 话题发布订阅基础实验（talker/listener）：[chap2/ros2_pubsub_py](./chap2/ros2_pubsub_py.md)

ROS Noetic 多传感器话题时间同步：[chap2/ros1_message_time_sync](./chap2/ros1_message_time_sync/index.md)
ROS Noetic 运行参数热更新（正弦信号）：[chap2/ros1_parameter_sine_wave](./chap2/ros1_parameter_sine_wave/index.md)


## 2. 人形机器人 <span id='humanoid'></span>

* [人形机器人基础](./humanoid/humanoid_basics.md)
* [人形机器人键盘运动控制](./humanoid/humanoid_keyboard_control.md)
* [人形机器人平衡控制与外力抗扰恢复](./humanoid/humanoid_push_recovery.md)
* [人形机器人 PPO 神经网络平衡控制](./humanoid/humanoid_ppo_locomotion.md)
* [人形机器人 2D 激光雷达 CNN 感知与避障](./humanoid/humanoid_lidar_nn_perception.md)
* [人形足迹下的局部避障算法对比](./humanoid/humanoid_local_planner_bench.md)
* [MuJoCo单足弹跳机器人简单仿真](./humanoid/mujoco_hopper_demo/index.md)
* [人形上下台阶步态仿真](./humanoid/humanoid_stair_climb.md)
* [ROS虚拟机器人圆形轨迹rviz可视化示例](./humanoid/ros_topic_demo/index.md)
* [人形机器人全身逆运动学与可操作性](./humanoid/whole_body_ik.md)

## 3. 肌肉骨骼人 <span id='musculoskeletal'></span>

* [肌肉骨骼人基础](./musculoskeletal/musculoskeletal_basics.md)
* [MuJoCo 肌肉骨骼灵巧手：键盘遥操作与神经网络肌肉激活控制](./musculoskeletal/msk_hand_teleop.md)

* [肌肉驱动的关节控制与 Hill 肌肉模型](./musculoskeletal/muscle_joint_control.md)
## 4. 地面载具  <span id='ground_vehicle'></span>

* [手动控制](./set_up_and_connect_to_carla.md)
* [MNIST 手写数字视觉感知与机器人运动控制](./ground/mnist_digit_control.md) —— CNN 识别手写数字为运动指令，防抖确认后控制 turtlesim 平移/旋转
* [生成对象](./ground/carla_spawn_objects.md)
* [地面载具物理仿真与键盘运动控制](./ground/carla_keyboard_control.md)
* [CARLA+YOLOv8 目标检测感知与减速避障](./ground/carla_yolo_avoidance.md)
* [阿克曼控制](./ground/ackermann_control.md)
* [传感器感知与给定轨迹跟踪（神经网络版）](./ground/carla_perception_control.md)
* [R2D2小车自动巡航仿真](./ground/r2d2_cruise.md)
* [路径点发布器](./ground/waypoint.md)
* [激光雷达建图与神经网络规划导航](./ground/carla_mapping_navigation.md)
* [自动驾驶代理](./ground/ad_agent.md)
* [自动驾驶示例](./ground/ad_demo.md)
* [rostopic 手动驱动](./ground/rostopic_cruise.md)
* [ROS Scenario Runner](./ground/ros_scenario_runner.md)
* [RVIZ Carla 插件](./ground/rviz_plugin.md)
* [扭转控制](./ground/twist_to_control.md)
* [RQT 插件](./ground/rqt_plugin.md)
* [点云地图创建](./ground/pcl_recorder.md)
* [Carla 交通标志识别（Windows 脚本）](./ground/carla_traffic_sign_recognition.md) - 基于 YOLOv8 的实时感知与键盘控制

* [YOLOv8 视觉感知目标跟随仿真](./ground/yolov8_target_tracking.md)
* [车辆键盘遥操作与二维模拟](./ground/vehicle_pygame_teleop.md)

* [任务1：机器人键盘控制模块](./robot_tasks/teleop_control/README.md)

* [TurtleBot3 激光雷达自主避障（ROS 2）](./ground/turtlebot3_lidar_avoid.md) —— 订阅机载激光雷达，按扇区提取前/左/右最近距离，经三态状态机与势场法决策后发布 /cmd_vel，实现迷宫场景自主避障


* [MuJoCo 差速小车轮滑感知与轨迹补偿](./ground/mujoco_diff_drive_slip_compensation.md)
* [MuJoCo 地面小车射线感知与分级减速避障](./ground/mujoco_car_avoidance.md)
* [MuJoCo 地面小车神经网络端到端避障](./ground/mujoco_car_nn_avoidance.md)
* [2D 小车目标追踪仿真](./ground/car_2d_target_tracker.md)
* [基于神经网络的无人小车自适应跟车（ACC）](./ground/carla_nn_car_following.md)
* [恶劣雨雾天气下多传感器感知性能对比实验](./ground/weather_perception_benchmark.md)
* [TurtleBot3 仿真自主导航与避障参数优化](./ground/turtlebot3_auto_nav/README.md)
* [图像录制 ROS 2 封装模块](./ground/image_recorder.md)
* [悬崖边缘紧急制动仿真](./ground/cliff_brake.md) —— 用 MuJoCo 物理引擎与 ROS 三节点闭环实现车头测距触发的滞回紧急制动，附安全包络定量测试与坠落对照
* [基于 Autoencoder 的雷达数据修复与避障](./ground/lidar_autoencoder_repair.md) —— 离线训练自编码器去除传感器噪声，修复后数据送入阈值避障控制链路，展示传感器数据容错层与控制层解耦的设计
* [地面载具灯光自动控制](./ground/carla_auto_light.md)
* [轨迹/里程计精度评估（ATE/RPE）](./ground/trajectory_accuracy_eval.md)

## 5. 空域载具 <span id='air_vehicle'></span>
* [无人机着陆阶段识别与接地载荷预测](./air/landing_assessment.md)

* [无人机载荷质量估计](./air/payload_mass_estimation.md)


* [无人机任务管理与安全状态机](./air/uav_mission_safety_manager/overview.md)

* [无人机飞行轨迹类型识别与 ROS2 自动任务日志](./air/flight_pattern_recognition.md)

* [无人机遥测异常检测与 ROS2 在线诊断](./air/telemetry_diagnostics.md)

* [建立虚拟机和空域载具之间的连接](./air/setup_and_connect.md)

* [空域载具的八叉树三维寻路](./air/octree_uav_3d_pathfinding/env_setup.md)

* [点云转八叉树占用地图](./air/octree_uav_3d_pathfinding/occupancy_mapping.md)

* [八叉树上的 A* 全局寻路](./air/octree_uav_3d_pathfinding/astar_planning.md)




* [空域模拟器的 ROS 封装器](./air/ros_pkgs.md)
* [无人机与 ROS 的桥接模块](./air/carlair_ros_bridge.md)
* [无人机屋顶降落](./air/rooftop_landing.md) —— 用机载 LiDAR 检测屋顶平面，取轮廓的最大内切圆圆心作为降落点，自主飞抵其上方后垂直降落
* [无人机屋顶降落（学习型落点选择）](./air/rooftop_landing_nn.md) —— 用点云高度图 + 小 CNN 直接回归屋顶落点；凹形屋顶（U/L/天井）与稀疏点云下比凸包最大内切圆更稳，附三组消融对比
* [无人机键盘控制](./air/uav_keyboard_control.md)
* [无人机视觉导航（前视CNN行为克隆）](./air/uav_neural_perception.md)

* [基于 PPO 的无人机导航避障](./air/uav_ppo_nav.md)

* [基于 PPO 的无人机固定路线巡航](./air/uav_route_cruise.md) —— 沿给定闭合航线巡航，遇压在航线上的障碍时临时偏离绕行

* [无人机导航强化学习对比实验（PPO/SAC/TD3）](./air/uav_rl_benchmark.md)

* [神经残差动力学与学习式轨迹跟踪](./air/uav_neural_residual_control.md)
* [基于 DQN 的无人机导航避障](./air/uav_dqn_nav.md) —— 与 PPO 同环境同观测，仅替换为 DQN 离散动作算法，用于端到端模型/算法对比

* [无人机终端键盘遥控器](./air/drone_teleop.md)

* [基于 ROS 消息解耦的无人机键盘遥控](./air/drone_ros_teleop.md)

* [无人机航点轨迹跟踪（PID 控制）](./air/waypoint_tracking.md)

* [基于 GRU 的无人机飞行状态预测与 ROS2 在线误差分析](./air/state_prediction.md)

* [MuJoCo 四旋翼仿真与 ROS2 桥接底座](./air/uav_mujoco_sim.md)

* [无人机编队飞行](./air/uav_formation_flight.md) —— 长机按航点巡航、僚机保持队形（一字/纵队/三角可动态切换），三架共用一条 AirSim RPC，依次起飞、低空展开后进入任务

* [低空载具的 ROS 示例教程](https://openhutb.github.io/air_doc/airsim_tutorial_pkgs/)

* [AP-CPP 主动感知覆盖路径规划](./ap_cpp_overfomo.md) —— 基于 AirSim 的无人机主动感知覆盖路径规划，在 OverFOMO 自适应覆盖规划的基础上进一步决定「该去哪里看」

* [YOLO目标检测与八字轨迹控制](./air/openhutb_yolo_mlp_control/README.md)

* [语义分割建图与语义路径规划](./air/uav_semantic_mapping_nav.md)
* [AllocNet 四旋翼避障规划与键盘遥操作](./air/allocnet_teleop.md) —— 在学习型时间分配的避障轨迹规划基础上，补齐键盘航点下发、地图常驻转发与轨迹记录的人机交互层

* [状态识别与轨迹绘制](./air/drone_trajectory_logger.md)

* [RflySim 单机起飞与端口故障排查（补充实验）](./air/rflysim_sitl_first_flight.md)



## 6. 水域载具  <span id='water_vehicle'></span>

* [水域载具 ROS2 接口](./water/HoloOcean.md)

* [HoloOcean Windows 本机环境搭建](./water/HoloOcean-Windows-Setup.md) —— 在 Windows 上用 Conda 安装 Python 客户端、下载场景包并跑通第一个仿真

* [WSL 跑通 HoloOcean2 的 ROS2 示例（无头模式）](./water/WSL_HoloOcean2_ROS2.md) —— 无 GPU 环境下用 `-nullrhi` 参数跑无头仿真

* [Docker 开发环境](./water/docker.md) —— 开发容器与工作空间挂载

* [HoloOcean ROS2 运行时容器](./water/docker_runtime.md) —— 用 `build_container.sh` 构建运行时镜像

* [水域载具示例（手柄 / 航点 / 深度航向指令）](./water/holoocean_examples.md) —— 多代理手柄控制、相机 HUD 与配置文件结构

* [水下机器人物理仿真与 6-DOF 键盘运动控制](./water/rov_physical_simulation.md)

* [水下多传感器感知与 3D 轨迹跟踪控制](./water/rov_trajectory_tracking.md)

* [水下声呐 SLAM 占据栅格建图与自主避障导航](./water/rov_sonar_slam_navigation.md)
  
* [新增水下三维地形测绘模块](./water/underwater_sonar_mapping.md)

* [动态洋流 ROV 键盘遥操与 MLP 巡航控制](./water/mjc_rov_current_teleop.md)

* [侧扫声呐图像仿真与 CNN 海底目标分类](./water/mjc_rov_sonar_cnn.md)

___

如果对文档中的任何问题可以在 [本文档的源码仓库](https://github.com/OpenHUTB/ros2) 中的 [问题](https://github.com/OpenHUTB/ros2/issues) 页面讨论或者提交 [拉取请求](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md) 直接修改文档。

___

## 参考文献
