title: 主页

# [模拟器的 ROS 文档](https://github.com/OpenHUTB/ros2)

欢迎使用 OpenHUTB 的 ROS 文档 [@macenski2022robot]。

文档环境初始化：[Ubuntu 一键安装与本地预览](./ubuntu_docs_setup.md)。

<span id="submission-guide"></span>
**提交项目之前**，请按以下指引自查，并阅读[仓库提交约定](https://github.com/OpenHUTB/ros2#注意事项)。PR 模板只列常见检查项；具体任务、模拟器和验收要求以仓库约定及维护者说明为准。

<details>
<summary>展开项目提交与复现指引</summary>

<ol>
<li><strong>确认提交范围。</strong> 在 Files changed 中逐个检查新增、修改和删除。不要提交 build、install、log 等无关生成目录、备份或重复文件，不用个人项目内容覆盖仓库首页及公共配置。需要修改公共内容时说明原因与影响。</li>
<li><strong>整理模块与启动入口。</strong> 源码放在 src 对应模块，教程放在 docs 对应模块，文件使用英文下划线命名。提供 main.* 和 launch，核对包名、可执行名、依赖、模型路径及安装规则，完善实际包描述、维护信息和许可证。</li>
<li><strong>按文档完整运行。</strong> README 写清实际验证的系统、ROS、Python、模拟器和关键依赖版本，以及获取源码、安装、构建、加载环境、启动和停止步骤，注明执行目录。建议从新终端操作，检查键盘输入、运动方向、状态切换、退出停车和异常清理。</li>
<li><strong>准确说明验证范围。</strong> 区分节点启动、话题通信、可视化演示与完整仿真效果。固定速度发布不等于轨迹跟踪，离线测试不等于在线闭环验证。指标注明定义、单位、场景与测量方法，保留必要日志；未验证的内容和简化假设如实说明。</li>
<li><strong>检查网页与配图。</strong> 新模块在本首页和 mkdocs.yml 增加入口，运行 <code>mkdocs serve --livereload</code> 检查相关页面、链接、公式和流程图。使用真实运行截图而非拍照，示意图明确标注；PNG 小于 3 MB、GIF 小于 5 MB，大量数据提供持久下载链接。</li>
<li><strong>完成说明与复核。</strong> PR 标题和提交信息说明修改目的，使用大模型时声明具体工具并对内容负责。合并前至少一名其他人在另一台机器测试并同意，不能用作者自测替代。纯文档或局部修复按实际影响检查，不要求新增无关模块。</li>
</ol>
<p>以上结合仓库要求和复现建议整理，不作为新增评分标准。也可参阅<a href="https://github.com/OpenHUTB/doc#撰写规范">文档撰写约定</a>和<a href="https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md">贡献指南</a>。</p>

</details>

- [简介](#list)
  - [入门](#list)
  - [人形机器人](#humanoid)
  - [肌肉骨骼人](#musculoskeletal)
  - [地面载具](#ground_vehicle)
  - [空域载具](#air_vehicle)
  - [水域载具](#water_vehicle)
  - [机械臂操作](#manipulation)
  - [路径规划与跟踪](#motion_planning)

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
* [小海龟键盘遥控与 ROS 基础实验](./chapter/chap1/turtlesim_teleop.md) —— 用方向键发布 /turtle1/cmd_vel，练习节点、话题、消息与 roslaunch 一键启动
* [第 2 章](./chapter/chap2.md) - ROS 基础
    * [ROS 2 话题发布订阅基础实验（talker/listener）](./chap2/ros2_pubsub_py.md)
    * [ROS Noetic 多传感器话题时间同步](./chap2/ros1_message_time_sync/index.md)
    * [ROS Noetic 运行参数热更新（正弦信号）](./chap2/ros1_parameter_sine_wave/index.md)
    * [ROS Noetic 环境自检与常见问题排查](./chap2/ros1_env_check/index.md)
    * [ROS 2 传感器数据均值滤波节点 (mean_filter)](./chap2/mean_filter.md)
* [第 3 章](./chapter/chap3.md) - 机器人系统设计
* [第 4 章](./chapter/chap4.md) - 机器人仿真
    * [URDF 四轮小车建模与 RViz 显示](./chapter/urdf_car_modeling.md)
* [第 5 章](./chapter/chap5.md) - 机器人感知
    * [fire_detect 构建与 ament 资源索引](./chap5/fire_detect.md) —— 资源索引从目录形式改为空文件，`setup.py install` 从 `can't copy 'resource/fire_detect'`（exit 1）变为 exit 0
    * [行人隐私脱敏视觉检测节点](./privacy_mask/README.md)
    * [基于激光雷达扫描的神经网络障碍物分类](./chapter/chap5/lidar_obstacle_nn.md)
* [第 6 章](./chapter/chap6.md) - 机器人 SLAM 与自主导航
    * [自建小车 Gazebo GMapping 建图](./chapter/urdf_car_gmapping.md)
* [第 7 章](./chapter/chap7.md) - 阿克曼四轮小车运动控制
* [第 8 章](./chapter/chap8.md) - ROS 机器人综合应用
    * [机器视觉多机器人蚁群编队](./ant/README.md)
    * [PocketSphinx 语音控制小乌龟](./voice/pocketsphinx_noetic.md)
    * [MoveIt 六自由度机械臂运动规划](./chapter/chap8/moveit_arm_planning.md)
    * [机械臂拧螺丝仿真](./chapter/chap8/mujoco_arm_screw.md)
    * [MuJoCo UR5e 关节空间抓放控制](./chapter/ur5e_pickplace.md)
    * [MuJoCo 二连杆笛卡尔阻抗力控](./chapter/two_link_impedance_control.md)
    * [机械臂投篮入筐仿真](./chapter/mujoco_arm_ball.md)
    * [MuJoCo 二连杆机械臂抓取搬运](./chapter/mujoco_arm_pick.md)
    * [三自由度机械臂空间螺旋线仿真](./chapter/mujoco_arm_3d.md)
    * [MuJoCo 双自由度机械臂画正方形](./chapter/mujoco_two_dof_arm_square.md)
    * [MuJoCo 二连杆机械臂 IK 画圆](./chapter/mujoco_arm_ik.md)
* [第 9 章](./chapter/chap9.md) - ROS 2
    * [Gazebo 仿真相机与图像话题验证](./chapter/chap9/gazebo_camera_sim.md)
    * [ROV 六自由度推力分配](./chapter/chap9/rov_thruster_allocation_ls.md)
    * [无人机禁飞区膨胀绕行](./chapter/chap9/uav_geofence_bypass.md)
    * [无人机 LOS 航点制导](./chapter/chap9/uav_los_waypoint_guidance.md)
    * [倒车纯追踪路径跟踪](./chapter/chap9/pure_pursuit_reverse.md)
    * [激光雷达回波强度辅助可通行性](./chapter/chap9/lidar_intensity_traversability.md)
* [第 10 章](./chapter/turtlebot3_gazebo_control.md) - Turtlebot3 Gazebo键盘控制


ROS 2 服务与参数基础实验（无人机模式切换与在线调参）：[chap2/ros2_service_param_demo](./chap2/ros2_service_param_demo.md)

ROS 2 QoS 兼容性检查实验：[chap2/ros2_qos_compatibility_lab](./chap2/ros2_qos_compatibility_lab.md)

ROS 2 Action 斐波那契数列计算示例：[chap2/fibonacci_action](./chap2/fibonacci_action.md)

## 2. 人形机器人 <span id='humanoid'></span>
* [人形机器人离线语音指令控制：端到端关键词识别（KWS）](./humanoid/humanoid_kws_command.md)
* [具身人形物理仿真与平衡行走控制](./humanoid/embodied_humanoid_control.md) —— MuJoCo 双足人形：键盘遥控、激光/深度感知、参数化步态与根节点平衡控制
* [关节 Stribeck 摩擦辨识与前馈补偿](./humanoid/joint_friction_identification.md) —— 恒速扫掠 + 网格/最小二乘辨识 Stribeck 四参数（库仑 −1.7%、静摩擦 −1.0%），前馈补偿把正弦跟踪 RMS 从 0.00703 降到 0.00310 rad（改善 55.9%）

* [人形机器人基础](./humanoid/humanoid_basics.md)
* [人形机器人双足步行仿真、激光 SLAM 与纯 NumPy 神经网络导航](./humanoid/humanoid_walk_nn.md)
* [人形机器人双臂搬运负载的神经网络平衡控制](./humanoid/humanoid_carry_payload_nn.md)
* [双足步态生成与 ZMP 稳定性](./humanoid/bipedal_gait_zmp.md)
* [基于三次样条插值的双足多步态平滑切换](./humanoid/biped_cubic_spline_gait_switch.md)
* [人形机器人键盘运动控制](./humanoid/humanoid_keyboard_control.md)
* [关节轨迹重定时：约束饱和下的最快时间缩放](./humanoid/joint_traj_retime.md) —— 自然三次样条 + 速度/加速度约束下的最大常数缩放，14 项测试
* [人形机器人平衡控制与外力抗扰恢复](./humanoid/humanoid_push_recovery.md)
* [人形机器人 PPO 神经网络平衡控制](./humanoid/humanoid_ppo_locomotion.md)
* [人形机器人 2D 激光雷达 CNN 感知与避障](./humanoid/humanoid_lidar_nn_perception.md)
* [G1 图像速度巡检](./humanoid/g1_rgb_velocity_patrol/index.md)
* [人形足迹下的局部避障算法对比](./humanoid/humanoid_local_planner_bench.md)
* [MuJoCo单足弹跳机器人简单仿真](./humanoid/mujoco_hopper_demo/index.md)
* [人形机器人关节轨迹录制与回放工具](./humanoid/humanoid_joint_tools.md)
* [人形上下台阶步态仿真](./humanoid/humanoid_stair_climb.md)
* [G1 gmapping 室内建图导航](./humanoid/g1_gmapping_navigation/index.md)
* [ROS虚拟机器人圆形轨迹rviz可视化示例](./humanoid/ros_topic_demo/index.md)
* [人形机器人全身逆运动学与可操作性](./humanoid/whole_body_ik.md)
* [G1 CostNet 栅格规划对照](./humanoid/g1_costnet_grid_planner/index.md)
* [G1 RGB 地面分割巡检](./humanoid/g1_rgb_floor_patrol/index.md)
* [人形机器人双臂末端可达域分析](./humanoid/humanoid_reachability.md)
* [人形机器人跌倒预判与保护性倒地](./humanoid/humanoid_fall_protection.md)
* [人形机器人 3D 碰撞体边界检测可视化](./humanoid/humanoid_collision_boundary.md)
*  [简化双足步态动画演示](./humanoid/zmp_walk_planner.md)
* [人形机器人轻量级 ZMP 步行控制基线](./humanoid/humanoid_zmp_walking.md)
* [人形机器人深度图姿态估计与平衡控制](./humanoid/humanoid_depth_pose_control.md)
* [关节指令的 QoS 与时延影响评估](./humanoid/joint_qos_latency_eval.md)

* [人形机器人支撑多边形与倾倒裕度核算](./humanoid/humanoid_support_margin.md) —— 双足凸包、稳定裕度与临界推力/冲量；默认 40 kg 样机裕度 125 mm，61 N 推力或 17.5 N·s 冲量即可推倒
* [人形机器人 IMU 姿态解算：互补滤波与 Madgwick 对比](./humanoid/humanoid_imu_attitude.md)
* [人形机器人离线语音指令控制](./humanoid/humanoid_voice_command.md)
* [人形机器人神经网络行走控制：CPG 基线与学习路线可行性分析](./humanoid/humanoid_ars_phase_walk.md)
* [人形机器人跑步牛顿第二定律理论计算与演示](./humanoid/humanoid_newton_running.md)
* [人形机器人全身柔顺控制](./humanoid/humanoid_g1_compliance.md)
* [人形机器人 MuJoCo 建图导航与神经网络控制](./humanoid/humanoid_mujoco_slam_nn_nav.md)
* [倒立摆（Cart-Pole）的 LQR 最优平衡控制](./humanoid/cartpole_lqr.md) —— 手写线性代数解 Riccati 方程得 K=[-0.954, -2.186, -35.768, -5.839]，终态摆角 2.8e-4 rad，10 项单测

## 3. 肌肉骨骼人 <span id='musculoskeletal'></span>

* [肌肉骨骼人基础](./musculoskeletal/musculoskeletal_basics.md)
* [肌肉骨骼人肌骨手臂仿真、工作空间建图与纯 NumPy 神经网络肌肉控制](./musculoskeletal/msk_arm_nn.md)
* [基于关节状态与肌腱力反馈的 MuJoCo 肌肉骨骼肘关节神经网络轨迹控制](./musculoskeletal/mujoco_musculoskeletal_elbow_nn_control.md)
* [肌肉-肌腱串联弹性实验](./musculoskeletal/msk_tendon_basics.md) —— 单关节 + 拮抗肌，肌腱储能/传力/释放三组实验
* [肌肉协同降维与恢复质量评估](./musculoskeletal/muscle_synergy_lab.md) —— NMF 求协同数，用 VAF 与子空间夹角区分“重建得像”与“协同是真的”，13 项测试
* [MuJoCo肌肉骨骼人键盘遥操作行走踢腿蹲起](./musculoskeletal/msk_human_walking_teleop.md) —— 22 条 Hill 肌肉驱动 13 DoF 全身模型，平地/坡道/负载三场景
* [MuJoCo 肌肉骨骼灵巧手：键盘遥操作与神经网络肌肉激活控制](./musculoskeletal/msk_hand_teleop.md)
* [MuJoCo 双关节肌肉骨骼手臂到达控制](./musculoskeletal/msk_arm_reach.md)
* [MuJoCo 肌肉骨骼手臂本体感觉与串联弹性肌腱](./musculoskeletal/msk_arm_proprioception.md)
* [肌肉驱动的关节控制与 Hill 肌肉模型](./musculoskeletal/muscle_joint_control.md)
* [肌肉疲劳建模：持续收缩下的肌力衰减与恢复](./musculoskeletal/muscle_fatigue.md) —— 在 Hill 模型上叠加疲劳因子，对比持续与间歇收缩的力衰减
* [肌肉驱动的夹爪抓取：真实接触抓取](./musculoskeletal/msk_gripper.md)
* [肌肉驱动的下肢关节与步态](./musculoskeletal/muscle_leg_gait.md)
* [膝关节肌肉骨骼仿真与轨迹跟踪](./musculoskeletal/knee_tracking.md)
* [机械臂手眼标定仿真](./humanoid/hand_eye_calibration.md)
* [人形上下台阶步态仿真：ZMP 预瞄控制与台阶参数扫描](./humanoid/humanoid_stair_zmp_preview.md)
* [MuJoCo单足弹跳机器人简单仿真：SLIP 模型与 Raibert 弹跳控制](./humanoid/mujoco_hopper_slip_control.md)
* [人形足迹下的局部避障算法对比：MPPI 与斥力法的窄通道宽度扫描](./humanoid/humanoid_passage_width_sweep.md)
* [给定黑箱关节轨迹，反推肌肉参数](./musculoskeletal/msk_muscle_ident.md)
* [肌肉冗余系统的力分配：最小范数与加权最小二乘](./musculoskeletal/muscle_force_distribution.md) —— 冗余拮抗肌的最小范数分配，力矩跟踪误差 < 1e-3 Nm，附激活度与跟踪曲线

## 4. 地面载具  <span id='ground_vehicle'></span>

* [Dubins 曲线轨迹生成与可视化](./ground/dubins_path.md) —— 6 种基本组合（LSL/LSR/RSL/RSR/RLR/LRL）枚举最短带运动学约束路径，纯 pygame 2D 可视化
* [RRT 快速随机扩展树路径规划](./ground/rrt_path_planning.md) —— 矩形障碍空间下的 RRT 采样规划：随机树扩展、目标偏置、Liang-Barsky 碰撞检测与路径回溯，pygame 2D 可视化
* [Bezier 曲线轨迹生成与可视化](./ground/bezier_trajectory.md) —— de Casteljau 算法与 Bernstein 多项式生成 Bezier 曲线，附 B 样条对比、曲率分析与弧长计算，纯 pygame 2D 可视化
* [人工势场法路径规划](./ground/apf_path_planning.md) —— 目标引力 + 障碍物斥力 + 局部最小值逃逸，分段引力势避免远场发散，pygame 势场热图与路径生长动画
* [小海龟多航点巡航与暂停恢复](./chapter/chap1/turtle_waypoint_patrol/index.md)
* [离线测试接入 GitHub Actions](./ground/offline_checks.md) —— 6 个零依赖离线测试接入 CI，分别通过 208 / 270 / 242 / 41 / 271 / 204 项
* [编码器量化误差与速度估计滤波对比](./ground/encoder_quantization_velocity.md) —— 1 kHz 差分把量化误差放大千倍；1024/4096/16384 CPR 下原始差分 1.371→0.404→0.133 rad/s，α-β 与卡尔曼把误差压到 0.198/0.047
* [车辆 CAN 报文信号解码与一致性校验](./ground/can_signal_decode.md) —— 位域 + CRC8 + 计数器三重校验，不校验时车速会被污染成 403 km/h

* [毫米波雷达目标跟踪与前碰撞预警（TTC/FCW）](./ground/radar_ttc_fcw.md) —— 用多普勒速度分离静止杂波，多帧关联估计 TTC 并输出 SAFE/WARN/STOP 分级预警
 - [基于 1D CNN 的激光雷达 SLAM](./ground/carla_nn_scan_matching.md)

* [手动控制](./set_up_and_connect_to_carla.md)
* [CARLA 手动控制（Windows + Ubuntu 20.04 实测）](./control/carla_manual_control.md)
* [MNIST 手写数字视觉感知与机器人运动控制](./ground/mnist_digit_control.md) —— CNN 识别手写数字为运动指令，防抖确认后控制 turtlesim 平移/旋转
* [生成对象](./ground/carla_spawn_objects.md)
* [室内清扫车的沿墙清扫：激光雷达与神经网络贴墙跟随](./ground/wall_clean_nn.md) —— 12×10 m 带家具房间，圆角轮廓 + 纯跟踪专家，行为克隆 MLP；专家 20/20 达标（覆盖 95.1%、贴墙误差 0.086 m），神经网络覆盖 78.9%、达标 5%
* [地面载具物理仿真与键盘运动控制](./ground/carla_keyboard_control.md)
* [动态障碍物避让：激光雷达帧间差分与神经网络时机决策](./ground/dyn_avoid_nn.md) —— 两个往复扫动的障碍，专家用前向仿真决定过线时机，行为克隆 MLP 仅凭两帧雷达推断运动，20 回合成功率 90% / 75%
* [电机温度时序预测与误差评估](./ground/mujoco_motor_temperature_predictor.md)
* [CARLA+YOLOv8 目标检测感知与减速避障](./ground/carla_yolo_avoidance.md)
* [阿克曼控制](./ground/ackermann_control.md)
* [车道线检测（前视视觉）](./ground/carla_lane_keeping/lane_detection.md)
* [车道保持控制](./ground/carla_lane_keeping/lane_keeping_control.md)
* [传感器感知与给定轨迹跟踪（神经网络版）](./ground/carla_perception_control.md)
* [R2D2小车自动巡航仿真](./ground/r2d2_cruise.md)
* [语义分割车道线巡游与 PID 横向控制](./ground/carla_lane_following.md) —— 语义分割提取车道线 + 单侧估计，PID 闭环转向；20 s 跑满，平均归一化偏移 0.086
* [R2D2小车航点巡航仿真](./ground/r2d2_waypoint_cruise.md) —— P控制器航点跟踪，方形路径循环巡逻，附轨迹数据记录
* [路径点发布器](./ground/waypoint.md)
* [激光雷达建图与神经网络规划导航](./ground/carla_mapping_navigation.md)
* [MuJoCo 差速车建图与导航同步仿真（EKF-SLAM + 神经网络局部规划）](./ground/mujoco_slam_nav_sync.md) —— 栅格建图、粒子/协方差定位与 A* + 网络局部规划同步联调
* [激光雷达点云 3D 目标检测与召回率评估](./ground/carla_lidar_obstacle_detection.md) —— 语义点云按实例分组 + 多帧累积；逐帧平均召回率 0.43（precision 0.65）
* [自动驾驶代理](./ground/ad_agent.md)
* [端到端神经网络图像驾驶](./ground/carla_end_to_end_nn.md)
* [Carla PilotNet 端到端控制（离线仿真演示）](./ground/carla_pilotnet_ros.md)
* [自动驾驶示例](./ground/ad_demo.md)
* [rostopic 手动驱动](./ground/rostopic_cruise.md)
* [作业综合整合与性能评价](./ground/carla_benchmark_suite.md)
* [ROS Scenario Runner](./ground/ros_scenario_runner.md)
* [交通信号灯识别与响应](./ground/carla_traffic_light.md)
* [RVIZ Carla 插件](./ground/rviz_plugin.md)
* [扭转控制](./ground/twist_to_control.md)
* [换道可行性判定](./ground/lane_change_feasibility.md)
* [倒车雷达分级告警](./ground/ultrasonic_rear_alert.md)
* [里程计轨迹记录](./ground/odom_trace_recorder.md)
* [转向执行器滞后](./ground/steering_actuator_lag.md)
* [刹车距离核算](./ground/braking_distance_check.md)
* [半主动悬架车身姿态控制](./ground/semi_active_suspension.md) —— 7 自由度整车垂向模型（车身垂向/俯仰/侧倾 + 四个非簧载质量），天棚 + 地棚混合阻尼把过减速带车身加速度峰值压低 9.3%、制动点头 16.3%、过弯侧倾 9.2%，随机路面最小轮荷提高 5.8%，并说明稳态姿态改不了、只能削过渡过程
* [换挡策略与闭环控制](./ground/gear_shift_policy.md)
* [牵引车挂车折刀判定](./ground/trailer_kinematics.md)
* [速度曲线规划](./ground/speed_profile_check.md)
* [转弯扫掠校验](./ground/turning_sweep_check.md)
* [爬坡能力与牵引力余量核算](./ground/vehicle_gradeability_check.md) —— 按整车质量、坡度、车速与附着系数核算所需/可用牵引力和余量，区分可爬坡、动力不足与打滑风险
* [涉水深度与浮力裕度核算](./ground/vehicle_wading_check.md) —— 按进气口、排气口与车门下沿取限深，核算浮力占比、湿态牵引力与允许涉水车速，区分 SAFE/CAUTION/FLOODED/FLOATING
* [半挂车折叠风险与倒车限速保护](./ground/trailer_jackknife_guard.md)
* [车辆几何通过性核算](./ground/passability_check.md) —— 接近角/离去角/纵向通过角 + 沿剖面逐位姿判断车头/车底/车尾会不会蹭、蹭多深
* [RQT 插件](./ground/rqt_plugin.md)
* [激光雷达点云语义分割](./ground/carla_lidar_pointcloud_segmentation.md)
* [点云地图创建](./ground/pcl_recorder.md)
* [定点停车](./ground/fixed_point_stop.md)
* [差速小车沿墙行驶](./ground/diff_drive_wall_following.md) —— 从 LaserScan 取五路距离，用距离误差和车身夹角误差控制 /cmd_vel，墙角自动原地左转，附离线仿真与跟墙误差指标
* [ROS与Conda环境排错指南](./ground/ros_conda_env_guide.md)
* [打靶](./ground/mujoco_shooting.md)
* [Carla 交通标志识别（Windows 脚本）](./ground/carla_traffic_sign_recognition.md) - 基于 YOLOv8 的实时感知与键盘控制

* [YOLOv8 视觉感知目标跟随仿真](./ground/yolov8_target_tracking.md)
* [车辆键盘遥操作与二维模拟](./ground/vehicle_pygame_teleop.md)
* [基于 TTC 的前向目标跟踪自动紧急制动](./ground/carla_ttc_emergency_braking.md) —— 前向目标跟踪求 TTC，超阈值满制动；激光雷达决策，实测停在前车 7.29 m
* [地面车 PID 巡航速度跟踪仿真](./ground/ugv_cruise_pid.md)
* [差速小车电机温度与电流观测记录](./ground/mujoco_motor_sensor_recorder.md)
* [ROS2 差速机器人卡死检测与自动脱困](./ground/ros2_stuck_recovery/index.md) —— 用指令/里程计/IMU/测距的一致性区分顶住、打滑、单轮悬空，分级执行倒车-转向-试探脱困，3 级失败停车上报；7 个场景 × 10 种子可离线复现

* [任务1：机器人键盘控制模块](./robot_tasks/teleop_control/README.md)
* [任务2: 模拟传感器数据源](./robot_tasks/sensor_perception/README.md)
* [任务3: 导航与路径规划模块](./robot_tasks/navigation_slam/README.md)
* [任务4：端到端策略网络](./robot_tasks/policy_learning/README.md)
* [任务5：系统集成](./robot_tasks/system_integration/README.md)

* [TurtleBot3 激光雷达自主避障（ROS 2）](./ground/turtlebot3_lidar_avoid.md) —— 订阅机载激光雷达，按扇区提取前/左/右最近距离，经三态状态机与势场法决策后发布 /cmd_vel，实现迷宫场景自主避障
* [ROS2 动态障碍跟踪与 TTC 预警](./ground/ros2_dynamic_obstacle_ttc.md)
* [Mbot 雷达避障仿真（ROS 1 Noetic）](./ground/mbot_lidar_avoid.md) —— Mbot 差速底盘在 Gazebo 中的 2D 激光雷达阈值避障，按扇区最小距离 + 滞回状态机输出 /cmd_vel

* [网页远程遥控小车](./ground/web_remote_car.md)
* [MuJoCo AEB自动紧急制动仿真](./ground/mujoco_vehicle_aeb.md)
* [载重偏置下的制动距离与跑偏](./ground/brake_load_transfer.md) —— 轴荷转移、抱死顺序与制动跑偏，18 项单元测试 + 两张实测图
* [端到端驾驶（图像到控制指令）](./ground/pix2cmd_end2end_driving.md)

* [MuJoCo 差速小车轮滑感知与轨迹补偿](./ground/mujoco_diff_drive_slip_compensation.md)
* [双惯量传动扭振的状态反馈极点配置](./ground/drivetrain_torsion_state_feedback.md) —— 用状态反馈把扭振阻尼比 0.134 提到 0.70，不用改机械结构

* [MuJoCo 差速小车到达目标点（go-to-goal）](./ground/mujoco_diff_drive_go_to_goal.md)
* [MuJoCo 差速小车射线绕行避障](./ground/mujoco_obstacle_avoidance.md)
* [MuJoCo 差速小车走迷宫](./ground/diff_drive_maze.md)
* [MuJoCo 地面小车射线感知与分级减速避障](./ground/mujoco_car_avoidance.md)
* [MuJoCo 地面小车神经网络端到端避障](./ground/mujoco_car_nn_avoidance.md)
* [MuJoCo 两轮自平衡小车](./ground/mujoco_balance_car.md)
* [MuJoCo 差速小车电机温升模型](./ground/mujoco_motor_thermal_model.md)
* [MuJoCo 地面差速小车侧倾/翻车风险预测与主动限速](./ground/mujoco_car_rollover_guard.md)
* [ROS 2 电机温度保护节点](./ground/mujoco_motor_thermal_guard.md)
* [MuJoCo 地面差速小车单侧驱动失效检测与差速补偿](./ground/mujoco_car_drive_fault.md)
* [MuJoCo 四轮独立悬架越野小车仿真](./ground/mujoco_offroad_suspension.md)
* [2D 小车目标追踪仿真](./ground/car_2d_target_tracker.md)
* [多车编队保持仿真（pygame 2D）](./ground/formation_keeping.md) —— 领航者沿圆周运动，两个跟随者按比例控制保持三角队形，统计领航者与跟随者共 3 个成员的队形均方根误差
* [电机热场景对比评价](./ground/motor_thermal_scenario_evaluator.md)
* [基于神经网络的无人小车自适应跟车（ACC）](./ground/carla_nn_car_following.md)
* [Gazebo 差速小车键盘控制](./ground/gazebo_diff_drive_teleop.md)
* [恶劣雨雾天气下多传感器感知性能对比实验](./ground/weather_perception_benchmark.md)
* [雨雾天气下自适应感知降级策略的静态帧复现](./ground/weather_adaptive_perception_replay.md)
* [巡航风阻与能耗敏感性分析](./ground/cruise_drag_energy_eval.md) —— 滚动 / 空气 / 坡度阻力分解与车速、风阻、坡度对能耗的影响
* [无信号路口冲突时序蒙特卡洛仿真](./ground/intersection_conflict_timing_eval.md) —— 到达时间差分布、冲突概率与让行延迟效果对比
* [轮胎滑移率-附着特性与最佳制动滑移率仿真](./ground/tire_slip_adhesion_eval.md) —— 四种路面魔术公式附着曲线、峰值附着与制动距离对比
* [神经网络强化学习算法与网络结构对比（MuJoCo CartPole）](./ground/rl_algorithm_compare.md)
* [车辆稳态操稳特性对比（不足 / 中性 / 过度转向）](./ground/steady_state_handling_eval.md) —— 质心位置对转向梯度、横摆增益与侧偏角的影响
* [基于Mujoco的键盘遥控小车](./ground/mujoco_car_project.md)
* [MuJoCo 叉车同步建图与自主导航](./ground/forklift_slam_nav.md)
* [MuJoCo 叉车相机引导的托盘接近与低速插入](./ground/mujoco_forklift_pallet_docking.md)
* [地面载具横向控制算法对比（纯跟踪 / Stanley / LQR）](./ground/lateral_control_benchmark.md) —— 同一车辆模型与指标下定量比较三种横向控制器
* [MPPI 采样 MPC 与蒸馏神经网络的 8 字赛道跟踪对比](./ground/mujoco_mppi_neural_tracker.md) —— 同一赛道上比较 MPPI、蒸馏网络与纯追踪的精度/算力/抗噪
* [ROS2 差速机器人 A* 全局规划与 DWA 局部避障](./ground/ros2_dwa_avoidance.md)
* [ROS2 DWA 局部规划器随机场景压力测试与舒适性评价](./ground/ros2_dwa_benchmark.md)
* [ROS2 A*+DWA 的 Gazebo 三维验证](./ground/ros2_dwa_gazebo.md)
* [UWB 多基站测距定位布站几何精度对比评测](./ground/uwb_baseline_geometry_eval.md) —— 四种布站构型的 GDOP 与蒙特卡洛定位精度对比
* [TurtleBot3 仿真自主导航与避障参数优化](./ground/turtlebot3_auto_nav/README.md)
* [TurtleBot3 键盘遥控](./ground/turtlebot3_keyboard_teleop.md) —— w/s/a/d 差速底盘遥控，松开按键 0.5 秒自动归零悬停
* [ROS2 TurtleBot3 激光雷达避障实验](./ground/ros2_turtlebot3_obstacle.md) —— 课程作业二独立归档：/scan 前方扇区测距、量程过滤、左右择优转向与断流看门狗（最小闭环）
* [地面无人车视觉 SLAM 建图与自主导航](./ground/ugv_visual_slam_nav.md) —— 单目 RGB-D 做 EKF-SLAM 与占据栅格，A\* 规划 + 纯追踪执行，并排训练端到端 CNN 做对照
* [DVS 事件相机背景活动抑制与运动目标跟踪](./ground/dvs_event_motion_tracking.md) —— 直接解析 CARLA 13 字节事件流，用 BAF 学习静态树冠噪声、残差连通域分割运动车辆，实测抑制 79.7% 背景事件、检出率 100%、质心 MAE 2.35 px，事件量仅为等价整帧像素的 3.21%
* [MuJoCo Locomotion 强化学习对比实验](./ground/rl_mujoco_locomotion.md)
* [图像录制 ROS 2 封装模块](./ground/image_recorder.md)
* [MuJoCo小车倒车入库仿真](./ground/mujoco_car_parking.md)
* [MuJoCo 侧方停车仿真](./ground/mujoco_parallel_parking.md)
* [MuJoCo 动态障碍局部避障仿真](./ground/mujoco_dynamic_avoidance.md)
* [CARLA 停车场场景生成（附带不依赖 CARLA 的倒车入库规划器）](./ground/carla_parking_lot.md) —— 在 CARLA 里画 6 个车位，另提供一个纯 Python 的倒车入库轨迹规划器，4 条断言可直接跑回归
* [reverse_parking_planner 离线回归测试接入 pytest](./ground/reverse_parking_planner_pytest.md) —— 把 197 项顶层检查改为 pytest 可收集入口，修复 `SystemExit: 0`
* [MuJoCo 红绿灯启停小车仿真](./ground/mujoco_traffic_light_stop.md)
* [MuJoCo 小车跟车仿真](./ground/mujoco_car_follow.md)
* [MuJoCo 小车滑斜坡](./ground/mujoco_slope_car.md)
* [悬崖边缘紧急制动仿真](./ground/cliff_brake.md) —— 用 MuJoCo 物理引擎与 ROS 三节点闭环实现车头测距触发的滞回紧急制动，附安全包络定量测试与坠落对照
* [线性 MPC 路径跟踪与纯追踪对比](./ground/mpc_path_tracking.md) —— 线性时变 MPC + 箱约束 QP（坐标下降）；40 s 闭环横向 RMSE 0.212 m、稳态 0.008 m，最大偏航角速度 0.904 vs 纯追踪的 1.200 rad/s，代价 27.2 ms/step
* [航向 PID 参数自整定与手工试凑对比](./ground/pid_auto_tuning.md) —— 用继电器反馈法自动求 Ku/Tu 并筛选 PID 参数，与手工试凑同工况对比超调与调节时间
* [磁力计硬磁/软磁标定与倾角补偿航向](./ground/mag_hard_soft_iron_calibration.md) —— 椭球拟合求硬磁偏置（误差 0.90 µT）与软磁矩阵（1.1%），对称平方根消除旋转自由度；航向 RMSE 42.65° → 1.26° → 融合 0.33°
* [连续控制器离散化的四种方法对比](./ground/control_discretization.md) —— Tustin 比前向欧拉准 1~2 个数量级，前向欧拉在 Ts>0.27 s 时失稳

* [基于 Autoencoder 的雷达数据修复与避障](./ground/lidar_autoencoder_repair.md) —— 离线训练自编码器去除传感器噪声，修复后数据送入阈值避障控制链路，展示传感器数据容错层与控制层解耦的设计
* [地面载具灯光自动控制](./ground/carla_auto_light.md)
* [限速标志识别与神经网络限速控制](./ground/traffic_sign_speed_control.md) —— MuJoCo 仿真中车载相机采集限速标志，CNN 字符识别限速数值，闭环限速控制差速小车，附退化矩阵与多场景性能评价
* [轨迹/里程计精度评估（ATE/RPE）](./ground/trajectory_accuracy_eval.md)
* [RTK 双差整周模糊度固定（LAMBDA）](./ground/rtk_ambiguity_resolution.md) —— 双差浮点解 + LAMBDA 去相关 + ratio 检验；20 历元窗口基线误差 160.51 mm → 0.89 mm，错误固定 0%，附 2/5/10/20/40 历元的收敛对照
* [基于 2D 激光雷达的蒙特卡洛定位（MCL）](./ground/lidar_mcl_localization.md) —— Gazebo 差速车 + 360 束激光，用纯 NumPy 手写粒子滤波（运动模型采样、似然场观测模型、低方差重采样），并用 MLP 学掉里程计运动残差；四组对比中位置误差从 0.345 m 降到 0.020 m

* [地面载具动力性测试](./ground/carla_vehicle_dynamics_test.md)
* [MuJoCo小车多目标导航](./ground/mujoco_multi_goal_nav.md)
* [列车 ATO 自动运行与站台精确停车](./ground/train_ato_stop.md) —— 按线路限速与坡道生成目标速度曲线，站台停车误差 0.02～0.18 m（标尺 0.3 m），另有 ATP 超速监督与晚制动对照
* [多编组列车纵向动力学与车钩力](./ground/train_consist_coupler.md) —— N 节车 + N-1 个弹簧-阻尼-间隙车钩逐节积分，指令按车传播；理想同步时车钩力严格为 0，头尾两种方向给出镜像的压钩/拉钩，8 节 100 ms 延迟到 125 kN，停车误差仍在 0.3 m 内
* [ROS2 A* 全局路径平滑（Catmull-Rom）](./ground/ros2_path_smoothing.md)
* [多车协同编队（CARLA）](./ground/carla_platoon.md) —— Leader 沿参考轨迹领航，3 辆 follower 沿同轨迹保持恒定弧长间距跟进的一字纵队编队
* [ROS2 2D 激光占据栅格建图](./ground/ros2_occupancy_mapping.md)
* [神经网络感知 / 控制 / SLAM 四任务仿真（CARLA）](./ground/carla_nn_perception_control_slam.md) —— 键盘运动控制、YOLOv5 感知与给定轨迹纯跟踪、激光 log-odds 栅格建图与 A* 导航、PilotNet 端到端驾驶（图像 → 控制量）
* [多传感器融合跟踪（LiDAR+Radar+Camera+IMU）2D 合成仿真与离线验证](./ground/carla_sensor_fusion.md) —— 外参标定 + 时间同步 + EKF 融合；2D 仿真提升约 32%，离线回放 30 s 融合 578 次、RMSE 0.053 m（均可复现；CARLA 为未验证的参考实现）
* [高精度定位与重定位（AMCL 式粒子滤波）](./ground/turtlebot3_relocalization.md)

* [两轮自平衡机器人（倾角PD + 神经网络策略克隆）](./ground/balance_bot.md) —— MuJoCo 两轮倒立摆，倾角 PD 站稳（20 s 最大倾角 0.96°），被推 2 N·s 后 3.22 s 回正，并用 MLP 克隆控制律
* [车辆 ESC 电子稳定控制](./ground/vehicle_esc_stability.md) —— 双轨车辆 + 魔术公式轮胎，横摆角速度跟踪与单侧差动制动；双移线侧偏角 89.2° → 3.8°、偏移 17.02 m → 0.59 m，附对开路面 select-low 的制动距离代价
* [基于里程计反馈与 A* 算法的差速底盘静态全局路径规划](./ground/diff_drive_astar_odom_nav.md) —— Gazebo 差速底盘在静态栅格上用 A* 规划全局路径，仅凭 `/odom` 里程计做 30 Hz 纯追踪闭环，并用真值量化跟踪误差与里程计漂移
* [视觉-惯性时间偏移互相关估计](./ground/vio_time_offset_estimation.md) —— 陀螺与视觉角速率归一化互相关 + 抛物线插值；200 次试验偏移误差 RMSE 0.89 ms，补偿后残差 0.1911 → 0.0907（改善 52.5%）
* [ROS2 差速机器人 A* 全局规划与 DWA 局部避障](ground/ros2_dwa_avoidance.md) —— A* 全局路线 + DWA 局部跟踪，5 场景 3 控制器对比，窄走廊与 U 形陷阱通过
* [差速机器人里程计标定与神经网络滑移补偿（ROS 1 + MuJoCo）](./ground/diff_drive_odom_calib.md) —— MuJoCo 物理仿真下的 catkin 包；两个解耦实验辨识路程/轮距系数，再用 MLP 学转向滑移的非线性补偿，方形路径终点误差 2.91 m → 1.94 m → 0.98 m

* [差速底盘驱动系统选型核算](./ground/diff_drive_powertrain_sizing.md) —— 滚动/坡度/加速/风阻合阻力折算轮边转矩、轮速与功率；默认 12 kg 底盘直接驱动 1.536 N*m 超持续转矩，按 16.76 减速比配箱后降到 0.092 N*m
* [IMU 噪声 Allan 方差辨识与倾角滤波整定](./ground/imu_allan_noise_identification.md) —— 1 h 静止陀螺记录辨识 ARW（中位数 +0.46%）与零偏不稳定性（+3.77%，95% 区间 [−16.7, +44.4]），ARW→角度噪声、BI→偏置游走；倾角卡尔曼 NEES 1.61，过程噪声偏小 4 倍时 10.07
* [基于 IMU 的路面异常检测（坑洼 / 减速带 / 粗糙路面）](./ground/imu_road_anomaly_detection.md) —— 三轴重力投影（静态姿态）、3σ 滑窗能量检测与规则分类，四分之一车仿真 + CARLA 实采两套带真值数据，事件挂接 GNSS 经纬度
* [无动力被动行走（轮辐式 Rimless Wheel）](./ground/passive_walker.md)
* [相机-激光雷达外参标定](./ground/lidar_camera_calibration.md) —— 合成多姿态棋盘格场景，PnP 求 LiDAR→相机外参并与真值对比，附噪声与摆位次数敏感性实验
* [相机-IMU 时间同步标定](./ground/camera_imu_time_calibration.md) —— 互相关估计相机与 IMU 的常量时间偏移；±200 ms 内最大误差 0.16 ms，50 ms 偏移下姿态 RMS 误差 3.36° → 0.72°，并给出时间戳取帧首时的半帧固定偏差

* [前视相机车道偏离预警（视觉横向偏移与地图真值对照）](./ground/carla_visual_lane_departure_warning.md) —— 用前视图像估计横向偏移，并与地图真值逐帧对照
* [差速 AGV 坡道起步辅助（防溜车）](./ground/hill_start_assist.md) —— MuJoCo 12° 斜坡，四轮轮角保持 PD + 坡度前馈，松刹车 3 s 后溜 2.18 m 降到 0.008 m
* [地面移动底盘速度指令安全监督](./ground/cmd_vel_guard.md) —— 对 `/cmd_vel` 做限幅、急停、超时与非法值拦截，无需仿真环境
* [差速小车双侧墙通道居中 PD 巡航仿真](./ground/diff_drive_corridor_centering.md) —— 纯 Python PD 控制器只用左右墙与前方三路测距把差速底盘保持在直通道中线，配套 2D 运动学虚拟测距节点，离线仿真给出轨迹、墙距与速度指令曲线
* [巡线小车仿真（色带中线 PD 跟踪）](./ground/line_follower_car.md) —— 一排虚拟光电传感器读取线偏差，pure pursuit 预瞄 + PD 转向闭环引导沿黑线巡线
* [窄道会车让行仿真（避让龛规则）](./ground/narrow_giveway_passing.md) —— 双向共用窄路两侧设避让龛，对向小车按距离远近决定进龛等待，交汇后驶出，纯 NumPy 二维仿真
* [VRU 冲突风险图](./ground/vru_conflict_risk_map.md) —— 按时间对齐自车与行人、儿童和骑行者轨迹，输出最小距离、TTC 和风险等级
* [里程计健康监视（打滑 / 堵转 / 反向检测）](./ground/odom_health_monitor.md) —— 比较 `/cmd_vel` 与 `/odom` 的速度比值，识别打滑、堵转与反向运动
* [轮速传感器 + 龙伯格观测器估计传动轴扭角](./ground/wheel_speed_observer.md) —— 只测轮速即可反推传动轴扭角，给出带宽-噪声的量化权衡

* [轮胎滑移率-附着曲线](./ground/tire_slip_curve.md) —— 用简化刷子轮胎模型输出滑移率与附着系数关系，标出峰值附着与 ABS 目标滑移带
* [横向载荷转移与侧翻裕度](./ground/rollover_margin.md) —— 用 SSF、临界侧向加速度和内外侧轮垂向力算「这车会不会翻」，纯代数、不依赖 CARLA
* [工况能耗与续航核算](./ground/drive_cycle_energy.md) —— 把一段行驶工况拆成滚阻、风阻、制动与动能项，算清楚电花在哪、还能跑多远
* [乘坐舒适性传递率](./ground/ride_comfort_transmissibility.md) —— 用二自由度悬架加 ISO 2631 频率加权算路面振动传到人身上剩多少，解释「40 难受、60 反而稳」
* [不足转向梯度与侧向稳定裕度](./ground/lateral_stability_margin.md) —— 用不足转向梯度与临界车速算「快点打方向会不会失控」，纯代数、可离线断言

* [狭窄通道通行：激光雷达开口辨识与神经网络选口穿行](./ground/gap_nav_nn.md) —— 隔墙三个开口（0.7/1.4/2.4 m），车宽 0.6 m，专家按可通行性与绕路代价选口，行为克隆 MLP 仅凭雷达判断孔径，20 回合成功率 100% / 90%
* [四轮独立驱动扭矩矢量横摆控制](./ground/torque_vectoring_yaw_control/index.md) —— 附着椭圆约束下的加权最小二乘分配 + 带前馈横摆率跟踪 + 单电机失效降级
* [ROS2 差速机器人 EKF 多传感器融合定位](./ground/ros2_ekf_localization/index.md) —— 轮式里程计 + IMU 航向 + 绝对位置观测的 EKF，输出 /odometry/filtered 供控制器直接使用
* [上游心跳看门狗](./ground/heartbeat_watchdog.md) —— 监视上游节点的 `/heartbeat`，超时判定失联并发布急停，与 `cmd_vel_guard` 互补
* [MuJoCo 地面无人车 SLAM 建图导航与神经网络控制](./ground/mujoco_ground_slam_nn_nav/README.md)
* [地面坡度估计](./ground/ground_slope_estimation.md)
* [急停闩锁与人工复位](./ground/estop_latch.md) —— 把 `/emergency_stop` 锁住，必须调用 `/estop_latch/reset` 人工确认才放行，默认上电即闩锁
* [智能驾驶员模型（IDM）跟车与车队减速波仿真](./ground/idm_car_following.md) —— 五车纵列跟驰，最低速/下潜/时刻与解析一致，9 项单测


## 5. 空域载具 <span id='air_vehicle'></span>

* [AirSim 无人机激光雷达感知与扫描校正](./air/uav_nn_perception_navigation/perception.md) —— 修正 ROS 扫描左右角度、末端采样角与多帧姿态补偿
* [无人机航向坐标变换与往返误差验证](./air/uav_yaw_frame_validation/index.md)
* [无人机测距序列轻量 MLP 去噪](./air/uav_range_denoising_mlp/index.md)
* [仓储通道巡检无人机测距与分级制动](./air/mujoco_warehouse_uav_collision_braking.md)

* [多无人机空域监测（多边形禁飞区与机间冲突）](./air/multi_uav_airspace_monitor.md) —— 面向多机轨迹的电子围栏、最近接近距离和冲突解脱建议

* [多旋翼单桨失效后的悬停可行域](./air/uav_rotor_failure_env.md) —— 零姿态力矩约束下的最大总推力，对比四/六/八旋翼是否必须自旋，12 项测试

* [无人机动态电子围栏预测预警](./air/uav_dynamic_geofence_guard.md) —— 针对移动临时禁飞区计算预测净空、最近接近时间和规避方向，并附 ROS 2 Humble 实跑证据

* [winter 场景无人机圆周飞行](./air/winter_circle.md)
* [无人机航向角 PD 稳定控制与阶跃转向仿真](./air/uav_yaw_stabilize.md) —— 阵风力矩与往复风扰下的航向保持精度、阵风恢复时间，附实现动图
* [AirSim ROS 1 Noetic PPO 联调](./air/airsim_rl_planner_ros1.md)
* [无人机制动旧入口](./ground/mujoco_drone_distance_stop.md)
* [城市道路地面载具：二维自行车动力学、激光 SLAM 与纯 NumPy 神经网络导航](./ground/road_numpy_nn_slam.md)


* [无人机空间围栏与速度门控](./air/uav_box_geofence_guard/index.md)

* [Town10 立面自主巡航与图像采集](./air/town10_facade_cruise_capture/index.md)

* [AirSim 无人机弓字形区域勘测](./air/drone_lawnmower_survey.md) —— 按区域尺寸、航线间距、高度和速度生成往返覆盖航线，支持无仿真器安全预览与参数边界测试
* [无人机输电线路双侧巡检航线预览](./air/uav_powerline_inspection.md) —— 根据杆塔坐标生成左右观察点、相机朝向与航程估算
* [输电线路无人机巡检任务预检](./air/powerline_mission_preflight.md) —— 在任务执行前检查航点结构、高度、航段距离和杆塔双侧覆盖

* [Town10 幕墙图像病害识别](./air/town10_facade_defect_detection/index.md)

* [AirSim Unity 无人机曲线绕柱避障实验](./air/airsim_obstacle_avoidance.md)
* [AirSim Unity 无人机高度自适应越障实验](./air/airsim_height_aware_overfly.md) —— 深度图逐列恢复障碍顶边，矮墙原地爬升越过、高障碍退回水平绕行；57 项离线测试 + 2026-11-09 两次越障实飞（越障后保持 5.2 m、到点误差 <0.7 m；相机偏移修正后 committed 约 2.6 m，尚未复飞）
* [无人机神经网络感知、导航与端到端控制](./air/uav_nn_perception_navigation/index.md) —— 激光雷达 12 扇区特征 + MLP 策略，到达误差 0.76 m；gmapping 建图与导航同步；CNN 端到端视觉控制，自主飞行 10.89 m
* [AirSim Unity 无人机双柱连续 S 形绕行实验](./air/airsim_double_pillar_avoidance.md) —— 在前视深度反应式绕柱基础上增加绕行侧解锁与冷却状态机，使沿程后续柱子按自身位置重新选边，形成 S 形路径（已实飞到达目标、第二柱反向绕行验证；降落确认待复验）
* [无人机深度相机神经网络避障飞行（MuJoCo）](./air/uav_mujoco_depth_avoidance.md) —— 64×64 机载深度图 → numpy CNN → 三扇区通行度 → 速度/偏航闭环，3 场景 × 5 实例共 15/15 成功
* [MuJoCo 无人机栅格激光建图与路径重规划](./air/uav_grid_navigation.md)
* [无人机着陆阶段识别与接地载荷预测](./air/landing_assessment.md)
* [MuJoCo 无人机移动平台着陆](./air/mujoco_uav_moving_pad_landing.md)
* [高度与倾角检测评估](./air/air_ground_detection_benchmark.md) —— 统计不同相机高度和俯仰角下的检测精度、召回率与小目标召回率

* [Fixed-wing SITL preflight](./air/fixed_wing_sitl_preflight.md): static configuration checks for PX4/ArduPilot fixed-wing SITL
* [无人机测距制动与神经控制](./air/mujoco_uav_range_neural_braking.md)
* [无人机测距制动轨迹指标分析](./air/mujoco_uav_braking_metrics.md)
* [无人机测距几何一致性检查](./air/mujoco_uav_range_sensor_quality.md)

* [无人机载荷质量估计](./air/payload_mass_estimation.md)
* [无人机载荷重心偏移与配平监测](./air/uav_payload_balance_guard.md) —— 利用左右推力差、滚转修正和横摆响应估计横向重心偏移，输出配平和安全包线告警
* [AirLib 传感器消息回归](./air/airlib_message_regression.md) —— 在 ROS 桥接前校验 IMU、GPS、LiDAR 和里程计数据契约
* [无人机 GPS 欺骗与定位完整性监测](./air/uav_gps_integrity_guard.md) —— 融合位置跳变、速度一致性和 IMU 差异，输出 GPS 可信、交叉检查或拒绝建议

* [UAV mission energy budget](./air/uav_energy_budget.md) ? estimates energy, reserve and endurance for hover, climb and cruise segments
* [电机推力不对称监测与紧急降落触发](./air/uav_motor_thrust_imbalance_monitor.md) —— 从 mixer 指令与 ESC 转速区分电机衰减（RPM 跟踪误差）与桨损（指令偏移），四态状态机 + 补偿指令 + 紧急降落触发；含 ROS 话题闭环冒烟测试与 MuJoCo 台架验证

* [无人机电芯健康与不平衡预警](./air/uav_battery_health_guard.md) —— 根据单体电压差、温度、内阻和负载压降识别弱电芯，输出限载或就近降落建议


* [无人机任务管理与安全状态机](./air/uav_mission_safety_manager/overview.md)

* [无人机飞行轨迹类型识别与 ROS2 自动任务日志](./air/flight_pattern_recognition.md)

* [前视 CNN 行为克隆视觉导航](./air/uav_neural_perception.md) —— 前视 RGB 图像 → CNN → 机体系速度指令，行为克隆直线跟随
* [深度图端到端穿越门竞速](./air/uav_depth_gate_racing_e2e.md)

* [无人机遥测异常检测与 ROS2 在线诊断](./air/telemetry_diagnostics.md)
* [无人机遥测缺失重建](./air/telemetry_imputation.md)

* [低空多源气象扰动下无人机影响预测与风险分级](./air/uav_weather_risk.md)

* [无人机火灾探测与自主巡检](./air/fire_detection_drone.md) —— HSV/ONNX 火焰检测配合有限状态机，支持弓字形、边界与螺旋三种巡检场景，发现火灾后自动悬停确认、盘旋取证并返航
* [配置 AirSim 客户端环境并连接模拟器（Windows 本机 / WSL2）](./air/airsim_client_setup.md)

* [建立虚拟机和空域载具之间的连接](./air/setup_and_connect.md)
* [无人机目标高度限幅、变化率约束与超时保持](./air/uav_altitude_command_guard.md)
* [无人机 GNSS 定位完整性监测与位置跳变失效保护](./air/uav_gnss_integrity_monitor.md) —— 定位输入侧的运动学合理性门控，位置突跳或失锁时冻结最后可信位姿并请求悬停

* [无人机偏航指令最短路径与角速度限幅](./air/uav_yaw_command_guard.md) —— 将偏航误差映射到 `[-pi, pi)`，避免跨 180° 时绕远路，并限制 `Twist.angular.z`

* [多旋翼重心偏移配平与饱和包线核算](./air/uav_cg_trim_envelope.md) —— 四电机悬停拉力分配、配平倾角与饱和包线；默认 1.2 kg 样机重心偏 3 mm 即俯仰 5.7°，6 N 电机上限对应 ±0.13 m 配平包线
* [无人机雷达DQN建图与端到端控制](./air/uav_lidar_dqn_navigation/index.md) —— 24 扇区雷达特征 + Dueling Double DQN 自主学习避障导航，gmapping 同步建图，图像端到端 DQN 控制
* [DQN 策略离线评估](./air/uav_lidar_dqn_navigation/policy_eval.md) —— 不依赖 ROS/AirSim，汇总训练日志并复现“正前方若干米有墙”的 Q 值与动作探测

* [无人机激光雷达三维安全走廊与自适应高度避障](./air/uav_lidar_safety_corridor.md) —— 基于 OpenHUTB 激光雷达点云选择安全高度，并完成世界系速度闭环飞行

* [无人机最小加加速度轨迹生成与跟踪](./air/uav_minimum_jerk_tracking.md) —— 生成分段五次轨迹（段内 jerk 连续），并与线性参考对比跟踪误差和加加速度峰值
* [无人机空速/地速三角估风与可观测性](./air/uav_wind_estimation.md) —— 风三角残差最小二乘 + 信息矩阵可观测性；悬停/直线不可估（条件数 2914-3600），圆周机动风速 RMSE 0.042 m/s（真值 3.6 m/s）

* [无人机相机云台两轴稳定控制](./air/uav_gimbal_stabilization.md) —— 二维目标视线 PID 跟踪、角速度限幅和死区控制，附仿真指标与 CSV
* [旋翼桨尖马赫数与桨盘载荷](./air/uav_prop_tip_mach.md) —— 由桨径、转速与温度算桨尖速度与马赫数（默认 10 寸桨 9000 rpm 时 0.35，安全），由整机质量与桨盘面积算桨盘载荷与悬停诱导速度（96.8 N/m²、6.28 m/s）

* [无人机 IMU 与气压高度融合](./air/uav_altitude_sensor_fusion.md) —— 对比气压高度、加速度积分和一维卡尔曼融合，附高度响应图与误差指标
* [无人机 L1 自适应高度控制仿真](./air/uav_l1_altitude_stabilization.md)

* [空域载具的八叉树三维寻路](./air/octree_uav_3d_pathfinding/env_setup.md)

* [点云转八叉树占用地图](./air/octree_uav_3d_pathfinding/occupancy_mapping.md)

* [八叉树上的 A* 全局寻路](./air/octree_uav_3d_pathfinding/astar_planning.md)

* [空域模拟器的 ROS 封装器](./air/ros_pkgs.md)
* [无人机与 ROS 的桥接模块](./air/carlair_ros_bridge.md)
* [无人机载荷重心包线检查](./air/uav_payload_cg_envelope.md) —— 结合空机与载荷质量检查纵向、横向重心余量。
* [无人机 SLAM 建图与导航](./air/uav_slam_nav.md) —— Humble 侧 slam_toolbox + nav2，跨 ros1_bridge 接 Noetic bridge
* [无人机深度相机建图与自主导航（深度反投影 + A* + 纯追踪）](./air/uav_depth_mapping_nav.md) —— 深度相机反投影建图、A* 规划与纯追踪控制的完整闭环（原 uav_slam_nav 更名，避免与已合并模块重名）
* [AirSim ROS 2 Humble 最小桥接](./air/airsim_ros2_bringup.md)
* [AirSim ROS 2 Humble GNSS 桥接](./air/airsim_ros2_gnss.md)
* [AirSim ROS 2 Humble 磁力计桥接](./air/airsim_ros2_magnetometer.md)
* [AirSim ROS 2 Humble 气压计桥接](./air/airsim_ros2_barometer.md)
* [无人机 LiDAR 地面/障碍分割](./air/uav_lidar_ground_filter.md)
* [无人机屋顶降落](./air/rooftop_landing.md) —— 用机载 LiDAR 检测屋顶平面，取轮廓的最大内切圆圆心作为降落点，自主飞抵其上方后垂直降落
* [无人机屋顶降落（学习型落点选择）](./air/rooftop_landing_nn.md) —— 用点云高度图 + 小 CNN 直接回归屋顶落点；凹形屋顶（U/L/天井）与稀疏点云下比凸包最大内切圆更稳，附三组消融对比
* [无人机降落过程监控（指令-运动一致性）](./air/uav_landing_monitor.md) —— 在线校验「速度指令 vs 实际高度变化」，抓“喊了不动 / 掉得没人管 / 超时没落地”这类静默故障，输出告警、状态 JSON 与 CSV 报告
* [移动平台（车/船）精准降落](./air/uav_moving_platform_landing.md) —— 只吃一路带延迟的平台位置，在线估速度、预测相遇点，落点误差 0.03 m（追实测位置的朴素做法 0.48 m 且落不准），附四种制导策略的定量对照
* [无人机键盘控制](./air/uav_keyboard_control.md)
* [无人机起飞—悬停—降落状态机](./air/uav_takeoff_hover_land.md) —— 订阅 `/uav/pose`、发布 `/uav/cmd_vel`，用状态机和比例控制完成起飞、到点悬停与降落，附离线仿真与指标
* [无人机 AirSim 键盘遥控（ROS 2 Humble）](./air/uav_airsim_keyboard.md) —— W/S/A/D/R/F/Q/E 键直控 AirSim 多旋翼，内置 AirSim 桥与看门狗，支持起飞/降落服务
* [无人机行为克隆模型对比基准（网络结构与数据增强）](./air/uav_bc_bench.md) —— 三种网络结构 × 有无数据增强的对照实验，按圈分组切分，含恒零基线

* [基于 PPO 的无人机导航避障](./air/uav_ppo_nav.md)
* [uav_ppo_nav 本地测试（pytest）](./air/uav_ppo_nav_local_test.md) —— 缺 gymnasium 时 `test_env` 干净跳过，pytest 1 passed / 1 skipped，直接运行 14 项通过
* [无人机定高日志误差与抗扰恢复时间评估](./air/uav_hover_log_eval.md)
* [uav_ppo_nav 部署节点本地测试](./air/uav_ppo_nav_node_test.md) —— 装桩后 pytest 可收集，修复前 7 errors（fixture 'N' not found），修复后 1 passed，直接运行 31 项通过
* [四旋翼悬停油门求解（推力曲线 + 海拔 + 电池电压）](./air/uav_hover_throttle.md) —— 悬停油门 55.8%，4000 m 升到 68.2%，电池 3.5 V/节升到 66.9%


* [基于深度强化学习的无人机最优路径查找（PPO/SAC）](./air/drl_drone_path_planning.md) —— 把 DRL-drones 的 PyBullet 环境重构为 ROS 包：28 维观测 + 4 维机体系速度指令，训练用 Stable-Baselines3、部署用纯 NumPy，并与几何基线同场对比

* [基于 PPO 的无人机固定路线巡航](./air/uav_route_cruise.md) —— 沿给定闭合航线巡航，遇压在航线上的障碍时临时偏离绕行

* [无人机导航强化学习对比实验（PPO/SAC/TD3）](./air/uav_rl_benchmark.md)

* [神经残差动力学与学习式轨迹跟踪](./air/uav_neural_residual_control.md)
* [低空地形跟随：恒定离地高度保持与 DEM 前馈](./air/uav_terrain_follow_agl.md) —— 要求全程保持固定离地高度（AGL 15 m），对比定高、纯雷达反馈与 DEM 前馈三种策略，MuJoCo 闭环验证
* [基于 DQN 的无人机导航避障](./air/uav_dqn_nav.md) —— 与 PPO 同环境同观测，仅替换为 DQN 离散动作算法，用于端到端模型/算法对比
* [城市动态交通下的无人机避障（PPO + DAgger）](./air/uav_traffic_nav.md) —— 用 CARLA 车流做动态障碍；规则专家示范 + 行为克隆 + DAgger，跨路成功率 48%±3.6%，高于贪心（39%）与速度障碍法（19%）
* [无人机ppo模块研究](./air/uav_experiments.md) —— 把键盘控制、PPO 导航、算法对比、航线巡航、动态交通五个实验合并到一页；各实验独立页面仍然保留

* [无人机终端键盘遥控器](./air/drone_teleop.md)
* [应急物资投送点选择与投放时机决策](./air/uav_emergency_drop.md) —— 平抛弹道解析解定提前量，加权贪心选点分架次，MuJoCo 真飞落点偏差 0.1 m

* [AirSim无人机键盘遥控与机载云台控制模块](./airsim_drone_gimbal_teleop/README.md)

* [无人机机载云台目标丢失重捕获与视轴误差评估](./air/uav_gimbal_reacquisition.md)

* [无人机视觉感知置信度退化保护](./air/uav_perception_confidence_guard.md) —— 综合检测置信度、预测熵、推理延迟和帧陈旧度，识别感知退化并输出保持或返航建议

* [无人机应急着陆点安全评估](./air/uav_emergency_landing_selector.md) —— 从坡度、粗糙度、障碍、人员风险和能量裕量中筛选安全落点，附 ROS 2 Humble 实际 launch 验证

* [基于 ROS 消息解耦的无人机键盘遥控](./air/drone_ros_teleop.md)

* [预设航线飞行模拟器](./air/flight_path_simulator.md) —— 纯 Python 离线脚本，按预设时间表播放一段巡航飞行并生成数据集与可视化，不依赖 ROS/仿真器

* [无人机航点轨迹跟踪（PID 控制）](./air/waypoint_tracking.md)

* [基于串级PID的四旋翼航点定点悬停与抗风扰仿真](./air/quad_cascade_pid_hover_wind.md) —— 纯 Python 6 自由度四旋翼刚体动力学仿真，位置-姿态-角速率串级 PID 闭环，含恒定风/阵风/湍流扰动，验证航点跟踪与定点悬停的抗风扰能力

* [基于 GRU 的无人机飞行状态预测与 ROS2 在线误差分析](./air/state_prediction.md)

* [MuJoCo 城市屋顶光伏覆盖率测量](./air/mujoco_uav_rooftop_pv_coverage_survey.md) —— 通过轻量屋顶场景、往复条带航线及同步 RGB-光伏掩码构建可复现的测量数据基础

* [MuJoCo 屋顶光伏覆盖率测量第二天](./air/mujoco_uav_rooftop_pv_coverage_survey_day2.md) —— 轻量语义分割、航迹控制、ROS 2 接口与三种布局评价
* [无人机通信链路退化保护与断联风险预测](./air/uav_link_outage_guard.md) —— 融合 RSSI、延迟、丢包与遥测陈旧度，预测链路风险并输出降速、保持或返航建议

* [神经视觉惯性里程计与三后端 SLAM 对比](./air/uav_neural_vio_slam.md)

* [无人机 EKF 状态估计（IMU/GPS/气压计融合）](./air/uav_ekf_state_estimation.md) —— 6 状态导航滤波、阵风扰动与纯 GPS/航位推算对比

* [MuJoCo 四旋翼仿真与 ROS2 桥接底座](./air/uav_mujoco_sim.md)
* [MuJoCo 四旋翼垂直通道 PPO 定高与抗扰控制](./air/uav_vertical_ppo.md)

* [无人机执行器故障注入与控制退化观测](./air/uav_actuator_fault_sim.md) —— 在 MuJoCo 底座上插入可编程的桨故障（完全失效 / 推力衰减 / 卡死 / 渐进退化）与 IMU 故障，实测"单桨完全失效 0.64 s 必坠、把损失推力均摊给其余三桨也救不回来"，为后续神经容错飞行模块提供数据与评测台
* [四旋翼吊挂负载防摆控制](./air/uav_slung_load.md) —— MuJoCo 台架上对比 PD、ZV 输入整形与摆角反馈，残余摆角从 7.0° 降到 0.1°

* [无人机桨故障健康度估计](./air/uav_rotor_fault_detector_nn.md) —— 用机载 IMU 与控制器指令（13 维特征）经 1D-CNN+GRU 实时回归四桨剩余推力权限，单桨完全失效 0.10 s 检出、误报率 0，同时提供 ROS2 节点与在线估计接口
* [桨故障下的端到端推力策略](./air/uav_fault_e2e_thrust_policy.md) —— 前视深度图 32×32 + 16 维本体量（含四桨健康度与三轴角速率）直接输出四个桨的归一化推力，用 DAgger 从几何控制器蒸馏；healthy/t1e0.3–0.5 工况下约 1/3 成功，诚实记录了"端到端推力输出对回归精度过于敏感"这一限制

* [无人机桨故障后的自主迫降](./air/uav_fault_emergency_landing.md) —— 下视深度相机在线建 2.5D 高度图，全卷积网络给"能不能落"打分选点，另一神经网络输出速度指令闭环飞过去；实测垂直下降穿楼、选点迫降安全着陆（触地垂直速度 0.05 m/s）

* [无人机桨故障下的神经容错控制](./air/uav_neural_fault_tolerant_control.md) —— 几何控制器给出期望合推力与力矩，加权最小二乘按健康度重新分配到四桨（桨彻底失效时主动放弃偏航），再用 5252 参数的 MLP 学分配残差；12 次对比飞行里把跟踪误差降到 0.188 m（相对加权分配降 29%）而生存率不变

* [四旋翼风扰下物资精准空投（MuJoCo）](./air/uav_wind_aware_delivery.md) —— 吊挂物资起飞，侧风下用 MLP 预测落点并迭代补偿释放点；10 组随机风平均误差 0.2674 m，不补偿 1.0532 m

* [无人机编队飞行](./air/uav_formation_flight.md) —— 长机按航点巡航、僚机保持队形（一字/纵队/三角可动态切换），三架共用一条 AirSim RPC，依次起飞、低空展开后进入任务
* [移动地面目标的后方等距跟随](./air/uav_moving_target_follow.md) —— 解析截获预测 + standoff 参考生成 + 偏移方向限速，MuJoCo 真飞跟随误差 0.1~4.6 m
* [神经安全过滤器](./air/uav_neural_safety_filter.md)
* [无人机控制权限饱和与积分饱和监测](./air/uav_control_authority_guard.md) —— 对比期望与实际角速度、执行器饱和和电压下降，识别控制权限不足并输出机动限制建议

* [低空载具的 ROS 示例教程](https://openhutb.github.io/air_doc/airsim_tutorial_pkgs/)
* [手势控制无人机](./air/gesture_control_drone.md)

* [AP-CPP 主动感知覆盖路径规划](./ap_cpp_overfomo.md) —— 基于 AirSim 的无人机主动感知覆盖路径规划，在 OverFOMO 自适应覆盖规划的基础上进一步决定「该去哪里看」

* [无人机航测任务规划（GSD/重叠度/WPML）](./air/survey_mission_planner.md) —— 由相机参数和目标 GSD 反算航高、航带间距、拍照间距与航点，导出 CSV/KML/WPML

* [无人机航点任务飞行前校验](./air/uav_waypoint_preflight_validator.md) —— 检查 ENU 相对航点的高度、重复点、爬升角、转弯角、航程和保守 ETA，并输出 JSON 与 RViz 预览

* [YOLO 感知与无人机控制](./air/openhutb_yolo_mlp_control/README.md)

* [基于阿基米德螺旋的无人机自主搜索与目标定位](./air/uav_spiral_search/spiral_search.md) —— 无人机沿阿基米德螺旋自动覆盖搜索区域，HSV 颜色检测连续确认后锁定目标并估计其 NED 坐标

* [语义分割建图与语义路径规划](./air/uav_semantic_mapping_nav.md)

* [AllocNet 四旋翼避障规划与键盘遥操作](./air/allocnet_teleop.md) —— 在学习型时间分配的避障轨迹规划基础上，补齐键盘航点下发、地图常驻转发与轨迹记录的人机交互层
* [PX4+ROS2+OpenCV 视觉目标追踪](./air/px4_ros2_opencv_tracking/index.md) —— 基于 PX4 Offboard 模式的 OpenCV 颜色目标追踪演示（合成图像输入）
* [无人机机载相机视觉目标跟踪](./air/uav_camera_tracking.md) —— 神经网络感知（图像→目标像素热图）与控制网络解耦训练，与端到端策略在静止/圆周/遮挡/阵风四个场景下闭环对比

* [低空起降场进离场排队与容量评估](./air/uav_vertiport_queue.md) —— 起降位占用区间建模三种排队策略（FIFO/优先级抢占/固定间隔），扫描请求率得到容量曲线与饱和点（80 架次/h），MuJoCo 闭环证明排定时刻真飞可执行

* [状态识别与轨迹绘制](./air/drone_trajectory_logger.md)

* [基于 ROS 2 的禁飞区（电子围栏）动态检测与安全预警系统](./air/geofence.md)

* [单门深度视觉伺服（门中心回归）](./air/uav_gate_center_servo.md) —— 12×16 深度图回归门中心，比例控制器输出 m/s 速度
* [无人机视觉端到端控制（CNN 行为克隆）](./air/uav_vision_nn.md) —— AirSim 前视图像输入，CNN 输出速度指令，端到端闭环飞行
* [无人机深度相机轨迹跟踪（CNN 行为克隆）](./air/uav_depth_track_nn.md) —— 深度相机感知 + 给定圆轨迹跟踪，CNN 输出速度指令

* [RflySim 单机起飞与端口故障排查（补充实验）](./air/rflysim_sitl_first_flight.md)

* [PX4 SITL + MAVROS2 Offboard 起降与失效保护](./air/px4_mavros2_offboard.md)


* [禁飞区检测模块](no_fly_zone_check/index.md)

* [MuJoCo无人机幕墙定距扫描](air/mujoco_facade_capture/index.md)
* [AirSim 多旋翼航点飞行与到达误差评估](air/airsim_multirotor_waypoint_nav.md) —— 起飞 → 4 航点（相对起飞点偏移）→ 降落；悬停中位误差最大 0.41 m，含机载视角动图
* [四旋翼无人机 MuJoCo 建图导航与神经网络控制](./air/aerial_mujoco_slam_nn_nav/README.md)

* [基于神经网络的无人机感知—规划—控制避障](./air/neural_ca_drone_submission/docs/index.md) —— 深度图 CNN 感知、神经启发式 A* 规划与模仿学习控制的分层避障，附端到端对照
  * [任务与方案](./air/neural_ca_drone_submission/docs/overview.md)
  * [模块一 感知：深度图 → 占用栅格](./air/neural_ca_drone_submission/docs/perception.md)
  * [模块二 规划：神经启发式 A*](./air/neural_ca_drone_submission/docs/planning.md)
  * [模块三 控制：模仿学习控制器](./air/neural_ca_drone_submission/docs/control.md)
  * [模块四 端到端：深度图 → 速度](./air/neural_ca_drone_submission/docs/e2e.md)
  * [实验与结果](./air/neural_ca_drone_submission/docs/results.md)
  * [演示动图与录制](./air/neural_ca_drone_submission/docs/gifs.md)
* [AirSim 无人机抗风扰补偿（MLP）](./air/uav_airsim_wind_rejection.md) —— IMU 时序加位置误差进 MLP，直接输出 xy 补偿速度；侧风下平均偏移从 14.68 m 降到 3.42 m（-77%），验证 R² 0.997

* [AirSim 无人机深度避障（CNN 端到端控制）](./air/uav_airsim_depth_obstacle_avoidance.md) —— 前视深度 64×64 进手写 CNN，出偏航角速率与目标高度，验证集 MSE 0.0640 / R² 0.298

* [无人机自主充电/机巢仿真](./air/drone_charging_nest.md) —— MuJoCo 四旋翼起飞、巡检、返航、机巢自动充电与再次起飞

* [AirSim 无人机 IMU 速度估计（MLP）](./air/uav_airsim_imu_velocity_estimator.md) —— IMU 5 帧 30 维进，单隐层 MLP 出速度大小，验证 R² 0.453、实飞 MAE 1.32 m/s
* [倾转旋翼过渡走廊与倾转角调度](./air/uav_tiltrotor_transition_corridor/index.md) —— 平飞平衡方程 + 功率/前进比双约束二分求走廊，按走廊调度倾转角并做掉高保护

* [AirSim 无人机 IMU 姿态估计（MLP）](./air/uav_airsim_attitude_estimator.md) —— IMU 5 帧 30 维进单隐层 MLP 回归 roll/pitch，平均 MAE 0.9°
* [四旋翼高度 PD + 扩张状态观测器（ESO/ADRC）抗扰控制](./air/quad_eso_hover.md) —— PD 稳态偏差 0.640 m、PD+ESO 近似 0，扰动估计收敛至 -5.810，9 项单测

## 6. 水域载具  <span id='water_vehicle'></span>

### 感知

* [水下声呐 SLAM 占据栅格建图与自主避障导航](./water/rov_sonar_slam_navigation.md)
* [MuJoCo 水下 ROV 串级定深控制](./water/mujoco_rov_depth_control.md) —— 浮力/阻尼显式建模，三段定深稳态误差 0.013 m

* [水下 ROV AprilTag 视觉定位与定点巡航仿真](./water/rov_apriltag_cruise.md)

* [水下 ROV 视觉引导的沉船结构巡检采样仿真](./water/rov_wreck_inspection.md)

* [水下 ROV 海底管道往返巡检（视觉感知 + EKF-SLAM + 端到端）](./water/rov_pipeline_follow.md) —— 140° 广角前视相机 + 逐列深度，色度编码应答器做 EKF-SLAM，级联控制沿管道偏置 0.9 m 完成 A→B→A 往返；四场景（直管/弯管/横流/浑水）× 五策略对比，含 CNN 感知与端到端策略消融

* [新增水下三维地形测绘模块](./water/underwater_sonar_mapping.md)

* [侧扫声呐图像仿真与 CNN 海底目标分类](./water/mjc_rov_sonar_cnn.md)
* [水下 AUV 矩形巡航与视觉双向避障仿真](./water/auv_end2end.md) —— ROS2 + Gazebo Classic，四状态矩形巡航与单目暗区检测双向避障

* [浑浊水体视觉增强与目标追踪（ROS 2，合成视频演示）](./water/water_vision_enhancement.md) —— 通道增益校正、白平衡、CLAHE 与暗通道去散射增强，KCF/CSRT/MOSSE 目标追踪，附合成浑浊水体演示节点

* [水面无人艇：激光雷达建图与神经网络局部规划](./water/usv_mujoco_nn.md) —— MuJoCo 三自由度船体，射线激光雷达 + log-odds 占据栅格建图，MLP 输出局部目标点实现自主避障，封装为 ROS Noetic 节点
* [水面无人艇 COLREG 会遇避碰](./water/usv_colreg_avoidance.md) —— 按避碰规则判断对遇/交叉/追越局面，让路船自动右转避让

* [USV 弗劳德缩比换算与拖曳功率估算](./water/usv_froude_scaling.md) —— 弗劳德相似定模型速度、λ³ 定阻力、λ^3.5 定功率；默认 10 m 船 10 倍缩尺，模型 2.5 N 换算实船总阻力 1925 N、轴功率 17.5 kW

* [DAVE 水下机器人仿真环境搭建与 REXROV/Oberon7 机械臂作业演示](./water/dave_underwater_sim.md) —— 基于 Project DAVE（ROS Noetic + Gazebo 11）的 REXROV + Oberon7 机械臂水下作业仿真，附安装脚本与截图/动图

### 规划

* [水下多传感器感知与 3D 轨迹跟踪控制](./water/rov_trajectory_tracking.md)

* [水下 ROV 对接任务：声呐扫描 + CNN 标记检测 + 逼近导航](./water/mjc_rov_docking_nav.md)

* [基于热力图的洋流干扰下 ROV 自适应路径规划](./water/water_rov_path_planning.md)

### 控制

* [水域载具示例（手柄 / 航点 / 深度航向指令）](./water/holoocean_examples.md) —— 多代理手柄控制、相机 HUD 与配置文件结构
* [USV 会遇局面判定与 COLREGs 避让](./water/usv_colregs_avoidance.md)

* [水下机器人物理仿真与 6-DOF 键盘运动控制](./water/rov_physical_simulation.md)

* [水下 ROV 六自由度手操与抗流能力评测仿真](./water/rov_handling_eval.md) —— 用漂移率、各向异性比与操控权限利用率 η 量化手操品质，附三组对照实验

* [ROV 浮力-压载配平与初稳性核算](./water/rov_ballast_trim.md) —— 全潜状态按 GM = z_B - z_G（不含水面船 BM = I/V 项）算中性浮力、重心/浮心偏心与横摇周期；默认样机缺 38.5 kg 压载，配平后 GM 42 mm、2° 倾角只对应 1.5 mm 横向偏心
* [ROV 深度保持 PID 控制与扰动恢复仿真](./water/rov_depth_hold.md) —— 负载上浮扰动与波浪扰动下的定深精度、扰动恢复时间，附实现动图
* [ROV 推进器配置矩阵与饱和感知推力分配](./water/rov_thruster_allocation.md) —— 8 推进器 6×8 配置矩阵（秩 6、条件数 10.9），箱约束 QP 分配；超包络时力旋量残差比先伪逆再限幅低 36%（20.00 vs 31.41）
* [ROV 六推进器推力分配（ROS 1/catkin）](./water/rov_thruster_allocation_ros1.md) —— 用推进器布置矩阵的伪逆把六维力/力矩分到 4 个水平推进器和 2 个垂直推进器，对比饱和时 clamp/scale 两种策略，并支持单推进器故障重构

* [动态洋流 ROV 键盘遥操与 MLP 巡航控制](./water/mjc_rov_current_teleop.md)
* [水下 ROV 声呐图像 CNN 目标分类（Gazebo 训练链路）](./water/rov_gazebo_sonar_cnn.md) —— Gazebo 采集图像、3 类 CNN 训练与 2 Hz 定时推理，权重与类别表随包安装

* [手写 PPO 强化学习 ROV 轨迹跟踪与 PID 基线对比](./water/mjc_rov_ppo_tracking.md)

* [同一 ROV 任务 PPO 与 SAC 强化学习算法对比](./water/mjc_rov_rl_compare.md)

* [水下机器人端到端深度神经网络视觉伺服控制模型](./water/rov_end_to_end_nn.md)

* [ROV 水下机器人 MuJoCo 建图导航与神经网络控制](./water/rov_mujoco_slam_nn_nav/README.md)

### 其他（环境、接口与容器）

* [水域载具 ROS2 接口](./water/HoloOcean.md)
* [无人艇自主靠泊：激光雷达与神经网络末段对准](./water/usv_docking_nn.md) —— 凹入式泊位，射线激光雷达 + 纯跟踪末段对准专家，行为克隆 MLP 复现，20 回合成功率 100%，末态横向误差 0.23 m

* [HoloOcean Windows 本机环境搭建](./water/HoloOcean-Windows-Setup.md) —— 在 Windows 上用 Conda 安装 Python 客户端、下载场景包并跑通第一个仿真

* [WSL 跑通 HoloOcean2 的 ROS2 示例（无头模式）](./water/WSL_HoloOcean2_ROS2.md) —— 无 GPU 环境下用 `-nullrhi` 参数跑无头仿真

* [Docker 开发环境](./water/docker.md) —— 开发容器与工作空间挂载

* [HoloOcean ROS2 运行时容器](./water/docker_runtime.md) —— 用 `build_container.sh` 构建运行时镜像

* [声呐数据滤波初步验证](./water/rov_sonar_filter.md)
* [水下机器人 MuJoCo 仿真、声呐 SLAM 与纯 NumPy 神经网络导航](./water/rov_numpy_nn_slam.md) —— 6-DOF ROV 物理仿真、键盘遥控、声呐建图定位、A* 导航与端到端 CNN

如果对文档中的任何问题可以在 [本文档的源码仓库](https://github.com/OpenHUTB/ros2) 中的 [问题](https://github.com/OpenHUTB/ros2/issues) 页面讨论或者提交 [拉取请求](https://github.com/OpenHUTB/.github/blob/master/CONTRIBUTING.md) 直接修改文档。

___
* [欠驱动无人艇低速定点保持与横漂补偿](./water/ros2_usv_station_keeping/index.md) —— 几何制导 + 在线流估计 + 双环控制，无侧推器也能顶流定点
* [多 AUV 协同围捕](./water/multi_agent_hunting.md) —— 拍卖算法做任务分配，水声信道带真实延迟与丢包，包抄点向目标收缩并有捕获判定，支持 seed 复现
* [水面无人艇（USV）视线（LOS）路径跟踪制导](./water/usv_los_guidance.md) —— LOS 制导 + 航向包裹，横漂误差收敛至 0.09 m

## 7. 经典控制 <span id='classical_control'></span>

* [倒立摆平衡小车 (cart-pole, LQR 最优控制)](./classical_control/cartpole_balance.md) —— 用 LQR 让倒立摆从倾斜被拉回竖直保持不倒,演示 ROS 发布/订阅闭环与最优控制
* [倒立摆能量摆起与 LQR 平衡](./classical_control/cartpole_swingup.md) —— 摆杆从下垂位置出发，能量控制泵能摆起、接近竖直后切换 LQR 保持平衡，并对比“只用 LQR 无法摆起”
* [滑模控制（SMC）：二阶系统的鲁棒轨迹跟踪](./classical_control/sliding_mode_control.md) —— 边界层切换律，到达条件 K > D_max；常值扰动下 SMC 平均误差 0.012，PD 0.255


## 8. 机械臂操作 <span id='manipulation'></span>

* [MuJoCo 单臂抓取与 π0 推理接口复现（CPU 演示）](./manipulation/pi0_mujoco_grasp.md) —— 基于 MuJoCo 的单臂抓取/放置仿真，含 π0 观测→动作块接口（CPU 策略替身）

* [二连杆机械臂 OpenSim 建模与 PD 控制仿真](./two_link_arm_opensim_simulation.md)

* [ROS2 话题桥接（二连杆机械臂肌肉骨骼仿真）](./muscle_arm_ros2_bridge.md)
* [二连杆机械臂肌肉骨骼建模与 MuJoCo 仿真](./muscle_arm_mujoco_simulation.md)



## 9. 反应式导航与覆盖 <span id='reactive_navigation'></span>

* [人工势场（APF）：梯度下降式反应式避障与局部极小逃逸](./reactive_navigation/potential_field_avoidance.md) —— 目标引力 + 障碍斥力逐点求梯度，50 m 地图走出 57 m 近直线绕障，含切向滑行逃逸与“卡死”一等结果

* [Bug2 导航：不建图的完备导航与确定性不可达判据](./reactive_navigation/bug2_navigation.md) —— 撞墙—沿墙—回线三行为状态机，目标可达必达、不可达时确定性宣告失败（A*/RRT 给不了的答案）

* [牛耕式全覆盖：扫遍整个房间的最简单方案](./reactive_navigation/boustrophedon_coverage.md) —— 等距平行线扫掠 + 之字掉头，L 形房间 100% 覆盖，路径长度 ≈ 面积/行距 + 掉头开销，作业时长可预算


## 参考文献
* [MuJoCo 失落公园无人机端到端视觉转向](./air/mujoco_abandoned_park_uav_visual_steering.md)
* [四旋翼无人机物理仿真、激光 SLAM 与禁飞区规避](./air/uav_numpy_nn_slam.md) —— 城市/峡谷/机场三场景、72 波束激光 SLAM、A* 绕禁飞区、纯 NumPy 神经网络与端到端 CNN
