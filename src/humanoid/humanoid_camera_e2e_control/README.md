# humanoid_camera_e2e_control

人形机器人**单目相机图像端到端神经网络控制**功能包（MuJoCo 物理仿真 + ROS 2 Humble）。

跟随视角相机图像直接输入 PilotNet 风格卷积神经网络，网络回归输出线速度与偏航角速度，
形成"像素 → 神经网络 → 控制指令"的闭环；感知、决策、控制全部由神经网络完成，
推理阶段不使用显式障碍物检测、规则避障或人工势场。

## 目录结构

```
humanoid_camera_e2e_control/
├── main.py                          # standalone 主入口（teacher / collect / train / run / test / ros2）
├── main.sh                          # 一键脚本：规则教师采集 → 训练 → 推理演示
├── package.xml / setup.py / setup.cfg
├── requirements.txt
├── .gitignore
├── config/params.yaml               # 仿真、PD 增益、限幅、策略与训练超参
├── models/humanoid_obstacle.xml     # 人形 MJCF + 交错立柱避障通道场景
├── launch/main.launch.py            # ROS2 launch（mode:=run 推理 / mode:=collect 采集）
├── humanoid_camera_e2e_control/
│   ├── sim_core.py                  # MuJoCo 仿真核心（站姿 PD、运动控制、相机渲染、接触检测）
│   ├── network.py                   # E2ECNN：共享卷积主干 + 回归头 + 方向分类辅助头
│   ├── data_io.py                   # 数据集读写、逐帧归一化、指令三态分类
│   ├── keyboard.py                  # 跨平台非阻塞按键读取
│   ├── mujoco_sim_node.py           # ROS2 仿真节点（发布里程计/TF/关节/跟随图像）
│   ├── e2e_policy_node.py           # ROS2 策略节点（订阅图像 → CNN → /cmd_vel）
│   └── keyboard_teleop_node.py      # ROS2 键盘遥控节点（采集模式）
├── scripts/
│   ├── teacher_collect.py           # 规则教师自动采集图像—指令数据集（默认采集路径）
│   ├── collect_data.py              # 人工键盘驾驶采集（备选路径）
│   ├── train_e2e.py                 # 训练 CNN，保存权重与测试集评测指标
│   └── record_demo.py               # 加载权重推理演示，录制 GIF/关键帧与性能指标
├── test/test_standalone.py          # 站姿、渲染、网络限幅、性能与权重加载自动化测试
├── weights/humanoid_cnn.pt          # 随包提交的训练权重（2.28 MB）
└── data/training/                   # 采集数据集（本地生成，不随仓库提交）
```

## 环境依赖

实测环境：Ubuntu 20.04.6 + ROS 2 Humble，Python 3.8.10，MuJoCo 2.3.7，
PyTorch 2.4.1（CPU），NumPy 1.24.4，imageio 2.34.2，Pillow 10.4.0。

```shell
pip3 install -r requirements.txt
```

> 注意：MuJoCo 3.x 在 Python 3.8 下不兼容，本包锁定 `mujoco>=2.3,<3.0`。

## 快速开始（standalone，无需 ROS）

在包根目录执行：

```shell
# 1) 规则教师自动采集数据集（默认路径，无需人工键盘，可复现）
python3 main.py --teacher --max-frames 3000

# 备选：人工键盘驾驶采集（W/S 前后、A/D 转向、空格停止、ESC 退出落盘）
# python3 main.py --collect --max-frames 2000

# 2) 训练端到端 CNN（85% 训练 / 15% 测试，输出 MSE、RMSE、方向准确率）
python3 main.py --train --epochs 50

# 3) 加载权重端到端推理演示，录制 GIF、关键帧与性能指标到 ./demo_assets/
python3 main.py --run --duration 25

# 自动化测试（站姿、渲染尺寸、网络限幅、渲染帧率、单帧推理耗时、权重加载）
python3 main.py --test
```

一键流程：

```shell
bash main.sh
```

## ROS 2 Humble launch

```shell
cd ~/ros2_ws
colcon build --symlink-install --packages-select humanoid_camera_e2e_control
source install/setup.bash

# 端到端推理：仿真节点 + CNN 策略节点
ros2 launch humanoid_camera_e2e_control main.launch.py mode:=run

# 键盘驾驶采集：仿真节点 + 键盘遥控节点
ros2 launch humanoid_camera_e2e_control main.launch.py mode:=collect

# 无图形环境关闭可视化窗口
ros2 launch humanoid_camera_e2e_control main.launch.py mode:=run visualize:=false
```

### 话题

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/cmd_vel` | geometry_msgs/Twist | 订阅 | 速度指令（策略节点 / 键盘节点发布） |
| `/humanoid/odom` | nav_msgs/Odometry | 发布 | 里程计（位置、偏航角、速度） |
| `/joint_states` | sensor_msgs/JointState | 发布 | 关节位置 |
| `/humanoid/camera/image_raw` | sensor_msgs/Image | 发布 | 跟随视角 RGB8（96×128） |
| `/tf` | tf2_msgs/TFMessage | 发布 | world → humanoid/base_link |

## 能力边界

本包移动控制为演示级运动学控制：前进改写底盘 freejoint 世界系线速度，转向直接旋转
根部四元数，不包含双足步态生成；端到端网络只学习"画面到速度指令"的映射，在固定
交错立柱通道内验证图像输入到控制指令的学习闭环，不代表真实步行动力学或跨场景泛化能力。
视角方案、实测指标与已知限制详见模块文档 `docs/humanoid/humanoid_camera_e2e_control.md`。
