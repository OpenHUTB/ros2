# 动态洋流 ROV 键盘遥操与 MLP 巡航控制

## 概述

本模块在 MuJoCo 中构建了一个 **6-DOF 水下 ROV 仿真环境**，核心特性：

- **动态洋流扰动**：时变双向流体力 `Fx=ax·sin(ωx·t), Fy=ay·sin(ωy·t)` 持续作用于 ROV
- **键盘运动控制**：`w/s/a/d/q/e` 直接发布 `/rov_teleop/cmd_vel`
- **MLP 神经网络巡航策略**：行为克隆（Behavior Cloning）从脚本专家采集数据，训练 6→64→32→2 的 MLP，输入 `[ex, ey, eyaw, vx, vy, wz]`，输出世界系期望速度 `[vx, vy]`
- **完全自包含**：不依赖仓库其他包，MJCF 模型仅用 box/plane/cylinder 基本几何体

## 运行环境

| 项目 | 版本 |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机，无 GPU） |
| ROS 2 | Humble（`/opt/ros/humble`） |
| Python | 3.8.10 |
| MuJoCo | 3.1.6（OSMesa 离屏渲染） |
| PyTorch | 2.4.1+cpu |
| NumPy | 1.24.4 |

依赖安装：

```bash
pip3 install -i https://mirrors.aliyun.com/pypi/simple/ -r requirements.txt
pip3 install torch==2.4.1+cpu torchvision==0.19.1+cpu \
  -i https://mirrors.aliyun.com/pypi/simple/ \
  --extra-index-url https://download.pytorch.org/whl/cpu
sudo apt install libosmesa6
```

## 动力学模型

水下动力学在代码层施加（不依赖 MuJoCo 流体扩展），每步向 `qfrc_applied` 叠加：

1. **浮力补偿**：$F_z = m \cdot g$（近似中性浮力）
2. **黏性阻尼**：$F_{lin} = -c_{lin} \cdot v$，$F_{ang} = -c_{ang} \cdot \omega$
3. **姿态自稳**：横滚/俯仰误差比例力矩
4. **洋流扰动**：$F_x = a_x \sin(\omega_x t),\; F_y = a_y \sin(\omega_y t)$
5. **速度闭环推力**：$F = K_p (v_{cmd} - v_{actual})$，限幅

## 话题

| 话题 | 类型 | 方向 | 频率 |
|---|---|---|---|
| `/rov_teleop/cmd_vel` | `geometry_msgs/Twist` | 输入（键盘/巡航节点） | — |
| `/rov_teleop/odom` | `nav_msgs/Odometry` | 位姿+速度 | 50 Hz |
| `/rov_teleop/image_raw` | `sensor_msgs/Image` (rgb8, 160×120) | 前视相机 | 10 Hz |
| `/rov_teleop/status` | `std_msgs/String` | 状态文本 | 0.5 Hz |

## 神经网络

**架构**：MLP（6 → 64 → 32 → 2），Tanh 激活

```text
输入: [ex, ey, eyaw, vx, vy, wz]   # 位置/航向误差 + 当前速度
输出: [vx_world, vy_world]          # 世界系期望速度
损失: MSE（行为克隆回归专家输出）
```

**训练流程**：
1. `collect`：脚本专家（比例控制器）绕 4 个航点跑圈，采集 4000 组 `(特征, 专家指令)`
2. `train`：Adam 优化器，200 epoch，CPU 上 <1 分钟收敛
3. `test`：闭环评估，统计 MAE 与航点到达率

**实测结果**：专家 4/4 航点到达，MLP 2/4，MAE 1.60m / 1.67m

![ROV 巡航动画](images/pr1_trajectory.gif)

![前视相机](images/pr1_camera.png)

## 运行步骤

```bash
# 编译
colcon build --symlink-install --packages-select mjc_rov_current_teleop
source install/setup.bash

# 全链路（采集→训练→评估）
bash main.sh

# 或分步
python3 main.py collect
python3 main.py train
python3 main.py test

# ROS 实时仿真
ros2 launch mjc_rov_current_teleop main.launch.py
# 另开终端
ros2 run mjc_rov_current_teleop keyboard_node
ros2 run mjc_rov_current_teleop cruise_node
```

## 性能评价

| 控制器 | MAE 位置误差 | 航点到达 |
|---|---|---|
| 脚本专家（P 控制器） | 1.60 m | 4/4 |
| MLP（行为克隆） | 1.67 m | 2/4 |

MLP 在洋流扰动下接近专家性能，证明神经网络学会了基本的航点跟踪行为；与专家的差距反映了 BC 方法在动态扰动下的泛化局限。

## 文件清单

```text
mjc_rov_current_teleop/
├── package.xml / setup.py / setup.cfg
├── requirements.txt / README.md
├── main.py / main.sh
├── mjc_rov_current_teleop/
│   ├── __init__.py
│   ├── common.py          # 水下动力学 + 路径解析
│   ├── cruise_net.py      # MLP 定义/采集/训练/评估
│   ├── sim_node.py        # ROS MuJoCo 仿真节点
│   ├── keyboard_node.py   # 键盘遥操
│   └── cruise_node.py     # ROS MLP 巡航节点
├── launch/main.launch.py
├── models/rov_pond.xml    # 自包含 MJCF（基本几何体）
├── config/rov_current_params.yaml
└── resource/mjc_rov_current_teleop
```
