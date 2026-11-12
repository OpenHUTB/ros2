# uav_rigidbody_keyboard_teleop

四旋翼 6 自由度刚体动力学键盘遥控仿真：推力-力矩混合分配、重力与气动阻力积分，高度保持 + 姿态 PD 稳定内环，键盘油门/俯仰/滚转/偏航遥控。

## 概览

| 项目 | 内容 |
| --- | --- |
| 话题类别 | 任务 1：物理仿真 + 键盘运动控制 |
| 平台 | Ubuntu 22.04 / ROS 2 Humble（仓库只有 ROS 2 包，无 ROS 1 版本）；离线仿真可在 Windows Python 3.13 运行 |
| 语言 | Python 3 |
| 依赖 | rclpy、nav_msgs、geometry_msgs、sensor_msgs；离线仅依赖 numpy、pygame |
| 运行方式 | `ros2 launch uav_rigidbody_keyboard_teleop uav_rigidbody_keyboard_teleop.launch.py`；`ros2 run uav_rigidbody_keyboard_teleop teleop_node`；离线 `python main.py --shots` |
| 仿真器 | 自研四旋翼 6-DOF 刚体动力学仿真器（Python + pygame 离屏/窗口渲染） |

## 原理

- 机体系 x 前、y 左、z 上；世界系 ENU。
- 四个旋翼推力 `T_i = k_f ω_i²`，反扭矩 `τ_i = k_tau ω_i² s_i`。
- 逆混合矩阵把期望合力 `T` 与三轴力矩 `τ_x, τ_y, τ_z` 分配到四个旋翼推力。
- 平动：`m·v̇ = F_world`；转动：`I·ω̇ = M_body − ω × Iω − 阻尼`。
- 稳定环：高度环按垂速误差补偿总推力；姿态环按目标倾斜角与角速度误差输出三轴力矩。

## 快速运行

### ROS 2 Humble

```bash
colcon build --packages-select uav_rigidbody_keyboard_teleop --symlink-install
source install/setup.bash
ros2 launch uav_rigidbody_keyboard_teleop uav_rigidbody_keyboard_teleop.launch.py
# 等价地直接起节点
ros2 run uav_rigidbody_keyboard_teleop teleop_node
# 覆盖仿真频率（launch 参数透传给节点）
ros2 launch uav_rigidbody_keyboard_teleop uav_rigidbody_keyboard_teleop.launch.py sim_rate:=100.0
```

### 离线仿真（无需 ROS）

```bash
python main.py --shots    # 离屏运行并保存全过程截图到 docs/img/
python main.py --demo     # 打开交互窗口：W/S 俯仰，A/D 滚转，↑/↓ 油门，Q/E 偏航
```

## 测试

```bash
python -m pytest test/ -q     # 10 项：动力学 7、混合分配 3
```

## 文件结构

```
uav_rigidbody_keyboard_teleop/
├── launch/uav_rigidbody_keyboard_teleop.launch.py
├── uav_rigidbody_keyboard_teleop/
│   ├── dynamics.py      # 6-DOF 刚体动力学与混合分配
│   ├── controller.py    # 高度保持 + 姿态 PD 稳定环
│   ├── teleop.py        # 按键集合 -> 杆量指令
│   └── teleop_node.py   # ROS 2 封装节点
├── test/                # 动力学与混合分配单元测试
├── main.py              # 离线一键实验入口
├── package.xml
├── setup.py
└── setup.cfg
```

## 大模型使用声明

过程中借助大模型工具辅助代码编写、整理包目录结构与本描述排版，核心逻辑与操作、场景参数设定与实际运行调试为本人完成，作者对提交内容负责。

## 许可

MIT
