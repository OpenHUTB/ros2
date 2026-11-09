# 四旋翼单桨失效后的推力分配可行域与姿态权限评估

本模块用一个静态推力分配模型回答三个工程问题：

1. 四桨完好时，悬停/平飞过程中还能剩多少滚转、俯仰、偏航力矩；
2. 一个旋翼失效后，剩余三桨还能不能维持悬停，能维持多大力矩；
3. 想让「单桨失效后仍能悬停」成立，推重比 T/W 至少要留到多少。

算法、公式、参数与结果图见[完整说明](../../../docs/air/uav_rotor_failure_allocation.md)。

## 环境与运行

离线部分只需要 Python 3.8 及以上、NumPy 与 Matplotlib，不需要 ROS、飞控或仿真器：

```bash
cd src/air/uav_rotor_failure_allocation
python3 -m pip install -r requirements.txt
python3 main.py                 # 打印余量表, 输出 3 张 PNG
python3 main.py --gif           # 额外输出偏航扫描 GIF
python3 main.py --mass 2.0 --arm-length 0.25 --torque-ratio 0.016 --max-thrust 9.81
python3 main.py --failed-rotor 2     # 指定失效哪个桨 (0..3)
```

默认参数是 2 kg 四旋翼、力臂 0.25 m、反扭矩系数 0.016 m、单桨最大推力 9.81 N
（即 T/W = 2）。结果写入 `outputs/`：`feasible_set.png`、`yaw_authority.png`、
`authority_margin.png`、`report.json`，加 `--gif` 再输出 `feasible_set.gif`。
用 `--output <目录>` 改输出位置。

ROS 2 部分使用 Ubuntu 22.04 / Humble 与系统 Python：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2
colcon build --packages-select uav_rotor_failure_allocation
source install/setup.bash
ros2 launch uav_rotor_failure_allocation main.launch.py
```

节点订阅 `uav/desired_wrench`（`geometry_msgs/WrenchStamped`，只取 `force.z` 作为总推力），
发布 `uav/rotor_thrust`（`std_msgs/Float64MultiArray`，四桨推力 N）与
`uav/allocation_status`（`std_msgs/String`，`ok` / `infeasible`）。参数 `failed_rotor`
取 `-1` 表示四桨完好，取 `0..3` 表示该桨失效；`max_thrust` 取 0 时按 T/W = 2 推算。

## 测试

```bash
python3 -m pip install -r requirements-dev.txt
python3 -m pytest test -v
```

14 项检查，覆盖分配矩阵、mg/4 悬停解、分配往返、推力上限、零力矩推力上限、
单桨失效悬停解与被动偏航、低推重比不可悬停、偏航权限公式三段、菱形/三角形可行域、
可行域与逐点分配的一致性（随机采样 4000 点）、参数校验。

## 验证范围

离线算法在 Windows 11 / Python 3.12 / NumPy 2.3.5 / Matplotlib 3.11.2 上运行，
`python3 -m pytest test -v` 14 项通过，`main.py` 生成的图与表见文档页。
未验证的部分：ROS 2 节点未在本机运行（本机没有 ROS 环境，只做了语法检查），
未在真机或 PX4/ArduPilot 上对比，未覆盖双桨及以上失效、桨叶卡死/风车阻力、
气动干扰与姿态环动态——这些都要另做带动力学的仿真或试飞才能下结论。

## 使用声明

使用 DeepSeek Harness（deepseek-flash）辅助实现与文档整理；提交者负责审阅、理解并对提交内容负责。
