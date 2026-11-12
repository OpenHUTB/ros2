# wheel_speed_observer 轮速传感器 + 龙伯格观测器估计传动轴扭角

只有 ABS 齿圈能测到车轮转速时，轴扭矩/扭角还能不能知道？本模块用**龙伯格观测器**回答：
能，只要系统可观测，并且要付出「噪声—带宽」的代价。

与仓库已有模块的区别：

- `odom_health_monitor` 监视里程计健康度（阈值与告警），不做状态估计；
- `imu_allan_noise_identification` 辨识 IMU 噪声参数，不设计观测器；
- 本模块做的是**用软件替代扭矩传感器**：从电机转速与轮速反推传动轴扭角。

## 1. 运行环境

- 操作系统：Ubuntu 22.04.5 LTS（实测）
- ROS 版本：ROS 2 Humble
- Python：3.10.12
- 依赖：`rclpy`、`std_msgs`；离线入口与测试只用标准库

## 2. 编译与测试

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select wheel_speed_observer
source install/setup.bash
colcon test --packages-select wheel_speed_observer
```

## 3. 运行

```bash
cd src/ground/wheel_speed_observer
python3 -m wheel_speed_observer.main
```

实测输出：

```text
能观性矩阵 = [[0.0000, 1.0000], [2000.0000, -3.3333]]
det = -2000.0000  →  可观测
开环极点 -1.667+44.690j / -1.667-44.690j（扭振固有频率 44.7 rad/s）

带宽倍数  wn(rad/s)              L=[l1,l2]                 极点   收敛时间(s)  噪声增益  扭角RMS
      0.5       22.4      [-0.750,27.972]      -15.7±16.0j     0.110     27.98   0.00027
      1.0       44.7      [-0.000,59.277]      -31.3±31.9j     0.045     59.28   0.00016
      2.0       89.4      [3.000,121.886]      -62.6±63.9j     0.040    121.92   0.00013
      3.0      134.2      [8.000,184.496]      -93.9±95.8j     0.030    184.67   0.00022
```

`收敛时间` = 扭角估计误差从初值 0.02 rad 收敛到 10% 的时间；
`噪声增益` = ‖L‖₂；`扭角RMS` = 车轮转速叠加 σ=0.05 rad/s 噪声后、稳态段的扭角误差 RMS。

**关键现象**：带宽越高收敛越快，`‖L‖` 单调增大；但扭角误差 RMS 并不单调——
它主要经 `l1` 进入，而 `l1` 在基准带宽附近恰好接近 0（上表 1.0 倍的 `-0.000`），
所以最干净的扭角估计出现在「带宽适中」而不是「越高越好」。

ROS 2 节点：

```bash
ros2 launch wheel_speed_observer main.launch.py
# 换带宽
ros2 launch wheel_speed_observer main.launch.py bandwidth_factor:=3.0
```

- 订阅 `/drivetrain/motor_speed`、`/drivetrain/wheel_speed`（`Float64`，rad/s）
- 发布 `/wheel_speed_observer/theta_hat`（`Float64`，rad）
- 发布 `/wheel_speed_observer/status`（`String`）

## 4. 观测器设计

```text
x1' = u - x2                     (u = 电机转速，已知输入)
x2' = (k*x1 + c*(u - x2)) / Jw   (x2 = 车轮转速，唯一量测)
A = [[0,-1],[k/Jw,-c/Jw]]，B = [[1],[c/Jw]]，C = [0,1]
x_hat' = A·x_hat + B·u + L·(y - C·x_hat)
```

能观性矩阵 `[C; CA]` 的行列式为 `-k/Jw ≠ 0`，系统可观测。
`L` 通过**对偶**方式求：把 `(Aᵀ, Cᵀ)` 当成受控对，用 Ackermann 得到 `Lᵀ`，再转置。

## 5. 简化假设

- 只保留扭振模态，假设电机转速已知（编码器足够准），轮速是唯一测量；
- 噪声按白噪声建模，未包含齿圈加工误差的周期性成分（实际会出现轮速阶次噪声）；
- 观测器按连续时间设计，节点用 1 ms 步长积分，未做离散化误差分析；
- 未考虑轮速在低速/零速时不可观测（ABS 齿圈在极低转速下丢脉冲）的情况；
- 未做参数摄动鲁棒性验证。

## 6. 大模型使用声明

使用大模型辅助生成代码、测试与文档；作者已复核可观测性、对偶 Ackermann 实现与
实测数据，并对提交内容负责。
