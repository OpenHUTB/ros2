# uav_hover_throttle 四旋翼悬停油门求解

给定机体质量、桨径与电池电压，悬停需要多少油门？本模块用**推力曲线 + ISA 大气密度 +
电机电压-转速关系**把这件事算清楚，并给出海拔、载重、电池衰减三个维度的影响。

与仓库已有模块的区别：

- `payload_mass_estimation` 是从飞行数据**估质量**；
- `uav_energy_budget` 算的是**航时/航程的能量预算**；
- 本模块算的是**悬停油门这一个工作点**，以及它随密度和电压怎么变，属于操纵性/配平问题。

## 1. 运行环境

- 操作系统：Ubuntu 22.04.5 LTS（实测）
- ROS 版本：ROS 2 Humble
- Python：3.10.12
- 依赖：`rclpy`、`std_msgs`；离线入口与测试只用标准库

## 2. 编译与测试

```bash
cd ~/ros2 && source /opt/ros/humble/setup.bash
colcon build --packages-select uav_hover_throttle
source install/setup.bash
colcon test --packages-select uav_hover_throttle
```

## 3. 运行

```bash
cd src/air/uav_hover_throttle
python3 -m uav_hover_throttle.main
```

实测输出（1.6 kg、4 旋翼、10 寸桨、4S 满电 16.8 V）：

```text
基准状态（海平面、满电 16.8 V）
  悬停转速 5018 rpm，最大转速 9000 rpm，悬停油门 55.8%
  悬停推力 15.69 N（重量 15.69 N），电功率 158.0 W = 98.8 W/kg
```

| 起飞质量 | 悬停转速 | 悬停油门 | 电功率 |
| --- | --- | --- | --- |
| 1.28 kg | 4488 rpm | 49.9% | 113.1 W |
| 1.60 kg | 5018 rpm | 55.8% | 158.0 W |
| 1.92 kg | 5497 rpm | 61.1% | 207.7 W |
| 2.24 kg | 5937 rpm | 66.0% | 261.8 W |

| 海拔 | 空气密度 | 悬停转速 | 悬停油门 | 电功率 |
| --- | --- | --- | --- | --- |
| 0 m | 1.2250 kg/m³ | 5018 rpm | 55.8% | 158.0 W |
| 1000 m | 1.1116 | 5267 rpm | 58.5% | 170.2 W |
| 2000 m | 1.0065 | 5536 rpm | 61.5% | 184.0 W |
| 3000 m | 0.9091 | 5825 rpm | 64.7% | 199.7 W |
| 4000 m | 0.8191 | 6136 rpm | 68.2% | 217.7 W |

| 总电压（4S） | 单节 | 可用最大转速 | 悬停油门 |
| --- | --- | --- | --- |
| 16.80 V | 4.20 V | 9000 rpm | 55.8% |
| 16.00 V | 4.00 V | 8571 rpm | 58.5% |
| 15.40 V | 3.85 V | 8250 rpm | 60.8% |
| 14.80 V | 3.70 V | 7929 rpm | 63.3% |
| 14.00 V | 3.50 V | 7500 rpm | 66.9% |

ROS 2 节点（在线求解，输入变化即重算）：

```bash
ros2 launch uav_hover_throttle main.launch.py
ros2 launch uav_hover_throttle main.launch.py mass:=2.2 altitude:=3000.0
```

- 订阅 `/uav/battery_voltage`、`/uav/altitude`（`Float64`，可选）
- 发布 `/uav_hover/throttle`（`Float64`，>1 表示飞不起来）
- 发布 `/uav_hover/status`（`String`）：密度、转速、油门、功率

## 4. 模型

```text
推力     T = Ct * rho * n^2 * D^4                 n 为转速（rev/s）
诱导功率 P_ind = T^1.5 / (FoM * sqrt(2 * rho * A_total))
型阻功率 P_prof = k_prof * n^3
电功率   P_elec = P_ind + P_prof
悬停转速 n_hover = sqrt(m*g / (Ct * rho * D^4))
悬停油门 throttle = n_hover / n_max(U)，n_max(U) = n_nom * U / U_nom
```

空气密度按 ISA 标准大气：`rho = rho0 * (1 - L*h/T0)^(g/(R*L) - 1)`。

## 5. 关键结论

1. 悬停油门对**质量**最敏感（+40% 质量 → 油门 55.8% → 66.0%）；
2. 海拔 4000 m 时空密度只剩 67%，同样质量需要把转速从 5018 提到 6136 rpm；
3. 电池从 4.20 V/节掉到 3.50 V/节，悬停油门从 55.8% 涨到 66.9%——这就是「电量低时手感变肉」；
4. 满电时 8000 m 以内都能悬停，但最弱电量下油门已到 66.9%，继续掉电压就会失去悬停余量。

## 6. 简化假设

- 推力系数 `Ct`、型阻系数按常数处理，不随雷诺数/来流角变化；
- 电机转速与电压按线性关系近似，未建模内阻压降与电调饱和；
- 未考虑地面效应、桨间干扰、机身阻塞；
- 悬停功率按动量理论理想值除以 figure of merit，未用真实台架数据标定；
- 未接入 AirSim/MuJoCo，纯解析计算 + 在线节点。

## 7. 大模型使用声明

使用大模型辅助生成代码、测试与文档；作者已复核动量理论公式、参数取值与实测数据，
并对提交内容负责。
