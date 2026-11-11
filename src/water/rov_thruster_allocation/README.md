# rov_thruster_allocation

ROV 推进器配置矩阵与饱和感知的推力分配：8 推进器（4 水平 + 4 垂直）构成 6×8 配置矩阵，
对比伪逆、伪逆后限幅与箱约束 QP 三种分配方式在"可达/超包络"两类需求下的误差与推力占用。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select rov_thruster_allocation
source install/setup.bash
ros2 launch rov_thruster_allocation main.launch.py scale:=1.0
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/rov/wrench_demand` | `geometry_msgs/Wrench` | 输出（仿真） | 目标力/力矩 |
| `/rov/thruster_forces` | `std_msgs/Float64MultiArray` | 输出（分配） | 8 个推力指令 |
| `/rov/allocation_status` | `std_msgs/String` | 输出（分配） | 残差、推力平方和、饱和数（JSON） |

## 离线实验

```bash
cd src/water/rov_thruster_allocation
python3 main.py --out results --figure-dir ../../../docs/img/water/rov_thruster_allocation
python3 -m unittest discover -s tests -v
```

## 主要结果

| 场景 | 伪逆（不限幅） | 伪逆后限幅 | QP 分配 |
|---|---|---|---|
| 包络内 残差 RMS | 0.00 | 2.63 | **0.00** |
| 包络内 触限比例 | 2.8 % | 2.8 % | 3.7 % |
| 超包络 残差 RMS | 0.00（不可实现） | 31.41 | **20.00** |
| 超包络 触限比例 | 29.6 % | 29.6 % | 39.3 % |

详见 `docs/water/rov_thruster_allocation.md`。


**口径**：QP 的箱约束是硬约束，max|f| 恒为 40 N，所以它那一列是“触限比例”（顶在限幅边界），不是“越限”。真正的越限只发生在未限幅伪逆上：包络内 2.75 %、超包络 29.62 %，峰值分别到 54.5 N 与 98.1 N。包络内伪逆的零残差建立在这样不可实现的解上，原因是 8 个推进器只张成 6 维力旋量、零空间 2 维，最小范数解不等于生成需求的 f_true，可能把某些分量推过 40 N。
