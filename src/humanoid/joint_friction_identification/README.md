# joint_friction_identification

关节 Stribeck 摩擦辨识与前馈补偿：在多个恒定速度下采集稳态力矩，先用一维网格搜索确定
Stribeck 速度，再用线性最小二乘解出库仑/黏性/静摩擦系数；随后把辨识模型作为前馈加入 PD 控制，
对比正弦跟踪误差。

## 环境

- 离线：Windows 11、Python 3.12.14、NumPy 2.3.5、Matplotlib 3.11.2（仅绘图）
- ROS 2：Ubuntu 22.04 + ROS 2 Humble

## 构建与运行（ROS 2）

```bash
source /opt/ros/humble/setup.bash
cd ~/hutb_ws
colcon build --symlink-install --packages-select joint_friction_identification
source install/setup.bash
ros2 launch joint_friction_identification main.launch.py
```

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint/velocity_command` | `std_msgs/Float64` | 输出（仿真） | 扫速指令 |
| `/joint/state` | `sensor_msgs/JointState` | 输出（仿真） | 稳态速度与力矩 |
| `/joint/friction` | `std_msgs/String` | 输出（辨识） | 四个摩擦参数与残差（JSON） |

## 离线实验

```bash
cd src/humanoid/joint_friction_identification
python3 main.py --out results --figure-dir ../../../docs/img/humanoid/joint_friction_identification
python3 -m unittest discover -s tests -v
```

## 主要结果

真值：库仑 0.45 N·m、黏性 0.08、静摩擦 0.65 N·m、Stribeck 速度 0.20 rad/s。

| 参数 | 真值 | 辨识值 | 误差 |
|---|---|---|---|
| 库仑摩擦 [N·m] | 0.45 | 0.4425 | −1.7 % |
| 黏性系数 | 0.08 | 0.0917 | +14.6 % |
| 静摩擦 [N·m] | 0.65 | 0.6434 | −1.0 % |
| Stribeck 速度 [rad/s] | 0.20 | 0.2380 | +19.0 % |

| 跟踪 | RMS 误差 [rad] | 最大误差 [rad] |
|---|---|---|
| 仅 PD | 0.00703 | 0.0120 |
| PD + 摩擦前馈 | **0.00310** | 0.0097 |

详见 `docs/humanoid/joint_friction_identification.md`。
