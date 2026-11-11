# 双关节肌肉骨骼手臂到达控制（MuJoCo + ROS 2）

上臂 + 前臂两个关节，由 **6 条 Hill 肌肉**通过固定腱驱动，其中两条是**双关节肌**
（同时跨肩、肘）。控制器只输出肌肉激活，关节力矩由 MuJoCo 按腱长、速度与力臂计算。

补的是 [`docs/musculoskeletal/msk_arm_reach.md`](../../docs/musculoskeletal/msk_arm_reach.md)
里描述、此前没有 `src/` 实现的模块。

- 包名：`mujoco_musculoskeletal_arm_reach`
- 构建：`ament_cmake`
- 依赖：`mujoco`（pip）、`rclpy`、`std_msgs`、`sensor_msgs`、`geometry_msgs`

## 1. ROS 2 计算图

```
/target_position ──► 平面两连杆 IK ──► 肩/肘 PD(+积分, 重力前馈)
                                          └─► 六肌肉协同分配 ─► /muscle_activations
                                                                  │
                                            MuJoCo（固定腱 + Hill 肌肉）
                                                                  ├─► /joint_states
                                                                  ├─► /end_effector_position
                                                                  └─► /muscle_forces
```

| 话题 | 类型 | 方向 | 内容 |
| --- | --- | --- | --- |
| `/target_position` | `geometry_msgs/Point` | 订阅 | 用 `x`、`z`，`y` 忽略 |
| `/joint_states` | `sensor_msgs/JointState` | 发布 | 肩、肘位置与速度 |
| `/end_effector_position` | `geometry_msgs/Point` | 发布 | 末端在手臂平面的 `x-z` |
| `/muscle_activations` | `std_msgs/Float64MultiArray` | 发布 | 6 条肌肉激活（顺序固定） |
| `/muscle_forces` | `std_msgs/Float64MultiArray` | 发布 | MuJoCo 计算的肌力 [N] |

## 2. 模型

`models/arm.xml`：

| 项目 | 数值 |
| --- | --- |
| 上臂 / 前臂长度 | 0.30 m / 0.28 m |
| 肩关节范围 | −1.25 … 1.45 rad |
| 肘关节范围 | −0.10 … 2.30 rad |
| 物理步长 / 控制周期 | 2 ms / 10 ms（5 个物理步，100 Hz） |
| 肌肉 | 6 条（肩屈/伸、肘屈/伸、双关节屈/伸），`F_max = 800 N` |

六条肌肉用**固定腱**的关节系数定义力臂：

| 肌肉 | 肩系数 | 肘系数 |
| --- | --- | --- |
| `shoulder_flexor` / `shoulder_extensor` | ∓0.030 | 0 |
| `elbow_flexor` / `elbow_extensor` | 0 | ∓0.025 |
| `biarticular_flexor` / `biarticular_extensor` | ∓0.020 | ∓0.020 |

> 符号坑：MuJoCo 肌肉缩短腱长，广义力是 `−F·∂L/∂q`。所以“屈肌”要取**负**系数、
> “伸肌”取正系数，才会产生 `+q` 方向的正力矩。

固定腱长度是关节坐标的线性组合，`q=0` 时为 0，缩短时可为负。因此三条肌腱的
`lengthrange` 分别取 `±0.09 / ±0.12 / ±0.14 m`，覆盖全部关节行程，零位对应归一化
长度的中值。

## 3. 控制

```
x = L1 cos q1 + L2 cos(q1+q2)
z = −L1 sin q1 − L2 sin(q1+q2)
```

1. 目标点做可达性检查（臂展内半径、肩/肘限位），不可达或非有限就**保留上一目标**；
2. 肘上构型 IK → `(q1*, q2*)`；
3. 肩、肘 PD + 积分 + 重力/科氏前馈（`qfrc_bias`）→ `τ1, τ2`；
4. 协同分配：

```
τ1 > 0 → shoulder_flexor      τ1 < 0 → shoulder_extensor
τ2 > 0 → elbow_flexor         τ2 < 0 → elbow_extensor
共同部分 → biarticular_flexor / biarticular_extensor
```

控制器不直接写关节力矩，只写 6 维激活；力矩由 MuJoCo 的肌腱/肌肉模型产生。
仿真异常时清零激活并停止。

## 4. 运行

```bash
python3 -m pip install -r requirements.txt
colcon build --packages-select mujoco_musculoskeletal_arm_reach --symlink-install
source install/setup.bash

ros2 launch mujoco_musculoskeletal_arm_reach main.launch.py
# 或
ros2 run mujoco_musculoskeletal_arm_reach arm_sim_node --seconds 3.5

ros2 topic pub --once /target_position geometry_msgs/msg/Point \
  "{x: 0.30, y: 0.0, z: -0.10}"
```

离线跑一次闭环：

```bash
python3 -c "from mujoco_musculoskeletal_arm_reach import load_config, run_reach; \
print(run_reach(load_config()).metrics())"
```

## 5. 实测结果（WSL2 Ubuntu 24.04 + MuJoCo 4.0.0 + ROS 2 Jazzy）

默认目标 `(0.38, −0.17)`：

| 指标 | 本实现 | 文档 |
| --- | --- | --- |
| 3.2 s 末末端误差 | **0.0039 m** | 0.005 ～ 0.007 m |
| 首次进入 1 cm | **0.21 s** | — |
| 节点运行 3.5 s 末误差 | **0.0046 m** | — |
| 运行 12 s 末误差 | **0.0024 m** | — |
| 目标 (0.30, −0.10)、运行 6 s 末误差 | **0.0013 m** | — |

```
[INFO] arm_reach 启动：目标 (0.38, -0.17)，控制周期 10 ms，6 条肌肉
[INFO] 运行 3.5s 结束，末端误差 0.0046 m
[WARN] 拒绝不可达目标 (2.000, 0.000)，保留上一目标
```

## 6. 测试

```bash
python3 -m pytest -q test/     # 23 个用例
colcon test --packages-select mujoco_musculoskeletal_arm_reach
```

覆盖：自由度/腱/肌肉数量、腱长覆盖全行程、双关节腱跨两关节、正运动学与 MuJoCo
对照、IK/FK 闭环、不可达与限位拒绝、主动肌选择、双关节分配、激活范围、积分限幅、
3.2 s 收敛到 1 cm。

## 7. 文件结构

```
msk_arm_reach/
├── models/arm.xml
├── config/simulation.yaml
├── launch/main.launch.py
├── mujoco_musculoskeletal_arm_reach/
│   ├── config.py      # 参数与校验
│   ├── model.py       # MuJoCo 手臂、正运动学、状态
│   ├── control.py     # IK、PD+前馈、六肌肉分配
│   ├── simulation.py  # 闭环到达与指标
│   └── node.py        # rclpy 节点
├── scripts/arm_sim_node
└── test/
```

## 8. 已知限制

- 肌肉力只由 MuJoCo 的 Hill 模型给出，没有对单块肌肉做参数标定；
- 分配是启发式（按力矩符号 + 双关节共享），不是最优肌力分配（如最小平方和）；
- 只做矢状面平面运动，没有腕关节与末端姿态约束；
- 目标切换是阶跃的，没有做轨迹插值。
