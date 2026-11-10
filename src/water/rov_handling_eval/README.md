# rov_handling_eval

水下 ROV 六自由度手操与抗流能力评测（ROS 2 + MuJoCo）。

和"能动能跑"的入门模块不同，这个包把**手感**变成三个可判定的数：漂移率、各向异性比、抗流操控权限利用率。

```
键盘 ──/cmd_vel(Twist)──▶ 物理内核(500 Hz) ──/rov/odom(50 Hz)──▶ 评测节点 ──/rov/eval──▶ 终端 / CSV
                                  │
                                  ├─ 浮力 + 配重（浮心与质心分离，产生恢复力矩）
                                  ├─ 各向异性二次阻力
                                  └─ 三层洋流（真值单独发到 /rov/current）
```

## 1. 环境

| 项 | 实测/目标版本 | 说明 |
|---|---|---|
| OS | Ubuntu 22.04 / 20.04（教学虚拟机） | 20.04 下必须用 Python 3.8（与 rclpy ABI 绑定） |
| ROS 2 | Humble | `ros-humble-desktop` |
| Python | 3.10（22.04）/ 3.8（20.04） | |
| MuJoCo | ≥ 3.0 | 纯 CPU 物理，**不需要显卡** |

> 本 README 的版本表只写目标环境；实际跑通的组合请在提 PR 时补上你自己的机器信息。

## 2. 依赖

```bash
pip3 install -r src/water/rov_handling_eval/requirements.txt \
     -i https://pypi.tuna.tsinghua.edu.cn/simple
```

## 3. 编译

```bash
cd ~/ros2
colcon build --packages-select rov_handling_eval --symlink-install
source install/setup.bash
```

## 4. 运行

**第一步永远是自检**（不需要 ROS，不需要显卡）：

```bash
./src/water/rov_handling_eval/main.sh --check
```

它会跑 3 s 自由漂移，把实测速度与解析终端速度对比：

```
净浮力        : +2.70 N（≈ +0.27 kg）
解析终端速度  : +0.206 m/s
实测平均速度  : +0.208 m/s（3.0 s 内位移 +0.623 m）
相对误差      : 1.0%（判据 ≤ 25%：通过）
```

方向相反或误差超 25% 会直接报错——**这是坐标/符号写反的最快发现方式**。

然后三种模式任选：

```bash
# A. 纯终端（无显卡）
./src/water/rov_handling_eval/main.sh

# B. 标准 ROS 2（两个终端）
ros2 launch rov_handling_eval eval.launch.py
ros2 run rov_handling_eval keyboard_teleop

# C. 3D 视窗（OpenGL 3.3+）
./src/water/rov_handling_eval/main.sh --gui
```

实验档位直接用 launch 参数切换：

```bash
ros2 launch rov_handling_eval eval.launch.py current_speed:=0.3 ballast:=0.27
ros2 launch rov_handling_eval eval.launch.py current_speed:=0.5 csv_path:=/tmp/eval.csv
```

## 5. 话题

| 话题 | 类型 | 方向 | 频率 |
|---|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | 订阅 | 20 Hz |
| `/rov/depth_hold` | `std_msgs/msg/Bool` | 订阅 | 事件 |
| `/rov/odom` | `nav_msgs/msg/Odometry` | 发布 | 50 Hz |
| `/rov/depth` | `std_msgs/msg/Float64` | 发布 | 50 Hz |
| `/rov/current` | `geometry_msgs/msg/Vector3` | 发布 | 10 Hz |
| `/rov/eval` | `std_msgs/msg/Float64MultiArray` | 发布 | 5 Hz |

`/rov/eval` 的 5 个分量：`[漂移率, η_x, η_y, η_z, 净浮力]`。

## 6. 键位

| 功能 | 键 | 功能 | 键 |
|---|---|---|---|
| 纵荡 | W / S | 横滚 | J / L |
| 横荡 | A / D | 俯仰 | I / K |
| 升沉 | R / F | 转向 | Q / E |
| 定深保持 | H | 急停 | 空格 |
| 推力倍率 | `-` / `+` | 退出 | X 或 Ctrl-C |

## 7. 测试

```bash
# 纯数学部分：不需要 ROS / MuJoCo
python3 -m unittest discover -s src/water/rov_handling_eval/test -p "test_*.py" -v
```

覆盖：净浮力、终端速度解析解、二次阻力方向与平方律、浮心恢复力矩符号、
速度环限幅、η 的文档值、洋流的可复现性与深度剖面。

## 8. 参数

`config/handling.yaml`，改完不用重编译。关键项：

| 参数 | 默认 | 说明 |
|---|---|---|
| `ballast` | 0.0 kg | +0.27 左右抵消净浮力 |
| `displacement` | 0.0110 m³ | 与 `mass` 一起决定净浮力 |
| `drag_linear` | [13.2, 35.7, 63.3] | 纵荡/横荡/升沉阻力系数 |
| `max_force` | [10, 10, 10] N | 单轴可用推力（η 的分母） |
| `current_speed` | 0.0 m/s | 实验档位：0.1 / 0.2 / 0.3 / 0.5 |
| `sim_hz` / `pub_hz` | 500 / 50 | 物理与发布频率 |

## 9. 验证边界（务必读）

| 内容 | 状态 |
|---|---|
| `dynamics.py` / `current_model.py` | ✅ 24 项单元测试通过（无 ROS、无 MuJoCo） |
| `main.py --check` | ⚠️ 需要本机装 MuJoCo 后实跑；作者环境未装，未实测 |
| 三个 ROS 节点 | ⚠️ 已通过语法编译与桩导入，**未做端到端实跑** |
| 第 6 节实验表格 | ⬜ 留给实跑填写 |

如实说明这三点，比在 README 里写"全部验证通过"更负责；提 PR 时请把实跑结果补进第 6 节。

## 10. 已知限制

- 模型是裸 ROV，没有机械臂，因此**不发 `/joint_states`**（不伪造空消息）；
- 角阻尼用线性近似，剧烈旋转时不如二次模型准，但手操判据够用；
- 洋流只做水平分量（`vz = 0`），上升流的耦合未建模。
