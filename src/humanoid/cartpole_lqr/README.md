# cartpole_lqr

倒立摆（Cart-Pole）的 LQR 最优平衡控制教学模块。

通过拉格朗日建模倒立摆、在竖直平衡点线性化、迭代求解离散 Riccati 方程得到
LQR 增益 $K$，闭环把倒立摆稳定在竖直位置，并与无控（自由倒下）对照。

## 两种运行方式

### 一、ROS 2 节点（能看到真实话题链路）

```bash
cd <ros2_ws>
colcon build --packages-select cartpole_lqr
source install/setup.bash

ros2 launch cartpole_lqr main.launch.py            # 推荐
# 或者直接跑可执行文件
ros2 run cartpole_lqr ros_node.py
# 对照组：不做控制，摆杆自由倒下
ros2 launch cartpole_lqr main.launch.py use_lqr:=false
```

话题接口：

| 方向 | 话题 | 类型 | 内容 |
|---|---|---|---|
| 发布 | `/cartpole/joint_states` | `sensor_msgs/JointState` | `name=[cart_position, pole_angle]`，`position=[x, theta]`，`velocity=[x_dot, theta_dot]` |
| 发布 | `/cartpole/force` | `std_msgs/Float64` | LQR 解出的小车推力 $u$ (N) |
| 订阅 | `/cartpole/force_cmd` | `std_msgs/Float64` | 外部接管推力，收到后 1 s 内有效，过期回落到 LQR |

观察与交互：

```bash
ros2 topic echo /cartpole/joint_states     # 看摆角逐步收敛到 0
ros2 topic echo /cartpole/force            # 看控制力
ros2 topic pub --once /cartpole/force_cmd std_msgs/Float64 "{data: 5.0}"   # 手动推一把
```

节点参数：`use_lqr`（bool）。也可以从命令行覆盖：

```bash
ros2 run cartpole_lqr ros_node.py --ros-args -p use_lqr:=false
```

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/humanoid/cartpole_lqr
python3 scripts/main.py --outdir scripts
python3 scripts/ros_node.py --offline --outdir scripts
```

运行后在 `scripts/` 下生成配图 PNG 并打印关键指标。未安装 `rclpy` 时
`ros_node.py` 会自动降级为离线模式，不会报错。

## 测试

```bash
cd src/humanoid/cartpole_lqr
python3 -m unittest discover -s tests -t .   # test_cartpole.py 7 项 + test_ros_params.py 3 项
```

## 依赖

- Python 3 标准库（无 numpy / matplotlib / scipy）
- ROS 2 可选：`rclpy`、`std_msgs`、`sensor_msgs`

## 关于验证范围（请评审注意）

- 离线仿真的数值（LQR 增益、末端摆角、峰值控制力）已在本机实跑，结果见文档。
- ROS 2 节点的话题名、消息字段与控制律，已用与 `rclpy` 接口一致的桩在无 ROS 环境下
  逐项验证（发布条数、`JointState.name`、外部接管生效等）。
- 节点已 `declare_parameter` 声明并读回全部参数
  （`use_lqr`/`theta0`/`rate`/`dt`），launch 的 `use_lqr:=...` 真正生效。
  非默认参数运行记录（桩上回放）：`use_lqr:=false rate:=20.0` 时定时器周期 0.050 s、
  10 s 摆角增大到 3.142 rad（自由倒下）；`theta0:=0.2` 时 10 s 回稳到
  -0.00032 rad，已写成 `tests/test_ros_params.py` 回归测试。
- **ROS 2 节点尚未在安装了 ROS 2 发行版（Humble / Jazzy）的机器上实测**，
  `colcon build` 与 `ros2 launch` 的具体输出请以目标环境为准。
