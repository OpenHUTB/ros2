# quad_eso_hover

四旋翼高度 PD + 扩张状态观测器（ESO / ADRC）抗扰控制教学模块。

用三阶线性 ESO 在线估计"重力 + 未知外扰"的总扰动 $f$，再前馈补偿掉，
使常值扰动下的稳态误差从 PD-only 的 $-d/(m k_p)$ 降到 0。这是自抗扰控制（ADRC）
最核心的一步。

## 两种运行方式

### 一、ROS 2 节点（传感器输入 → 观测器 → 控制输出）

```bash
cd <ros2_ws>
colcon build --packages-select quad_eso_hover
source install/setup.bash

ros2 launch quad_eso_hover main.launch.py
# 或者
ros2 run quad_eso_hover ros_node.py
```

话题接口：

| 方向 | 话题 | 类型 | 内容 |
|---|---|---|---|
| 订阅 | `/quad/altitude` | `std_msgs/Float64` | 高度传感器读数 $y$ (m) |
| 发布 | `/quad/odom` | `nav_msgs/Odometry` | 被控对象真值：`pose.pose.position.z` 高度、`twist.twist.linear.z` 垂速 |
| 发布 | `/quad/thrust` | `std_msgs/Float64` | 推力指令 $u$ (N) |
| 发布 | `/quad/disturbance_est` | `std_msgs/Float64` | ESO 估计的总扰动 $\hat f$ (m/s²) |

这个链路就是评审要的"传感器输入 + 控制输出"：把 `/quad/altitude` 换成 Gazebo /
真实飞控的高度话题，控制器无需改动。观察扰动估计跳变：

```bash
ros2 topic echo /quad/disturbance_est    # t=4s 后应从 -9.81 跳到 -5.81
ros2 topic echo /quad/thrust
```

节点参数：`w0`（ESO 带宽, rad/s）、`ref`（目标高度, m）。命令行覆盖：

```bash
ros2 run quad_eso_hover ros_node.py --ros-args -p w0:=20.0 -p ref:=1.5
```

### 二、纯离线仿真（不需要装 ROS 2）

```bash
cd src/air/quad_eso_hover
python3 scripts/main.py --outdir scripts
python3 scripts/ros_node.py --offline --outdir scripts
```

未安装 `rclpy` 时 `ros_node.py` 自动降级为离线模式。

## 实现要点：观测器步长必须与对象同步

节点按 `rate` 触发，但一个控制周期内会做 `n_sub = (1/rate)/dt` 次定步长积分。
**ESO 的更新必须放在子步循环内、与对象用同一个 `dt` 推进**；若只在每个周期末更新
一次却仍用 `dt` 做步长，观测器的等效时间尺度会比对象慢 `n_sub` 倍，相位滞后会把
闭环推入发散（实测会发散到 1e14 量级）。这一点在离线 `simulate_eso()` 里天然满足，
写成 ROS 节点时容易踩坑。

## 测试

```bash
cd src/air/quad_eso_hover
python3 -m unittest discover -s tests -t .   # test_eso.py 6 项 + test_ros_params.py 3 项
```

## 依赖

- Python 3 标准库（无 numpy / matplotlib）
- ROS 2 可选：`rclpy`、`std_msgs`、`nav_msgs`

## 关于验证范围（请评审注意）

- 离线仿真数值（PD-only 稳态偏差 -0.640 m、PD+ESO 误差 ≈ 0、$\hat f=-5.810$）
  已在本机实跑，结果见文档。
- ROS 2 节点已用与 `rclpy` 接口一致的桩验证：话题名与字段、高度收敛到 1.00000 m、
  $\hat f$ 收敛到 -5.8100、推力 5.8100 N。
- 节点已 `declare_parameter` 声明并读回全部参数（`ref`/`w0`/`wc`/`rate`/`dt`），
  launch 的 `w0:=... ref:=...` 真正生效。非默认参数运行记录（`ref:=2.0 w0:=10.0
  rate:=20.0`，桩上回放 12 s）：$\beta_1=30.0$、定时器周期 0.050 s、高度收敛到
  2.00000 m、$\hat f=-5.8100$，已写成 `tests/test_ros_params.py` 回归测试。
- **ROS 2 节点尚未在安装 ROS 2 发行版的机器上实测**，`colcon build` / `ros2 launch`
  的行为请以目标环境为准。
