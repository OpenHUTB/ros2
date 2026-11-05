# diff_drive_odom_calib

ROS 1（Noetic）差速机器人的**里程计标定 + 神经网络滑移补偿**。
在 **MuJoCo** 里跑一台会打滑的车，先用两个解耦实验辨识线性误差，
再用 MLP 学掉"转向越大、滑移越大"的非线性部分。

## 与仓库已有模块的区别

| 模块 | 体系 | 任务 | 是否处理里程计误差 |
|---|---|---|---|
| `ground/mujoco_diff_drive_slip_compensation` | ROS 2 + MuJoCo | 在线补偿轮地滑移 | 是（在线） |
| `ground/turtlebot3_auto_nav` | ROS 2 + Gazebo | Nav2 参数调优 | 否（假设里程计可信） |
| `air/uav_keyboard_control` | ROS 1 catkin | 无人机键盘遥控 | 否 |
| **本模块** | **ROS 1 catkin + MuJoCo** | **离线标定 + 学习式补偿** | **是（离线 + 学习）** |

## 目录结构

```
├── models/diff_drive.xml          MuJoCo 场景（两驱动轮 + 一个万向球，真实摩擦接触）
├── scripts/
│   ├── main.py                    标定主节点：直行 + 旋转实验 → 三个系数 → 方形验证
│   ├── sim_node.py                仿真节点：/odom、/wheel_ticks、/imu、/ground_truth、TF
│   ├── teleop.py                  键盘遥控（termios，W/S/A/D，无 GUI 依赖）
│   └── demo_drive.py              自动跑方形路径并打印里程计 vs 真值的偏差
├── src/diff_drive_odom_calib/
│   ├── mujoco_env.py               MuJoCo 环境封装（接触、轮速、IMU、渲染）
│   ├── model.py                   里程计积分器（纯数学，不含仿真）
│   ├── calibration.py             三个系数的闭式解
│   ├── slip_nn.py                 MLP（4→24→24→2，纯 NumPy 手写反传）
│   └── config.py                  路径/几何/参数 + 渲染后端引导
├── launch/{main,sim,demo}.launch
├── tools/{check_physics,run_experiment,make_gif,demo_drive}.py
└── config/calib.yaml
```

## 运行环境

| 组件 | 版本（实测） |
|---|---|
| 系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机，无 GPU） |
| ROS | Noetic（ROS 1） |
| Python / numpy | 3.8.10 / 1.17.4 |
| MuJoCo | 3.2.3（渲染用 `MUJOCO_GL=osmesa` 或本机 GL） |
| 依赖 | rospy、std_msgs、geometry_msgs、nav_msgs、sensor_msgs、tf2_ros、numpy、PIL |

## 编译与运行

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
ln -s <仓库路径>/src/ground/diff_drive_odom_calib .
cd ~/catkin_ws && catkin_make && source devel/setup.bash

# ① 一条命令跑完标定
roslaunch diff_drive_odom_calib main.launch

# ② 键盘遥控（另开终端）
roslaunch diff_drive_odom_calib sim.launch
rosrun  diff_drive_odom_calib teleop.py

# ③ 不启动 ROS 也能跑的三件事
python3 tools/check_physics.py            # 物理自检（滑移/接触/姿态）
python3 tools/run_experiment.py           # 标定 + 训练 NN + 三组对比
python3 tools/make_gif.py                 # 渲染方形路径并合成 GIF
```

## 原理

**误差从哪来**：里程计用标称 $r, b$ 积分编码器，而真实轮径/轮距有偏差；
转向时轮胎还要横向刮擦，滑移率随转速上升 —— 后者是常数系数拟合不了的。

**三个系数**（闭式解，推导见文档第 4 节）：

$$
ds = c_s\, r\, \frac{\dot\phi_l+\dot\phi_r}{2}dt, \qquad
d\theta = c_\theta \frac{r(\dot\phi_r-\dot\phi_l)}{\hat b}dt + k_h\, ds
$$

| 系数 | 含义 | 辨识实验 |
|---|---|---|
| $c_s$ | 平均轮径 + 纵向滑移 | 直行 2 m |
| $k_h$ | 直行航向漂移率 | 直行 2 m |
| $c_\theta$ | 轮距 + 转向滑移 | 原地转 3 圈 |

**为什么还要神经网络**：实测滑移随转向率从 5.3% 涨到 17.2%，
$c_\theta$ 只能取一个平均值。MLP 用轮速与 IMU 作输入、输出补偿系数，
专门拟合这部分非线性。

## 实测结果（Ubuntu 20.04 + Python 3.8.10 + mujoco 3.2.3）

| 项目 | 数值 |
|---|---|
| 稳态纵向滑移 | 4.7%~5.4% |
| 转向损失 | 4%~8% |
| 滑移随转向率 | 5.3% → 7.5% → 11.8% → 17.2%（ω = 0 → 1.2） |
| 方形路径 4×4 m（14.2 m） 原始里程计 | 2.914 m / 72.6° |
| 　+ 参数标定 | 1.937 m / 46.9° |
| 　+ 神经网络 | **0.79 ~ 1.34 m / −20° ~ −31°**（对随机种子敏感，见文档 §6.3） |
| MLP 验证集 MSE | 0.877 → 0.066 |

## 已知限制

- **航向残余误差 20°~30°**：$c_\theta$ 把转向滑移当轮距误差吸收了，
  一次 90° 转角偏 4% 左右，四次累积就到这个量级。要根治得改用
  "IMU 角速度直接标定轮距 + 互补滤波"，本版没做。
- **终点绝对误差 1.3 m 量级**，距离"能用于导航"还差得远。
- 未做导航与 SLAM；感知只用了编码器与 IMU，没有雷达/相机。
- 仿真到实车未验证。

## 版本记录

| 版本 | 改动 | 现象 / 原因 |
|---|---|---|
| 1.0.0 | 初版：纯解析运动学 | 被指出"用六行积分冒充物理仿真、没有神经网络" |
| 2.0.0 | 换成 MuJoCo 真物理 + MLP 补偿 | 见下 |
| 2.0.1 | 万向球从两个改成一个 | 两个球分走载荷，驱动轮压不住地，打滑 78% |
| 2.0.2 | 轮子碰撞体从圆柱改球形（圆柱仅作外观） | 圆柱-平面是线接触，粘滑振荡，滑移在 7%~80% 乱跳 |
| 2.0.3 | 标定丢弃起步 1.5 s | 起步滑移 30~45%，c_s 被拉低到 0.854 |
| 2.0.4 | NN 标签改用转向工况采样 | 直线段 dθ_odom≈0，比值爆炸 |
| 2.0.5 | 所有文本文件转 LF | Windows 写出 CRLF，GitHub diff 每行带 ^M、整篇算重写 |
