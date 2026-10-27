# diff_drive_odom_calib

ROS 1（Noetic）差速移动机器人**里程计标定**：用两段专门的实验分别辨识
"路程系数 / 直行漂移率 / 转角系数"，把方形路径上的航迹漂移从**米级**压到**厘米级**。

纯 `rospy + numpy`，**不需要 Gazebo、不需要 GPU**，`roscore` 起来就能跑。

## 与仓库已有模块的区别

| 模块 | 体系 | 任务 | 是否涉及里程计标定 |
|---|---|---|---|
| `src/ground/mujoco_diff_drive_slip_compensation` | ROS 2 + MuJoCo | 实时补偿轮地滑移 | 否（在线补偿，不是离线标定） |
| `src/ground/turtlebot3_auto_nav` | ROS 2 + Gazebo | Nav2 参数调优 | 否（假设里程计已可信） |
| `src/chap7/smartcar_control` | ROS 1 教材 | 阿克曼车运动控制 | 否 |
| **本模块** | **ROS 1（catkin）** | **用实验辨识 r / b 的系统误差** | **是** |

关键差别：**里程计标定是"导航之前那一步"**。上面那些模块都在假设"里程计是准的"的前提下
做避障、导航、补偿；而真实机器人出厂时轮径和轮距都会有 1%~3% 的偏差，
这一段不标定，后面所有基于航迹推算的功能都会带着米级漂移跑。
仓库里没有任何模块处理这件事。

## 目录结构

```
src/ground/diff_drive_odom_calib/
├── package.xml / CMakeLists.txt / setup.py   catkin 包（ROS 1）
├── scripts/
│   ├── main.py          标定主节点（--selftest 可脱离 rosrun 自检）
│   └── sim_robot.py     仿真机器人节点（发布 /odom、/wheel_ticks、/ground_truth、TF）
├── src/diff_drive_odom_calib/
│   ├── model.py         差速运动学 + 里程计积分器（纯逻辑）
│   └── calibration.py   两个实验组织 + 参数辨识 + 评估
├── config/
│   ├── sim_robot.yaml   仿真机器人的真值参数（真实机器人上是未知量）
│   └── calib.yaml       标定实验参数
├── launch/main.launch   一条命令跑完整个标定
└── test/test_calibration.py   9 项纯逻辑单元测试
```

## 运行环境

| 组件 | 版本（实测环境） |
|---|---|
| 操作系统 | Ubuntu 20.04.6 LTS（VMware 虚拟机） |
| ROS | **ROS Noetic（ROS 1）** |
| Python | 3.8.10 |
| 依赖 | `rospy`、`std_msgs`、`geometry_msgs`、`nav_msgs`、`tf2_ros`、`numpy` |
| 显卡 | 不需要（不涉及渲染） |

## 编译与运行

```bash
# 1) 放进 catkin 工作空间
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
ln -s <本仓库路径>/src/ground/diff_drive_odom_calib .

# 2) 编译
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 3) 一条命令跑完：仿真机器人 + 直行实验 + 旋转实验 + 方形路径验证
roslaunch diff_drive_odom_calib main.launch
```

标定节点是一次性任务：跑完自动打印对照表并把参数写到
`src/ground/diff_drive_odom_calib/config/calib_result.yaml`，然后退出
（`main.launch` 里该节点设了 `required="true"`，所以整个 launch 会随之关停，这是预期行为）。

不装 ROS 也能验证算法本身：

```bash
python3 scripts/main.py --selftest      # 纯逻辑自检，不需要 roscore
python3 -m unittest discover -s test -v # 9 项单元测试
```

## 原理

### 1. 误差从哪来

里程计积分用的永远是**标称**参数，而机器人实际的轮径、轮距与标称值有偏差：

$$
ds = \frac{r(\dot\phi_l + \dot\phi_r)}{2}\,dt, \qquad
d\theta = \frac{r(\dot\phi_r - \dot\phi_l)}{b}\,dt
$$

- 左右轮半径不等 $r_l \ne r_r$ → 直行时航向持续漂（每米漂 $k$ rad）
- 轮距标定偏小/偏大 $b \ne \hat b$ → 每次转角都按比例差

### 2. 三个待辨识系数

给里程计积分器加三个可标定系数：

$$
ds = c_s\, r\, \frac{\dot\phi_l+\dot\phi_r}{2}dt, \qquad
d\theta = c_\theta \frac{r(\dot\phi_r-\dot\phi_l)}{\hat b}dt + k\, ds
$$

| 系数 | 物理含义 | 由哪个实验辨识 |
|---|---|---|
| $c_s$ | 平均轮径修正 | 直行实验：$\Sigma ds_{true} / \Sigma ds_{odom}$ |
| $k$ | 直行航向漂移率 (rad/m) | 直行实验：$\Delta\theta_{true} / \Sigma ds_{true}$ |
| $c_\theta$ | 轮距修正 | 旋转实验：$\Delta\theta_{true} / \Delta\theta_{odom}$ |

### 3. 为什么分两个实验，而不是 UMBmark 的单条方形路径

UMBmark 用一次双向方形路径同时辨识两个误差，实验紧凑；但两个误差在**同一个终点误差里耦合**，
解算要靠原论文的近似公式，不容易讲清"为什么这么算"。
分步标定的好处是：直行实验里 $\dot\phi_l=\dot\phi_r$，转角公式退化成 0，
**转角误差完全不出现**；旋转实验里 $ds\approx0$，$k$ 项也不出现。
两个实验各解一个方程，物理含义一目了然。

## 参数

| 参数（`config/sim_robot.yaml`） | 默认 | 含义 |
|---|---|---|
| `r_left` / `r_right` | 0.0505 / 0.0495 m | 左右轮真实半径（真值，机器人自己不知道） |
| `b_true` | 0.306 m | 真实轮距（标称 0.300） |
| `slip_std` | 0.004 | 打滑扰动，非系统误差的来源 |
| `rate_hz` | 50 | 仿真与话题发布频率 |

| 参数（`config/calib.yaml`） | 默认 | 含义 |
|---|---|---|
| `straight_distance` / `straight_speed` | 2.0 m / 0.2 m/s | 直行实验 |
| `rotation_turns` / `rotation_speed` | 3 圈 / 1.0 rad/s | 旋转实验 |
| `square_side` | 4.0 m | 验证用方形路径边长 |

## 实测结果

在 Ubuntu 20.04 + ROS Noetic 虚拟机里 `catkin_make` 后 `roslaunch` 跑通（详见 `docs/ground/diff_drive_odom_calib.md`）：

| 项目 | 值 |
|---|---|
| 辨识 c_s | 0.99978（真值 1.00000） |
| 辨识 k_head | −0.06463 rad/m（真值 −0.06536） |
| 辨识 c_theta | 0.98031（真值 0.98039） |
| 方形路径 16 m，**标定前** | 位置误差 **2.285 m**（占路径 14.3%），航向误差 **+52.71°** |
| 方形路径 16 m，**标定后** | 位置误差 **0.030 m**（占路径 0.2%），航向误差 **+0.57°** |
| 单元测试 | 9 项全部通过 |

## 版本记录（都来自虚拟机实跑复现）

| 版本 | 修的问题 | 现象 |
|---|---|---|
| 1.0.0 | 初版 | — |
| 1.0.1 | 旋转实验写成"正半圈 + 反半圈"以求抵消起始朝向 | 总转角互相抵消成 0，$c_\theta$ 辨识成兜底值 1.000，标定后航向仍残留 7.2°（正好是 4 次 90° 各差 2% 的累积）。改成单向转 3 圈 |
| 1.0.2 | `main.launch` 用 `<node ns="sim_robot">` + 整份 yaml 加载 | 节点的私有参数落在 `/sim_robot/sim_robot/r_left`，与 yaml 里的 `sim_robot: r_left` 不匹配，节点读到的全是默认值。改成**两份扁平 yaml + 节点私有命名空间** |
| 1.0.3 | 标定节点在 launch 里没设 `required` | 标定跑完节点正常退出，但 launch 会把它当成异常继续挂着，用户以为卡住了。设 `required="true"` 让整条 launch 一起收工 |

## 常见问题

**Q1：`roslaunch` 报了 `REQUIRED process [odom_calib] has died`**
这是**正常**的：标定节点是一次性任务，跑完就退出，`required="true"` 会让整个 launch 收工。
看它退出前打印的对照表即可。

**Q2：节点一直打印"等待仿真机器人发布 /ground_truth 与 /wheel_ticks"**
说明 `sim_robot.py` 没起来。单独起一下：`rosrun diff_drive_odom_calib sim_robot.py`。

**Q3：`ImportError: No module named diff_drive_odom_calib`**
没有 `source devel/setup.bash`。本包用 `catkin_python_setup()` 安装 Python 模块，
必须 source 工作空间后才能 import。

**Q4：想用标定结果**
`config/calib_result.yaml` 里的三个系数直接写进你自己的里程计积分器即可；
`src/diff_drive_odom_calib/model.py` 的 `OdometryIntegrator` 就是参考实现。

## 已知限制

- **只标系统误差**：打滑、地面不平等**非系统误差**标定消不掉，只能靠多次重复实验统计散布
  （`test/test_calibration.py::TestRepeatability` 里对这一点有断言）。
- **仿真用的是运动学模型**，不含轮地接触动力学；真实的轮径误差还会随载重、胎压变化，
  实车需要定期复标。
- **真值由仿真提供**：真实系统里这一步要靠卷尺/全站仪人工测量（UMBmark 原始做法），
  本模块把"外部测量"抽象成 `/ground_truth` 话题，接口位置一致，替换成人工输入即可。
