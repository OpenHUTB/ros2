# cartpole_swingup — 倒立摆能量摆起 + LQR 平衡

摆杆从自然下垂位置出发：先用**能量控制**给系统泵入能量，把摆杆摆到竖直附近；
进入设定窗口后切换 **LQR** 保持平衡，并把小车拉回原点。订阅
`/cartpole/state`，发布 `/cartpole/force`。配套教程见
[docs/classical_control/cartpole_swingup.md](../../docs/classical_control/cartpole_swingup.md)。

## 运行环境

| 项目 | 配置 |
| ---- | ---- |
| 操作系统 | Ubuntu 20.04（ROS 节点） |
| ROS | ROS Noetic（Python 3） |
| 依赖 | rospy、std_msgs、python3-numpy |
| 离线仿真 | Windows / Linux，Python 3 + numpy + matplotlib（不需要 ROS） |
| 被控对象 | 需要外部提供 `/cartpole/state` 的仿真器或实车驱动 |

## 运行步骤

### 方式一：ROS 节点

```bash
# 1. 拷贝到工作空间并编译
mkdir -p ~/catkin_ws/src
cp -r <本仓库>/src/classical_control/cartpole_swingup ~/catkin_ws/src/
chmod +x ~/catkin_ws/src/cartpole_swingup/main.py
cd ~/catkin_ws && catkin_make
source devel/setup.bash

# 2. 启动一个能发布 /cartpole/state、订阅 /cartpole/force 的倒立摆仿真器
#    例如仓库里的 cartpole_balance 模块中的 cartpole_sim.py

# 3. 启动摆起节点
roslaunch cartpole_swingup main.launch
```

节点启动后会打印 LQR 增益，摆起成功时终端输出模式切换日志
`SWINGUP -> BALANCE`。

### 方式二：离线仿真（不需要 ROS）

```bash
cd <本仓库>
python -m pip install numpy matplotlib
python src/classical_control/cartpole_swingup/offline_sim.py
```

脚本用 RK4 积分非线性动力学，复现同样的控制器，并给出「只用 LQR」的对照组。

## 参数

| 参数 | 默认值 | 说明 |
| ---- | ---- | ---- |
| `~k_energy` | 8.0 | 能量误差到控制力的增益（N/J） |
| `~k_cart_p` / `~k_cart_d` | 0.8 / 0.6 | 摆起阶段小车位置/速度反馈 |
| `~switch_angle` | 0.4 | 切入 LQR 的角度阈值（rad） |
| `~switch_rate` | 2.0 | 切入 LQR 的角速度阈值（rad/s） |
| `~fall_angle` | 0.7 | 超过该角度退回能量摆起（rad） |
| `~control_rate` | 50.0 | 控制频率（Hz） |
| `~state_timeout` | 0.5 | 状态超时（s），超时后控制力归零 |

## 话题接口

| 方向 | 话题 | 类型 | 说明 |
| ---- | ---- | ---- | ---- |
| 订阅 | `/cartpole/state` | `std_msgs/Float64MultiArray` | `[x, x_dot, theta, theta_dot]`，theta 从竖直向上方向量起（rad） |
| 发布 | `/cartpole/force` | `std_msgs/Float64` | 小车水平力（N），向右为正 |

## 预期效果

摆杆从下垂附近开始，约 5 s 摆到竖直附近并切换 LQR，约 10 s 后角度误差小于 0.01°、
小车回到原点附近；最大控制力约 8.7 N，小车最大行程约 0.86 m。
只用 LQR（按竖直平衡点线性化）从下垂位置出发时会在 −1～1 之间持续大幅摆动，
无法稳定在竖直位置。
