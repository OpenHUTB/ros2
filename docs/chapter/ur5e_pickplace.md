# MuJoCo UR5e 关节空间抓放控制

本模块把 MuJoCo 物理仿真与 ROS Noetic 节点解耦，实现一个 5 状态的循环抓放：
复位 → 靠近 → 下降 → 抓取 → 抬升放置 → 再复位。方块从 `(0.4, 0, 0.05)` 被抓取，
搬运到 `(-0.4, 0, 0.05)` 的绿色放置点，释放后复位并开始下一周期。

## 1\. 与参考模块的区别

|模块|环境|自由度|驱动方式|重点|
|-|-|-|-|-|
|`musculoskeletal\_arm\_reach`|ROS 2 Humble + colcon|2|Hill 肌肉激活|双关节肌与逆运动学到达|
|本模块（pick-place）|ROS Noetic + catkin|6|position 伺服（PD 闭环）|关节空间轨迹规划与状态机|

本模块不重复单关节肌肉实验，而是解决：解析 IK、三次多项式平滑轨迹、
freejoint 方块抓取（运动学硬连接）、ROS 节点线程安全与仿真稳定性。

## 2\. ROS 计算图与接口

```mermaid
flowchart LR
    A\["/pick\_fsm<br/>决策节点"] -->|"/joint\_target<br/>Float64MultiArray"| B\["/mujoco\_bridge<br/>仿真节点"]
    B -->|"/joint\_states<br/>JointState"| A
    A -->|"/grasp\_signal<br/>Bool"| B
    A -->|"/cube\_reset<br/>Bool"| B
    B -->|"/joint\_states"| C\["/rviz（可选）"]
```

本模块创建 **2 个 ROS 1 节点**，4 个话题，不使用 service / action：

|话题|类型|方向|内容与发布时机|
|-|-|-|-|
|`/joint\_target`|`std\_msgs/Float64MultiArray`|FSM → Bridge|6 个关节角 `\[pan, lift, elbow, w1, w2, gripper]`，每次轨迹步发布|
|`/joint\_states`|`sensor\_msgs/JointState`|Bridge → FSM/RViz|6 个机械臂关节角与速度，50 Hz，按名字（不受 freejoint 顺序影响）|
|`/grasp\_signal`|`std\_msgs/Bool`|FSM → Bridge|`True` 抓取 / `False` 释放，桥端做距离校验|
|`/cube\_reset`|`std\_msgs/Bool`|FSM → Bridge|每个周期开始把方块复位回抓取点|

启动后检查实时 ROS 计算图：

```bash
rostopic list
rostopic info /joint\_states
rostopic hz /joint\_states
rqt\_graph
```

## 3\. 运行逻辑

启动阶段按以下顺序执行：

1. `main.launch.py`（实际为 `pick\_arm.launch`）启动 `mujoco\_bridge` 与 `pick\_fsm`；
2. bridge 从 `models/ur5e.xml` 加载模型，创建 MuJoCo 状态与被动 viewer；
3. FSM 等待第一帧 `/joint\_states` 后进入 RESET 状态；
4. bridge 以 50 Hz 循环：应用待处理标志（抓取 / 复位）→ `mj\_step` 10 个子步（20 ms）→ 发布关节状态。

每个控制周期：FSM 读取真实关节角，由 `decide()` 决定当前状态下的轨迹点；
目标点通过解析 IK 给出，轨迹用三次多项式插值（起止速度 0）。
抓取前桥端校验末端 `ee\_site` 与方块中心距离 `< 0.10 m`，过远则拒绝并打印告警。

控制器不直接解算肌肉激活：bridge 只把目标关节角写入 `position` 伺服执行器，
实际关节力矩由 MuJoCo 的 PD 闭环（kp/kv）计算。

## 4\. 双关节（6 自由度）模型

模型位于 `models/ur5e.xml`。

|项目|数值|
|-|-|
|基座到肩部高度|0.40 m|
|上臂长度|0.40 m|
|下臂长度|0.35 m|
|工具/末端偏移|0.08 m|
|肩关节（lift）范围|-1.6 \~ 1.6 rad|
|肘关节范围|-2.8 \~ 2.8 rad|
|控制周期|20 ms（5 个 4 ms 子步 = 10 × 2 ms）|
|方块|0.08 m 立方体，mass 0.1 kg，带 `freejoint`|

[5 状态抓放流程画面](../img/pickplace/pickplace_sequence.png)

工作空间内端靠近目标时的 IK 误差 < 1 mm。肩关节限位裁掉了部分扇区。

六个执行器（position 伺服）：

|执行器|kp|kv|
|-|-|-|
|shoulder\_pan / shoulder\_lift / elbow|120 / 150 / 120|15 / 20 / 16|
|wrist\_1 / wrist\_2|50|8|
|gripper|30|4|

关节默认 `armature=0.01, damping=3.0`，连杆显式轻质量（0.05\~0.8 kg），
这是消除高频自激、保持仿真数值稳定的关键参数。

## 5\. 逆运动学与轨迹

末端位姿由两连杆解析 IK 给出（本模型连杆方向为 `(sinθ, cosθ)`，角度映射
`θ1 = π/2 − β1, θ2 = −β2`），六个路点：

|路点|末端 x-z|关节角 (lift, elbow)|
|-|-|-|
|HOME|(0.15, 0.55)|-0.667, 2.63|
|APPROACH|(0.40, 0.20)|0.975, 2.006|
|GRASP|(0.40, 0.12)|1.189, 1.886|
|LIFT|(0.30, 0.45)|0.116, 2.395|
|PLACE\_A|(-0.30, 0.45)|0.116, 2.395（pan 翻转 π）|
|PLACE|(-0.40, 0.12)|1.189, 1.886（pan 翻转 π）|

每个状态内部用三次多项式插值：

```text
q(t) = q0 + (qf-q0)·(3(t/T)² − 2(t/T)³)
```

启停速度为 0，无冲击；到达判据 `max|q\_now − q\_target| < 0.06 rad`
（抓取后阈值放宽到 0.10，避免 weld 力扰动误判）。

## 6\. 构建与运行

```bash
source /opt/ros/noetic/setup.bash
pip3 install --user -r src/pickplace/requirements.txt

mkdir -p \~/catkin\_ws/src
# 把本包放为 \~/catkin\_ws/src/ros\_mujoco\_arm\_pickplace
cd \~/catkin\_ws
catkin\_make
source devel/setup.bash

roslaunch ros\_mujoco\_arm\_pickplace pick\_arm.launch
# 或纯后台不出窗口：
roslaunch ros\_mujoco\_arm\_pickplace pick\_arm.launch viewer:=false
# 或免编译直接跑：
cd \~/ros\_mujoco\_arm\_pickplace \&\& bash start.sh
```

发送一次单步抓取（调试用）：

```bash
rostopic pub -1 /joint\_target std\_msgs/Float64MultiArray \\
  "data: \[0.0, 1.189, 1.886, 0.0, 0.0, 0.0]"
```

## 7\. 测试

```bash
python3 -m pytest src/pickplace/test -q
```

覆盖内容：

* 模型加载与关节 / freejoint / 执行器数量；
* 正运动学与解析 IK 闭环（六个路点误差 < 1 mm）；
* 关节限位裁剪、目标长度校验；
* 抓取距离拒绝（远端不允许 attach）；
* 完整 1 周期状态机：无 NaN、过渡序列 0→1→2→3→4→0；
* 方块被搬运：x 范围覆盖 \[0.4, -0.4]；
* MuJoCo 模型加载与安全步进（qacc 全有限）。

## 8\. 闭环运行结果

默认 2 周期循环下实测：

```text
nan\_count      = 0
transitions    = \[0,1,2,3,4,0,1,2,3,4,0]
cube\_x\_range   = (-0.406, 0.405)
抓取时刻 ee-方块距离 = 0.068 m（阈值 0.10）
IK 六路口径误差      ≈ 0 m
```

机械臂平滑完成 复位→靠近→下降→抓取→抬升→搬运→下降→释放→复位，无超调崩溃。

## 9\. 当前边界

* 抓取采用"运动学硬连接"（每子步把方块位姿锁定到末端参考系），
不依赖 MuJoCo `<equality>` weld 约束（3.17 实测其约束雅可比不生效）；
* 夹爪开合仅为视觉装饰，不参与物理夹持；
* 所有路点为预设关节空间轨迹，无在线采样 IK / 无碰撞路径规划；
* 尚未加入力控、视觉感知与双臂协同。

这些内容分别进入后续"柔性夹爪 / 视觉抓取 / 双臂"提交，避免一次提交同时修改
模型、物理与控制造成难以验收。

