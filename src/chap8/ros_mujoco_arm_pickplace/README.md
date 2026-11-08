# ros_mujoco_arm_pickplace

基于 **ROS Noetic + MuJoCo** 的 UR5e 简易机械臂 5 状态抓放演示：

```
复位(RESET) -> 靠近(APPROACH) -> 下降(DESCEND) -> 抓取(GRASP)
                                          -> 抬升放置(LIFT_PLACE) -> 循环
```

- 方块：0.08m 立方体，带 `freejoint`，初始在 `(0.4, 0, 0.05)`；放置目标在 `(-0.4, 0, 0.05)`（绿色 marker）。
- 抓取：**运动学硬连接**（每控制周期把方块位姿锁定到末端执行器 + 清速），跨 MuJoCo 版本稳定、无 NaN；XML 中保留 `grasp_weld` 约束（`active="false"`）供参考，代码不依赖约束求解（实测 mujoco 3.17.0 的 equality 约束雅可比异常）。
- 运动：关节空间三次多项式插值（起止速度 0）+ 解析 IK（双分支闭式解，末端误差 < 1mm）。
- 分层：`pickplace_planner.py`（纯 numpy 决策：IK / 轨迹 / 状态机）与 `pickplace_core.py`（MuJoCo 物理）分开，**ROS 状态机节点不加载模型**。

---

## 1. 环境依赖（Ubuntu 20.04 + ROS Noetic）

```bash
# ROS Noetic 自带 rospy / sensor_msgs / std_msgs
source /opt/ros/noetic/setup.bash
cd src/chap8/ros_mujoco_arm_pickplace
pip3 install --user -r requirements.txt      # mujoco / numpy / pytest
# 可选：想看 MuJoCo 原生窗口
sudo apt install -y python3-tk libglfw3 libgl1-mesa-glx
```

> 不用 conda。脚本全部 `#!/usr/bin/env python3`，直接 `python3` 运行。

## 2. 快速验证物理核心（无 ROS，可先在任意机器跑）

```bash
cd src/chap8/ros_mujoco_arm_pickplace
python3 scripts/pickplace_core.py
# 输出：6 个路点 IK 误差 ≈ 0、2 周期完整循环、nan_count=0、方块 x 轨迹 0.4 -> -0.4
```

只验证运动学（不需要装 mujoco）：

```bash
python3 scripts/pickplace_planner.py     # 六个路点 IK 往返误差（mm）
```

## 3. 测试

```bash
cd src/chap8/ros_mujoco_arm_pickplace
python3 -m pytest test -q
```

- `test/test_pickplace_planner.py`：IK / 限位 / 1~2 周期状态链（无需 mujoco）
- `test/test_pickplace_core.py`：模型结构 / IK 与模型 FK 对账 / 复位 / 抓取拒绝 / 整周期无 NaN（需 mujoco，未装则整文件 skip）

## 4. ROS 方式一：catkin 编译 + roslaunch

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
# 把本项目放到 src/ros_mujoco_arm_pickplace
cd ~/catkin_ws
catkin_make
source devel/setup.bash
roslaunch ros_mujoco_arm_pickplace pick_arm.launch
# 纯后台（不弹 MuJoCo 窗口）
roslaunch ros_mujoco_arm_pickplace pick_arm.launch viewer:=false
```

## 5. ROS 方式二：不编译，直接一键启动

```bash
cd src/chap8/ros_mujoco_arm_pickplace
bash start.sh        # 自动开 roscore + bridge + FSM 三个终端
```

或手动：

```bash
# 终端1
roscore
# 终端2
python3 scripts/main_mujoco_bridge.py
# 终端3
python3 scripts/main_pick_fsm.py
```

## 6. 话题一览

| 话题 | 类型 | 方向 | 说明 |
|---|---|---|---|
| `/joint_target` | `std_msgs/Float64MultiArray` | FSM -> Bridge | 6 个关节角目标 `[pan, lift, elbow, wrist1, wrist2, gripper]` |
| `/joint_states` | `sensor_msgs/JointState` | Bridge -> FSM/RViz | 按关节名字发布（不受 freejoint 顺序影响） |
| `/grasp_signal` | `std_msgs/Bool` | FSM -> Bridge | `True` 抓取 / `False` 释放（桥端做距离校验） |
| `/cube_reset` | `std_msgs/Bool` | FSM -> Bridge | 每个循环开始把方块复位回抓取点 |

## 7. 文件结构

```
ros2
├── docs/
│   ├── chapter/
│   │   └── ur5e_pickplace.md        # 本模块教程（mkdocs 第 8 章导航项）
│   └── img/
│       ├── arm_pickplace.png        # 5 状态抓放流程画面
│       └── arm_pickplace_demo.gif   # 抓放循环动图
└── src/
    └── chap8/
        └── ros_mujoco_arm_pickplace/
            ├── launch/
            │   └── pick_arm.launch        # 一键 roslaunch（bridge + FSM，viewer/RViz 可选）
            ├── models/
            │   └── ur5e.xml               # MuJoCo 模型：freejoint 方块 + position 伺服 + weld 参考约束
            ├── rviz/
            │   └── ur5e.rviz              # 可选 RViz 配置（需自备 URDF 才能显示模型）
            ├── scripts/
            │   ├── pickplace_planner.py   # ★ 决策层：解析 IK / 三次插值 / 5 状态机（纯 numpy）
            │   ├── pickplace_core.py      # ★ 物理层：模型加载 / mj_step / 运动学抓取 / 整周期跑测
            │   ├── cubic_interpolator.py  # 三次多项式轨迹（起止速度 0）
            │   ├── main_mujoco_bridge.py  # ROS 仿真桥（线程安全：回调只置标志，主循环执行）
            │   └── main_pick_fsm.py       # ROS 状态机（只做决策，不加载 MuJoCo 模型）
            ├── test/
            │   ├── conftest.py
            │   ├── test_pickplace_planner.py
            │   └── test_pickplace_core.py
            ├── requirements.txt
            ├── start.sh                   # 免编译一键启动
            ├── package.xml
            ├── CMakeLists.txt
            └── README.md
```

## 8. 关键实现说明

- **关节角约定**：本模型连杆方向为 `(sinθ, cosθ)`，IK 用标准两连杆角 `β = π/2 - θ` 映射，`θ1=π/2−β1, θ2=−β2`。所有关节角按**名字**读写 `jnt_qposadr / jnt_dofadr`（mujoco 3.x 无 `jnt_qveladr`）。
- **稳定性**：连杆显式轻质量（避免默认密度导致重臂 + 高频自激）；关节 `armature=0.01, damping=3.0`；position 伺服 `kp 120/150/120/50/50/30, kv 15/20/16/8/8/4`；`timestep=0.002`。实测全程无 `NaN/Inf`。
- **线程安全**：bridge 的 ROS 回调只写入 `core.target_q` / `core._pending_weld` / `_pending_reset`，物理状态全部在主循环内修改。
- **IK 不污染状态**：`ik()` 是纯计算（不含任何 `mj_forward`），轨迹从真实当前位置起步，杜绝"瞬移"。
- **决策与物理分离**：`main_pick_fsm.py` 只 import `pickplace_planner`，不为算 IK 加载整份 XML；`test_pickplace_planner.py` 里有一条用例专门断言 planner 不会间接导入 mujoco。
- **方块复位**：循环演示要求每个周期方块回到抓取点，`RESET` 状态进入时通过 `/cube_reset` 让桥端复位（无界面模式下就地执行）。复位请求在 `_move()` **之前**判定，否则第二周期会抓空。

## 9. 常见问题

- **`module 'mujoco' has no attribute 'mjOBJ_xxx'`**：mujoco 版本过低（<2.3.3）；请 `pip3 install -U mujoco`（≥2.3.3）。
- **`XML Error: unrecognized element light`**：`<light>` 必须放在 `<worldbody>` 内，且 mujoco ≥ 3.0 不接受根级/`<visual>` 内 light。
- **`jnt_qveladr` 不存在**：3.x 已改名 `jnt_dofadr`，本项目已按新版 API 编写。
- **方块不跟随机械臂**：确认 `grasp_signal` 已发布且桥端日志出现 `cube attached`；若被 `rejected! ee-cube dist>0.1` 拒绝，说明未到位（等待 `RESET/APPROACH/DESCEND` 走完再抓）。
- **看不到 MuJoCo 窗口**：`echo $DISPLAY` 是否为空；`viewer:=false` 只是关窗口不影响仿真。

## 10. AI 使用声明

本模块的代码、模型、文档与测试在编写过程中使用豆包（Doubao）辅助生成与调试；
全部结论来自 Ubuntu 20.04 + ROS Noetic + MuJoCo 环境的实际运行，提交者已审阅并对内容负责。
