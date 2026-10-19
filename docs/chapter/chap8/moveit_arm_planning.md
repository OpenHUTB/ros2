# MoveIt 六自由度机械臂运动规划与 OMPL 规划器对比

> 状态：初稿。代码、截图和实测数据待补；标有“待实测”的单元格在正式提交前必须替换为真实结果。正式 PR 前删除本行。

本实验在 ROS Noetic 下使用 MoveIt 1 对六自由度机械臂进行运动规划，覆盖正运动学、逆运动学、笛卡尔路径、障碍物避让和抓取五类任务，并在同一组目标位姿上比较 OMPL 的 RRTConnect、RRT*、PRM 三种规划器。实验只使用 MoveIt 的 fake controller 和 RViz 可视化，**不启动 Gazebo、MuJoCo 或任何物理引擎**。

## 1. 实验目的

- 理解 MoveIt 的规划场景、规划组、末端执行器和 fake execution。
- 掌握 FK、IK、笛卡尔路径和抓取的基本调用方式。
- 用统一指标比较不同 OMPL 规划器：成功率、规划耗时、路径长度、末端误差。
- 给出可复现的命令、配置和结果表，方便其他同学在同一台虚拟机上复现。

## 2. 与已有机械臂模块的区别

| 模块 | 自由度 | 仿真/框架 | 主要内容 |
| --- | --- | --- | --- |
| 单自由度机械臂模块 | 1 | Gazebo + PID | 单关节角度自动演示 |
| MuJoCo two dof arm | 2 | MuJoCo | 两关节摆动，熟悉 MuJoCo |
| 本模块 | 6 | MoveIt 1 + RViz fake execution | FK/IK/笛卡尔/抓取 + OMPL 规划器对比 |

本模块不涉及 URDF 建模、关节 PID 整定和物理引擎，重点是 MoveIt 的运动规划链路和规划器性能对比。

## 3. 环境与依赖

| 项目 | 配置 |
| --- | --- |
| 操作系统 | Ubuntu 20.04 |
| ROS | Noetic |
| MoveIt | MoveIt 1（`ros-noetic-moveit`） |
| 可视化 | RViz |
| Python | Python 3 |
| 物理引擎 | 无（fake execution，不启动 Gazebo） |

安装依赖：

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-joint-state-publisher \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro
```

## 4. 目录结构

本模块复用 `src/chap8` 下已有的机械臂模型和 MoveIt 配置，新增一个实验封装包。计划结构如下：

```text
src/chap8/
├── marm_description/        # 机械臂 URDF/xacro（已有）
├── marm_moveit_config/      # MoveIt 配置（已有，需做 Noetic 兼容修复）
├── marm_planning/           # 原有 FK/IK/笛卡尔/抓取 demo（已有）
└── moveit_arm_planning/     # 本模块新增
    ├── launch/
    │   ├── main.launch      # 一键启动 MoveIt + RViz
    │   └── eval.launch      # 批量规划器对比
    ├── config/
    │   ├── targets.yaml     # 目标位姿列表
    │   └── planners.yaml    # OMPL 规划器与重复次数
    ├── scripts/
    │   ├── plan_eval.py     # 批量规划与指标记录
    │   └── pick_place_task.py
    ├── CMakeLists.txt
    ├── package.xml
    ├── main.sh
    └── README.md
```

## 5. Noetic 兼容修复

`src/chap8` 里的机械臂包来自较早的 ROS 1 版本，在 Noetic 下需要做两处小修复。

### 5.1 去掉 `xacro --inorder`

`marm_moveit_config/launch/planning_context.launch` 当前写法：

```xml
<param name="$(arg robot_description)"
       command="$(find xacro)/xacro --inorder '$(find marm_description)/urdf/arm_gazebo.xacro'"/>
```

Noetic 的 xacro 已删除 `--inorder` 选项，改为：

```xml
<param name="$(arg robot_description)"
       command="$(find xacro)/xacro '$(find marm_description)/urdf/arm_gazebo.xacro'"/>
```

### 5.2 修正 `state_publisher`

`marm_description/launch/view_arm.launch` 里使用了旧节点名：

```xml
<node name="robot_state_publisher" pkg="robot_state_publisher" type="state_publisher" />
```

改为：

```xml
<node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" />
```

同时删除该文件中未使用的 `<arg name="model" />`，或者给它一个默认值，避免启动时提示参数未设置。

> URDF 中的 `<gazebo>` 标签只是模型描述，RViz 和 MoveIt 会忽略它们；本实验不会启动 Gazebo 进程。

### 5.3 脚本可执行位

`src/chap8/marm_planning/scripts/` 下的五个 demo 在仓库里是 `100644`，没有可执行位。直接 `rosrun` 会报 `Permission denied`。进入工作空间后先补上：

```bash
chmod +x ~/catkin_ws/src/chap8/marm_planning/scripts/*.py
```

本模块的 `main.sh` 和 `eval.launch` 统一用 `python3` 调用脚本，避免依赖文件模式；如果 PR 里一并修正这些脚本的可执行位，评审时说明这是 Noetic 下的复现修复。

## 6. 编译与启动

把 `src/chap8` 下的机械臂包和本模块放到同一个 catkin 工作空间：

```bash
mkdir -p ~/catkin_ws/src/chap8
cp -r <repo>/src/chap8/marm_description ~/catkin_ws/src/chap8/
cp -r <repo>/src/chap8/marm_moveit_config ~/catkin_ws/src/chap8/
cp -r <repo>/src/chap8/marm_planning ~/catkin_ws/src/chap8/
cp -r <repo>/src/chap8/moveit_arm_planning ~/catkin_ws/src/chap8/

cd ~/catkin_ws
catkin_make
source devel/setup.bash
```

先单独验证 MoveIt 配置：

```bash
roslaunch marm_moveit_config demo.launch use_gui:=false
```

预期结果：

- RViz 打开，能看到六自由度机械臂；
- 左侧 MotionPlanning 面板正常加载；
- 机械臂处于 `home` 位姿；
- **不会**弹出 Gazebo 窗口。

再启动本模块的一键入口：

```bash
roslaunch moveit_arm_planning main.launch
```

如果虚拟机里 RViz 黑屏或报 OpenGL 错误，先设置软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

## 7. 实验内容

### 7.1 正运动学（FK）

给定一组关节角，观察末端执行器的位姿变化：

```bash
rosrun marm_planning moveit_fk_demo.py
```

记录：

- 目标关节角；
- 末端执行器实际位姿；
- 运动是否平滑、是否发生碰撞。

### 7.2 逆运动学（IK）

给定末端执行器的目标位姿，让 MoveIt 求解关节角并规划：

```bash
rosrun marm_planning moveit_ik_demo.py
```

记录：

- 目标位姿；
- 求解是否成功；
- 末端位置误差和姿态误差；
- 规划耗时。

### 7.3 笛卡尔路径

让末端执行器沿直线或圆弧运动：

```bash
rosrun marm_planning moveit_cartesian_demo.py
```

重点观察笛卡尔路径是否连续、是否出现关节跳变。

### 7.4 障碍物避让

在规划场景中加入桌面、盒子等障碍物，再规划到目标位姿：

```bash
rosrun marm_planning moveit_obstacles_demo.py
```

记录：

- 障碍物尺寸和位置；
- 规划是否成功；
- 规划轨迹是否绕过障碍物；
- 规划耗时。

### 7.5 抓取

在规划场景中放置桌面和目标盒子，执行接近、夹取、抬起、放置的完整流程：

```bash
rosrun marm_planning moveit_pick_and_place_demo.py
```

记录：

- 抓取成功次数；
- 放置位置误差；
- 总耗时。

### 7.6 OMPL 规划器对比

对同一组目标位姿，分别使用 RRTConnect、RRT*、PRM 进行规划。规划器名称以 `marm_moveit_config/config/ompl_planning.yaml` 为准；MoveIt 1 运行时通常使用带 `kConfigDefault` 后缀的名称，例如 `RRTConnectkConfigDefault`。

批量运行：

```bash
roslaunch moveit_arm_planning eval.launch \
  targets:="$HOME/catkin_ws/src/chap8/moveit_arm_planning/config/targets.yaml" \
  repeats:=3 \
  timeout:=5.0 \
  out_dir:="$HOME/moveit_arm_results"
```

建议先准备 10 个手工确认可达的目标位姿，跑通后再扩展到 20 个。每个目标位姿重复 3 次，规划超时设为 5 秒。

指标定义：

| 指标 | 定义 |
| --- | --- |
| 成功率 | 成功规划次数 / 总尝试次数 |
| 规划耗时 | 调用 `plan()` 前后的墙钟时间，不含 RViz 渲染 |
| 末端路径长度 | 规划轨迹按等间隔采样后，末端位置欧氏距离之和 |
| 位置误差 | 规划终点与目标点的欧氏距离 |
| 姿态误差 | 规划终点与目标姿态的四元数夹角 |
| 关节路径长度 | 相邻轨迹点关节角绝对差之和，用于观察关节空间绕行程度 |

### 7.7 参数文件

`config/targets.yaml` 保存末端目标位姿，每个目标包含位置和四元数姿态：

```yaml
targets:
  - name: target_01
    position: [0.35, 0.00, 0.35]
    orientation: [0.0, 0.7071, 0.0, 0.7071]
  - name: target_02
    position: [0.40, 0.10, 0.30]
    orientation: [0.0, 0.7071, 0.0, 0.7071]
```

上面的数值只是格式示例，不代表该机械臂一定可达。首版请先在 RViz 的 MotionPlanning 面板里手工确认 10 个可达位姿，再写入 YAML；跑通后再扩展到 20 个。

`config/planners.yaml` 保存规划器名称、重复次数和超时：

```yaml
planners:
  - RRTConnectkConfigDefault
  - RRTstarkConfigDefault
  - PRMkConfigDefault
repeats: 3
timeout: 5.0
```

规划器名称必须和 `marm_moveit_config/config/ompl_planning.yaml` 中的定义一致，不同 MoveIt 版本的默认后缀可能不同，运行前先确认。

`scripts/pick_place_task.py` 只负责调用 `marm_planning` 里已有的抓取 demo 并记录指标，不重复实现抓取算法。

### 7.8 实验流程

1. `roslaunch marm_moveit_config demo.launch use_gui:=false`，确认机械臂和 MotionPlanning 面板正常。
2. 依次运行 FK、IK、笛卡尔、障碍物、抓取五个 demo，记录终端输出并截图。
3. 在 RViz 里确认 10 个可达目标位姿，写入 `config/targets.yaml`。
4. 运行 `eval.launch`，得到三种规划器的逐次记录和汇总 CSV。
5. 用真实结果替换第 8 节的“待实测”，补齐配图，再删除文首的初稿状态说明。

## 8. 实验结果

> 以下数据全部待实测。正式提交前必须用真实运行结果替换，不能保留“待实测”。

### 8.1 单次任务

| 任务 | 是否成功 | 规划耗时（s） | 位置误差（m） | 姿态误差（rad） | 备注 |
| --- | --- | --- | --- | --- | --- |
| FK | 待实测 | 待实测 | 待实测 | 待实测 |  |
| IK | 待实测 | 待实测 | 待实测 | 待实测 |  |
| 笛卡尔路径 | 待实测 | 待实测 | 待实测 | 待实测 |  |
| 障碍物避让 | 待实测 | 待实测 | 待实测 | 待实测 |  |
| 抓取 | 待实测 | 待实测 | 待实测 | 待实测 |  |

### 8.2 OMPL 规划器对比

| 规划器 | 尝试次数 | 成功次数 | 成功率 | 平均耗时（s） | P95 耗时（s） | 平均末端路径（m） | 平均位置误差（m） |
| --- | --- | --- | --- | --- | --- | --- | --- |
| RRTConnect | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 |
| RRT* | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 |
| PRM | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 | 待实测 |

### 8.3 配图

图片待运行后补充，统一放在 `docs/img/chap8/moveit_arm_planning/`，PNG 小于 3 MB。

<!--
![RViz 中的机械臂初始位姿](../img/chap8/moveit_arm_planning/rviz_home.png)

![规划场景中的障碍物](../img/chap8/moveit_arm_planning/planning_scene.png)

![三种 OMPL 规划器对比](../img/chap8/moveit_arm_planning/planner_comparison.png)
-->

## 9. 常见问题

### 9.1 `xacro: unrecognized argument --inorder`

Noetic 已删除 `--inorder`，按第 5.1 节修改 `planning_context.launch`。

### 9.2 `Could not find package moveit_ros_move_group`

MoveIt 没有安装完整，执行：

```bash
sudo apt install -y ros-noetic-moveit
```

### 9.3 RViz 黑屏或 OpenGL 报错

虚拟机没有独立显卡时，使用软件渲染：

```bash
export LIBGL_ALWAYS_SOFTWARE=1
```

并把 RViz 窗口缩小到 1280×720 以内，减少渲染压力。

### 9.4 `Unable to identify any set of controllers`

本实验使用 fake execution，不需要真实控制器。确认启动的是 `marm_moveit_config/launch/demo.launch`，且其中 `fake_execution` 为 `true`。

### 9.5 规划一直失败

先检查目标位姿是否在机械臂工作空间内；把目标位姿改到 `home` 附近再试。也可能是目标姿态与当前姿态差距过大，导致 IK 无解。

### 9.6 `rosdep` 报 `manipulation_msgs` 找不到

`marm_planning/package.xml` 里有未使用的旧依赖。本模块的脚本不依赖 `manipulation_msgs`，可以先直接 `catkin_make`；如果后续清理依赖，再单独提交一个小改动。

## 10. AI 使用声明

本文档与配套代码在编写过程中使用了 AI 大模型辅助（需求分析、方案讨论、文档起草和问题排查）。文档中的全部运行数据将由作者在 ROS Noetic + MoveIt 环境中实测采集；作者对提交内容的正确性与完整性负全部责任。
