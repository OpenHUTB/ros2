# moveit_arm_planning

六自由度机械臂的 MoveIt 运动规划实验包：在 ROS Noetic 下使用 MoveIt 1 完成 FK、IK、笛卡尔路径、障碍物避让和抓取，并对 RRTConnect、RRT*、PRM 三种 OMPL 规划器做统一指标对比。

本模块只使用 MoveIt fake execution 和 RViz，**不启动 Gazebo、MuJoCo 或任何物理引擎**。

## 运行环境

- Ubuntu 20.04
- ROS Noetic
- MoveIt 1（`ros-noetic-moveit`）
- RViz
- Python 3

## 依赖安装

```bash
sudo apt update
sudo apt install -y \
  ros-noetic-moveit \
  ros-noetic-joint-state-publisher \
  ros-noetic-robot-state-publisher \
  ros-noetic-xacro
```

## 目录结构

```text
moveit_arm_planning/
├── launch/
│   ├── main.launch
│   └── eval.launch
├── config/
│   ├── targets.yaml
│   └── planners.yaml
├── scripts/
│   ├── plan_eval.py
│   └── pick_place_task.py
├── CMakeLists.txt
├── package.xml
├── main.sh
└── README.md
```

## 编译

本模块复用 `src/chap8` 下已有的 `marm_description`、`marm_moveit_config` 和 `marm_planning`。把四个包放到同一个 catkin 工作空间：

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

编译前先按 `docs/chapter/chap8/moveit_arm_planning.md` 第 5 节修复 Noetic 下的 `xacro --inorder` 和 `state_publisher`。

`marm_planning/scripts/` 下的 demo 没有可执行位，第一次运行前执行：

```bash
chmod +x ~/catkin_ws/src/chap8/marm_planning/scripts/*.py
```

## 运行

先验证 MoveIt 配置：

```bash
roslaunch marm_moveit_config demo.launch use_gui:=false
```

再启动本模块：

```bash
roslaunch moveit_arm_planning main.launch
```

批量比较三种 OMPL 规划器：

```bash
roslaunch moveit_arm_planning eval.launch \
  targets:="$HOME/catkin_ws/src/chap8/moveit_arm_planning/config/targets.yaml" \
  repeats:=3 \
  timeout:=5.0 \
  out_dir:="$HOME/moveit_arm_results"
```

结果写入 `~/moveit_arm_results/`，包括逐次规划记录 CSV 和汇总 CSV。

## 实验内容

| 任务 | 入口 | 观察点 |
| --- | --- | --- |
| FK | `rosrun marm_planning moveit_fk_demo.py` | 关节角变化与末端位姿 |
| IK | `rosrun marm_planning moveit_ik_demo.py` | 求解成功率、末端误差 |
| 笛卡尔路径 | `rosrun marm_planning moveit_cartesian_demo.py` | 路径连续性与关节跳变 |
| 障碍物避让 | `rosrun marm_planning moveit_obstacles_demo.py` | 是否绕开障碍物 |
| 抓取 | `rosrun marm_planning moveit_pick_and_place_demo.py` | 抓取/放置成功率 |
| 规划器对比 | `roslaunch moveit_arm_planning eval.launch` | 成功率、耗时、路径长度 |

`scripts/pick_place_task.py` 只调用并记录 `marm_planning` 里已有的抓取 demo，不重复实现抓取算法。

## 常见问题

- `xacro: unrecognized argument --inorder`：按文档第 5.1 节去掉 `--inorder`。
- `Could not find package moveit_ros_move_group`：安装 `ros-noetic-moveit`。
- RViz 黑屏：`export LIBGL_ALWAYS_SOFTWARE=1`。
- 规划失败：先把目标位姿改到 `home` 附近，确认机械臂工作空间。

更完整的实验说明见 `docs/chapter/chap8/moveit_arm_planning.md`。
