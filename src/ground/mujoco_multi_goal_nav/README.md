# mujoco_multi_goal_nav

MuJoCo `free joint` 小车多目标导航：从原点出发依次到达三个目标点，每到一个点打印坐标与
实测误差，全部到达后停车。纯 Python + MuJoCo，不依赖 ROS 话题。

## 文件说明

| 文件 | 作用 |
|---|---|
| `main.py` | 入口：场景定位、目标点列表、平面速度闭环、姿态锁定、统计输出 |
| `scene.xml` | 场景：地面、小车、三个目标点标记（不参与碰撞） |
| `launch/main.launch` | ROS Noetic 一键启动（`roslaunch mujoco_multi_goal_nav main.launch`）|
| `test/test_main.py` | 回归测试：目标点常量、参数、场景路径回退、目标标记不参与碰撞、姿态复位 |
| `requirements.txt` | 运行依赖（`mujoco`、`numpy`）|
| `CMakeLists.txt` / `package.xml` | catkin 包定义，与 `src/ground/` 下同类模块一致 |
| `../../docs/ground/mujoco_multi_goal_nav.md` | 文档站页面 |
| `../../docs/img/ground/mujoco_multi_goal_nav_run.png` / `_demo.gif` | 效果图与动图 |

## 运行环境

- Ubuntu 20.04 / Python 3.8+
- `pip3 install -r requirements.txt`
- 本文数据在 MuJoCo 4.0.0 下实测；MuJoCo 2.3.7 复跑结果逐位相同（见「实测结果」），
  代码未使用 4.x 独有 API，`requirements.txt` 的依赖下限即 2.3.7

## 目标点与参数

| 目标点 | 坐标 (x, y) / m | 标记颜色 |
|---|---|---|
| 1 | (1.5, 1.0) | 绿 |
| 2 | (-1.5, 1.0) | 黄 |
| 3 | (-1.5, -1.0) | 紫 |

| 参数 | 默认 | 说明 |
|---|---|---|
| `--nogui` | 关 | 不建窗口，只跑物理（无显示环境）|
| `--duration` | 40.0 | 最大仿真时长（秒）|
| `--csv` | 无 | 轨迹 CSV 输出路径 |
| `SPEED` | 0.3 | 最高线速度 m/s |
| `SLOW_RADIUS` | 0.3 | 进入减速区的距离 m |
| `STOP_DIST` | 0.02 | 到达判定阈值 m |

## 运行

```bash
cd src/ground/mujoco_multi_goal_nav

pip3 install -r requirements.txt

python3 main.py                        # 带窗口
python3 main.py --nogui                # 无窗口，出统计
python3 main.py --nogui --csv traj.csv # 顺带落轨迹
```

### 回归测试

```bash
cd src/ground/mujoco_multi_goal_nav
python3 -m unittest discover -s test -v
# 或：python3 -m pytest test/test_main.py -v
```

8 项用例全部通过（目标点常量、参数范围、场景路径回退、目标标记 `contype/conaffinity` 为 0、
`lock_pose()` 复位姿态），不启动仿真、约 0.1 s 跑完。

### 通过 launch（ROS Noetic）

```bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash
roslaunch mujoco_multi_goal_nav main.launch                    # 带窗口
roslaunch mujoco_multi_goal_nav main.launch nogui:=true        # 无窗口
```

`nogui` 由 `main.launch` 里两个 `<node>` 的 `if` / `unless` 选分支，不带窗口的那支追加 `--nogui`；
刻意不用 `eval` 表达式，因为 roslaunch 解析 substitution 时按括号配对，表达式里的 `(` 会报
`RLException: Invalid left parenthesis`。

#### launch 启动失败时的排查顺序

```bash
source /opt/ros/noetic/setup.bash
cd ~/catkin_ws && catkin_make && source devel/setup.bash

rospack find mujoco_multi_goal_nav                                            # 包能不能找到
ls -l $(rospack find mujoco_multi_goal_nav)/../../lib/mujoco_multi_goal_nav/   # install 产物
roslaunch mujoco_multi_goal_nav main.launch nogui:=true
```

| 报错 / 现象 | 原因 | 处理 |
|---|---|---|
| `[main.launch] is neither a launch file in package [mujoco_multi_goal_nav]` | 没 `catkin_make` 或没 `source`，或包不在 `~/catkin_ws/src` 下 | 先 `catkin_make` 再 `source devel/setup.bash`；确认目录层级 |
| `Cannot locate node of type [main.py]` | `lib/<pkg>/main.py` 没生成或没有执行位 | 重新 `catkin_make`；确认 `CMakeLists.txt` 里 `catkin_install_python(PROGRAMS main.py)` |
| 节点起来后立刻退出，日志里有 `缺少 MuJoCo` | 当前 Python 环境没装 MuJoCo | `pip3 install -r requirements.txt` |
| `roslaunch: command not found` / `ROS_VERSION=2` | 环境是 ROS 2，本包是 catkin(ROS 1) 包 | 直接 `python3 main.py --nogui`，或换 ROS 1 Noetic 环境 |

## 实测结果（`python3 main.py --nogui`；MuJoCo 4.0.0 与 2.3.7 双版本复跑，数值一致）

| 目标点 | 累计用时 / s | 到位误差 / m |
|---|---|---|
| 1 | 7.71 | 0.0199 |
| 2 | 19.37 | 0.0199 |
| 3 | 27.73 | 0.0200 |

到达 3 / 3，总仿真时长 27.73 s，最终位置 (-1.500, -0.980)。

## 实现要点

- 速度直接写 `data.qvel[0] / [1]`，大小 `min(SPEED, SPEED*dist/SLOW_RADIUS)`，
  到 `dist < STOP_DIST` 切下一个目标；
- 每步 `mj_step` 后把 `qpos[2]`、`qpos[3:7]` 复位、`qvel[2]`、`qvel[3:6]` 置 0，
  车头全程保持初始朝向，不会斜着平移；
- `resolve_scene_path()` 三级回退（环境变量 `MUJOCO_MULTI_GOAL_SCENE` → 脚本同目录 → `../share/<pkg>/`），
  显式指定 XML、源码目录直跑、安装空间启动三种情形都能找到 `scene.xml`。

## 已知限制

- 直接写 `qvel` 属运动学控制，轮子不转、没有驱动力矩参与，物理真实性有限；
- 姿态被锁定，车不会转向；
- 目标点固定在 `main.py` 的 `GOALS` 常量中，改点需同步改 `scene.xml` 标记。

## AI 使用声明

代码与文档在编写过程中使用 **豆包（Doubao）** 辅助；场景参数、控制律与实测数据均由作者
核对并实际运行确认。
