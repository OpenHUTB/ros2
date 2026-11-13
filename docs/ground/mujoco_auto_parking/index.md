# MuJoCo 四轮差速车自动泊车

> 机器人操作系统及应用课程作业：任务①物理仿真与键盘控制、任务②传感器感知与给定轨迹控制、任务③建图与导航、任务④图像端到端控制。

本模块在 MuJoCo 3.12 中搭建一个停车场场景，用四轮差速小车完成四个任务。所有感知、规划、控制、端到端算法均为神经网络，训练、测试和 ROS 2 封装都在仓库内。

## 四个任务

| 任务 | 内容 | 文档 |
| --- | --- | --- |
| ① | 停车场物理仿真 + 键盘运动控制 | [任务①：物理仿真与键盘控制](./01_physical_simulation_keyboard.md) |
| ② | 相机/深度/激光雷达感知 + 车辆位姿感知网络 + 给定 S 曲线控制网络 | [任务②：感知与给定轨迹控制](./02_perception_trajectory_control.md) |
| ③ | log-odds 占据栅格建图 + Neural A* + 神经网络导航闭环 | [任务③：建图与导航](./03_slam_navigation.md) |
| ④ | 前视图像 → 速度/转向 的端到端 CNN | [任务④：图像端到端控制](./04_end_to_end.md) |

## 运行环境

| 组件 | 版本 |
| --- | --- |
| Ubuntu | 22.04（WSL2 实测） |
| ROS 2 | Humble |
| Python | 3.10 |
| MuJoCo | 3.12.0 |

运行依赖见 `src/ground/mujoco_auto_parking/requirements.txt`：`mujoco==3.12.0`、`numpy`、`PyYAML`。
文档与 GIF 工具另需 `markdown`、`htmldocx`、`python-docx`、`Pillow`、`opencv-python`，见 `src/ground/mujoco_auto_parking/tools/requirements-docs.txt`（运行仿真本身不需要这些）。

## 快速开始

```bash
cd <仓库根目录>          # 仓库根就是 colcon 工作空间
source /opt/ros/humble/setup.bash
colcon build --packages-select mujoco_auto_parking --symlink-install
source install/setup.bash

cd src/ground/mujoco_auto_parking
./main.sh --launch         # 任务①：仿真 + 键盘
./main.sh --task2          # 任务②：感知 + S 曲线控制
./main.sh --task3          # 任务③：建图 + Neural A* + 导航
./main.sh --task4          # 任务④：图像端到端控制
```

> 仿真全程离屏渲染：WSL2 下 `main.sh` 默认 `MUJOCO_GL=osmesa`，本模块没有 MuJoCo 交互窗口。带桌面或 WSLg 时可以加 `--gui`，让 `main.sh` 切到 `glfw` + 软件光栅化后端（没有 X 显示时不要加，GLFW 离屏上下文可能段错误）。观察运行画面请用 `ros2 run rqt_image_view rqt_image_view` 订阅 `/camera/image_raw`，或运行 `tools/make_gifs.py` 生成 GIF。

测试与训练：

```bash
./main.sh --test          # 任务①
./main.sh --test-sensors  # 任务②传感器
./main.sh --test-nn       # 任务②网络
./main.sh --test3         # 任务③
./main.sh --test4         # 任务④
./main.sh --train         # 感知 + 轨迹控制
./main.sh --train-planning  # Neural A*
./main.sh --train-e2e     # 端到端 CNN
```

## 实测结果

| 任务 | 指标 | 结果 |
| --- | --- | --- |
| ① | 前进/后退/转向、键位、Watchdog | 全部通过；pytest 18 项 |
| ② | 车位感知 MAE（80 个测试位姿） | dx 0.342 m、dy 0.202 m、dθ 0.054 rad |
| ② | 给定 S 曲线闭环 | NN RMSE 0.359 m 到达；零权值 0.704 m 未到达 |
| ③ | 建图 + Neural A* + 导航 | 上/下入口约 14.3 s 到达，误差 0.34 m，覆盖约 55% |
| ③ | Neural A* vs A* | 展开节点 659 vs 1370（合成网格） |
| ④ | 图像端到端闭环 | CNN RMSE 0.355 m 到达；零权值 0.684 m |
| ④ | 推理延迟 | 约 1.9 ms/帧 |

## 实测证据

- [证据目录与回归日志](../../evidence/mujoco_auto_parking/README.md)
- 一键回归：在 `src/ground/mujoco_auto_parking` 下执行 `bash tools/verify_all.sh`，脚本跑完后把六个入口的退出码写入 `docs/evidence/mujoco_auto_parking/verification/summary.txt`。该目录由脚本运行时创建，未纳入仓库，所以站点上没有直接链接。
- [Word 导出报告](../../evidence/mujoco_auto_parking/mujoco_auto_parking_report.docx)

## 与仓库现有模块的区别

| 现有模块 | 它做了什么 | 本模块新增什么 |
| --- | --- | --- |
| `mujoco_car_project` | MuJoCo 差速车 + 键盘，ROS1 catkin，无 launch/传感器 | ROS2 包、停车场、相机/雷达、神经网络、建图导航、端到端 |
| `mujoco_diff_drive_slip_compensation` | ROS2 差速车基础，只有 `/cmd_vel` 和真值里程计 | 传感器、感知网络、Neural A*、导航闭环、端到端 |
| `carla_perception_control` | CARLA 前进路点跟踪 | MuJoCo 停车场、倒车/档位语境、建图导航、端到端 |
| `carla_nn_car_following` | ACC 跟车 | 自动泊车场景，车位感知与路径规划 |
| `src/ground` 其它 | 没有建图/导航模块，也没有真正跑通的图像→控制闭环 | 本模块补上任务③④ |

## 目录结构

```text
mujoco_auto_parking/
├── main.py / main.sh         # 课程主入口
├── launch/                   # main / task2 / task3 / task4
├── config/simulation.yaml
├── models/
│   ├── parking_lot.xml       # 停车场 + 差速车 + 相机/雷达
│   ├── perception.npz        # 车位感知网络
│   ├── trajectory.npz        # 轨迹控制网络
│   ├── planner.npz           # Neural A* 引导场
│   └── e2e_small_cnn_bc.npz  # 端到端 CNN
├── mujoco_auto_parking/      # 节点与算法库
├── train/                    # 4 个训练脚本
├── test/                     # 传感器/网络/规划/端到端测试
└── tools/                    # GIF、Word、回归脚本
```

## 文档工具与提交说明

- 本模块按仓库根 `mkdocs.yml` 构建，`mkdocs build --clean` 已在本地通过；GitHub Pages 由仓库的 `OpenHUTB/actions-mkdocs` 工作流构建（该工作流使用 OpenHUTB 的 `hutb-doc` 发行版）。
- Word 导出使用 `markdown + htmldocx + python-docx`（依赖清单见 `tools/requirements-docs.txt`，`pip install -r tools/requirements-docs.txt`），脚本为 `src/ground/mujoco_auto_parking/tools/export_word.py`（在包目录下即 `tools/export_word.py`）。默认读取仓库 `docs/`，输出到 `docs/evidence/mujoco_auto_parking/mujoco_auto_parking_report.docx`；在 WSL 临时工作空间运行时设置 `AUTO_PARKING_DOCS_DIR=/path/to/ros2/docs`。脚本会重新打开导出的 docx，断言五个一级标题（模块首页与四个任务页）都在，缺少任一条即报错。如果课程提供了 `Markdown2Anything` 专用版本，提交前可以用课程版本再导出一次做格式对照。
- GIF 和截图都来自真实 MuJoCo 渲染；`tools/make_gifs.py` 生成 GIF，`screenshots/` 下是从 GIF 提取的静态截图。
- GitHub 的 commit message 不能直接嵌图；运行效果图放在 PR 正文和 `docs/img/ground/mujoco_auto_parking/`。如果老师要求 commit message 里也带图，推送前用 `git commit --amend` 在提交信息末尾补一行图片路径。

## 已知限制

- 任务②感知依赖目标车位的绿色立柱/绿色区域；换停车场时需要重新采集和训练。
- 任务③在未知栅格上采用“未知视为可通行”的乐观规划，没有回环检测；停车场较小，误差可控。
- 任务④在 S 曲线演示上做行为克隆，不是完整的倒车入库策略；更大范围的端到端泊车需要更多专家数据。
- 所有结果来自 MuJoCo 仿真，不含实车验证。
