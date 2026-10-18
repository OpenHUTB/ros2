# `gazebo_diff_drive_car/` 文档目录说明

> 路径：`docs/ground/gazebo_diff_drive_car/`
> 隶属模块：**OpenHUTB ROS 机器人任务 · 地面载具方向**
> 仿真平台：Gazebo 11 ｜ 机器人：差速驱动无人车 ｜ 系统：Ubuntu 20.04 + ROS Noetic

---

## 这个目录是什么

本目录是 **`src/ground/gazebo_diff_drive_car/` ROS 包**的**配套技术文档**，按任务书要求逐章拆分。**不是运行时依赖、不参与 catkin 构建**，仅供人类阅读与评审。

- **入口文档**：[`../gazebo_diff_drive_car.md`](../gazebo_diff_drive_car.md)（在 `docs/ground/` 上一层，作为 mkdocs 章节索引）
- **章节文档**：本目录下 11 篇 `.md`，对应任务书的各功能点
- **图素材**：在 `docs/ground/images/`（截图）与 `docs/ground/data/`（轨迹图、速度曲线、指标报告）

> **给 reviewer 的一句话**：本目录是 ROS 包 `gazebo_diff_drive_car` 的**逐章配套文档**，与代码一一对应；图素材放在 `docs/ground/{images,data}/` 而非本目录，是因为 mkdocs 站点把同一领域（ground）的图集中管理。

---

## 11 篇文档清单与对应代码

| 文档 | 任务书对应 | 内容要点 | 对应代码文件 |
| --- | --- | --- | --- |
| [`teleop.md`](./teleop.md) | 任务 (1) 5 分 | 物理仿真 + 键盘运动控制；车体模型、离地高度、增量式控制 | `scripts/teleop_keyboard.py` |
| [`laser_avoid.md`](./laser_avoid.md) | 任务 (2) 5 分 | 激光雷达感知→控制闭环；扇区划分、自遮挡过滤、FSM + 势场法 | `scripts/laser_avoid.py` |
| [`camera_avoid.md`](./camera_avoid.md) | 任务 (2) 5 分 | 深度相机感知→视觉避障；ROI 统计、地板值剔除、双传感器对比 | `scripts/depth_camera_avoid.py` |
| [`kinematics.md`](./kinematics.md) | 理论基础 | 差速驱动正逆运动学推导、参数敏感性分析 | — |
| [`algorithm.md`](./algorithm.md) | 算法说明 | 人工势场构造、局部极小值规避、滞回抑制、有限状态机 | `scripts/laser_avoid.py` 内 FSM |
| [`sensor_processing.md`](./sensor_processing.md) | 工程实现 | 无效值处理、多传感器数据同步策略 | 所有 `*_avoid.py` |
| [`architecture.md`](./architecture.md) | 架构设计 | 统一入口 + 模式调度；VelocityWatchdog 安全看门狗；TF 树 | `main.py`、`launch/main.launch` |
| [`parameters.md`](./parameters.md) | 参数说明 | 场景尺寸、车辆尺寸、传感器参数、控制参数全集 | `config/params.yaml`、`urdf/car.xacro`、`worlds/car_world.world` |
| [`experiments.md`](./experiments.md) | 实验复现 | 完整可复现的操作流程（10 步）+ 运行截图 | `main.sh`、`main.bat` |
| [`evaluation.md`](./evaluation.md) | 性能评价 | 7 项量化指标（路径效率、控制频率、实时因子等）+ 可视化分析 | `scripts/odom_recorder.py`、`analyze_trajectory.py`、`plot_trajectory.py` |
| [`troubleshooting.md`](./troubleshooting.md) | 工程经验 | **8 个真实 bug 的完整定位过程**（如 `/cmd_vel` 无消息、激光自遮挡、深度图地板值 1.005 m 等） | — |

---

## 图素材组织

### `docs/ground/images/`（运行截图，3 张，**已入库**）

| 文件 | 引用位置 | 内容 |
| --- | --- | --- |
| `G_teleop_a.png` | `teleop.md`、`experiments.md`、`gazebo_diff_drive_car.md` | 键盘遥控模式仿真截图 |
| `H_laser_a.png` | `laser_avoid.md`、`experiments.md`、`gazebo_diff_drive_car.md` | 激光雷达避障截图 |
| `I_camera_a.png` | `camera_avoid.md`、`experiments.md`、`gazebo_diff_drive_car.md` | 深度相机避障截图 |

### `docs/ground/data/`（实验产物，3 项，**本轮 PR 补提交**）

| 文件 | 引用位置 | 内容 |
| --- | --- | --- |
| `trajectory_plot.png` | `evaluation.md`、`laser_avoid.md`、`experiments.md`、`gazebo_diff_drive_car.md` | 三种模式轨迹对比图 |
| `speed_profile.png` | （预留） | 速度曲线图 |
| `metrics_report.txt` | （预留） | 性能指标文本报告 |

> **图片渲染失败根因（已修复）**：仓库根 `.gitignore` 第 34 行 `data/` 规则**没有前导 `/`**，会匹配仓库里**任意层级**的 `data/` 目录，把 `docs/ground/data/` 下的图也一并吞掉。本轮 PR 在 `.gitignore` 中加入白名单 `!docs/ground/data/**` 并 `git add` 这 3 个文件。

---

## 与代码包的目录对应

```
src/ground/gazebo_diff_drive_car/        docs/ground/
├── main.py                              ├── gazebo_diff_drive_car.md   ← 章节索引（mkdocs 入口）
├── main.sh / main.bat                   │
├── package.xml / CMakeLists.txt         ├── gazebo_diff_drive_car/     ← 11 篇章节文档
├── launch/main.launch                   │   ├── teleop.md
├── urdf/car.xacro                       │   ├── laser_avoid.md
├── worlds/car_world.world               │   ├── camera_avoid.md
├── scripts/                             │   ├── kinematics.md
│   ├── teleop_keyboard.py               │   ├── algorithm.md
│   ├── laser_avoid.py                   │   ├── sensor_processing.md
│   ├── depth_camera_avoid.py            │   ├── architecture.md
│   ├── odom_recorder.py                 │   ├── parameters.md
│   ├── analyze_trajectory.py            │   ├── experiments.md
│   └── plot_trajectory.py               │   ├── evaluation.md
├── config/params.yaml                   │   └── troubleshooting.md
└── rviz/car.rviz                        │
                                         ├── images/                    ← 运行截图（已入库）
                                         │   ├── G_teleop_a.png
                                         │   ├── H_laser_a.png
                                         │   └── I_camera_a.png
                                         └── data/                       ← 实验产物（本轮补入库）
                                             ├── trajectory_plot.png
                                             ├── speed_profile.png
                                             └── metrics_report.txt
```

---

## 维护约定

1. **加新章节** → 在本 README「11 篇文档清单」表格中追加一行，并在上层 [`../gazebo_diff_drive_car.md`](../gazebo_diff_drive_car.md) 的「文档目录」表中也追加对应链接。
2. **加新图** → 截图统一放 `docs/ground/images/`，实验产物（轨迹图、报告）放 `docs/ground/data/`；**不要**把图素材散落到本目录下，避免与章节 md 混在一起。
3. **引用路径** → 章节 md 引用图统一用相对路径 `../images/xxx.png` 或 `../data/xxx.png`；不要用绝对路径或外部图床。
4. **`.gitignore` 注意** → 不要再写无前导 `/` 的目录规则（如 `data/`），它会把任意层级的同名目录都吞掉；如需忽略某子目录，用 `/src/.../data/` 这种锚定到根的写法。
5. **PR 模板勾选** → 若本次 PR 改动了代码包，请在 PR 描述里附上对应章节文档的相对路径链接，方便 reviewer 比对。

---

## 给 reviewer 的备注

- 本目录**所有 11 篇 md 与 3 张 images/ 截图均已入库**，PR 渲染失败仅因 `data/` 下 3 个实验产物被 `.gitignore` 误伤，本轮已修复并补提交。
- 代码改动若想找到对应文档，请按上表「对应代码文件」一列反查。
