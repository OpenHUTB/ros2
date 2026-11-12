# 无人机桨故障后的自主迫降（在线建图 + 神经评分 + 闭环制导）

四旋翼在飞行中掉一个桨之后还能不能安全落地，取决于两件事：**往哪儿落**、**怎么飞过去**。
本模块用下视深度相机在线建一张 2.5D 高度图，用卷积网络给每个格子打"能不能落"的分，
再用一个回归网络输出速度指令，配合底座的几何控制器闭环飞到落点。

对应课程任务③（虚拟环境建模、导航），并复用任务②的感知与控制：

| 模块 | 任务 | 做法 |
| --- | --- | --- |
| `uav_actuator_fault_sim` | ① | 故障注入层，给后续模块提供可复现的故障与评测台 |
| `uav_rotor_fault_detector_nn` | ② 感知 | IMU + 指令 → 1D-CNN+GRU → 四桨健康度 |
| `uav_neural_fault_tolerant_control` | ② 控制 | 神经残差 + 加权伪逆分配，带病继续飞 |
| **`uav_fault_emergency_landing`** | **③ 建图/导航** | **本模块：建图 → 评分选点 → 神经制导闭环迫降** |

与仓库里已有的 `uav_emergency_landing_selector`（规则打分 + 固定候选点）的区别：
本模块是**在线建图 + 神经网络评分 + 闭环制导**，起飞点故意选在楼顶上方，
所以"垂直往下掉"会穿楼，"选点迫降"能飞到空地——差别可以在动图里直接看到。

## 为什么不能直接往下掉

`urban_block` 场景里楼最高 10.68 m，而四旋翼在底座里**没有碰撞体**（会直接穿过楼板）。
把起飞点放在最高楼上方 13.7 m：

| 策略 | 结果 | 触地时刻 | 触地垂直速度 | 落点误差 |
| --- | --- | --- | --- | --- |
| 垂直下降（基线） | 撞到障碍物 | 12.88 s | 0.42 m/s | — |
| 规则评分 + 专家律 | 安全迫降 | 9.42 s | 0.56 m/s | 0.00 m |
| 神经评分 + 神经制导 | 安全迫降 | 9.90 s | **0.05 m/s** | 0.04 m |

神经制导的触地垂直速度比专家律低一个量级（0.05 vs 0.56 m/s），代价是慢 0.5 s。
上面是"单桨衰减剩余 50%"那一组的结果；`--eval` 还会跑健康基线与越界组（剩余 22%），
后者三种策略都会硬着陆，用来标出容错边界。完整九行数据见 `docs/air/uav_fault_emergency_landing.md`。

## 方法

1. **建图**：`down_cam`（机体正下方，fovy=80°）深度图 → 反投影到世界系 → 每格取最高点，
   得到 48×48、0.5 m 分辨率的 2.5D 高度图。
2. **规则评分（教师）**：`free × flat × margin × conf × near`——无障碍、邻域平坦、
   离障碍有距离、邻域观测足够、离当前位置近。部署时不跑规则，只当蒸馏的教师。
3. **神经评分**：3 层全卷积 + sigmoid，2625 参数，换网格尺寸不用改结构。
4. **神经制导**：13 维特征（相对落点、速度、四桨权限、高度、水平距离、剩余权限）→
   隐藏层 64×2 的 MLP，5251 参数，输出三轴速度指令，监督自解析专家律。
5. **闭环**：`GeometricController`（姿态内环）+ `ls_allocation`（按可控桨数切行权重，
   桨彻底失效时主动放弃偏航）+ 到位门限（没飞到落点正上方不许压到已知最高障碍以下）。

网络只输出**速度指令**，不直接输出推力/力矩：即使网络在训练分布外给出离谱输出，
最多飞得难看，不会把机体翻过来。

## 目录

```
uav_fault_emergency_landing/
├── main.py / main.sh / main.bat        # 入口
├── launch/landing.launch.py            # ROS2 启动
├── config/landing.yaml                 # 节点参数（顶层键 /**）
├── models 不在此包：动力学与场景全部复用 uav_mujoco_sim
├── results/
│   ├── score.pt  (14 KB)               # 评分网络权重，入库
│   ├── guidance.pt (26 KB)             # 制导网络权重，入库
│   ├── score_dataset.npz               # 不入库，--gen-data 生成
│   └── guidance_dataset.npz            # 不入库，--gen-data 生成
└── uav_fault_emergency_landing/
    ├── mapping.py     # 深度反投影、高度图、规则评分
    ├── landing.py     # 专家律、制导特征、落点选择
    ├── model.py       # 两个网络与检查点
    ├── controller.py  # 闭环控制器（vertical / expert / neural）
    ├── data.py        # 数据集生成
    ├── train.py       # 训练
    ├── evaluate.py    # 闭环评测（含真值判据）
    ├── render.py      # 动图与图
    ├── ros_node.py    # ROS2 节点
    ├── cli.py / main.py
    └── selftest.py
```

## 运行环境

| 项 | 版本 |
| --- | --- |
| 系统 | Ubuntu 22.04.5（WSL2 也可以） |
| ROS | ROS2 Humble |
| Python | 3.10（ROS2 侧） |
| 仿真 | MuJoCo 3.12.0（Windows 上 3.17.0 也能跑） |
| 神经网络 | torch 2.14.1+cpu |
| 其他 | numpy>=1.21、pillow>=10、matplotlib>=3.5（只有画图需要） |

不需要独显、不需要 Gazebo、不需要 Unreal。

## 快速开始

```bash
cd src/air/uav_fault_emergency_landing
pip install -r requirements.txt

# 1) 自检（缺 torch / mujoco 的组会自动 SKIP）
python main.py --selftest

# 2) 生成数据（评分地图 120 张 + 制导样本 4000 条，约 30 s）
python main.py --gen-data

# 3) 训练两个网络（各约 0.5 s）
python main.py --train

# 4) 三种模式 × 三组工况（健康 / 剩余推力 50% / 剩余推力 22%）的对比（约 15 s）
python main.py --eval

# 5) 出一段对比动图
python main.py --demo results/landing_demo.gif
```

单元测试：

```bash
python -m pytest test -q     # 23 passed, 1 skipped
```

## ROS2 启动

```bash
colcon build --packages-select uav_mujoco_sim uav_fault_emergency_landing
source install/setup.bash
ros2 launch uav_fault_emergency_landing landing.launch.py mode:=neural rotor:=1
```

| 话题 | 类型 | 说明 |
| --- | --- | --- |
| `/uav/odom` | `nav_msgs/Odometry` | 位姿与速度，默认 20 Hz（`publish_rate`） |
| `/uav/status` | `std_msgs/String` | `HOVER` / `DESCENDING` / `LANDED` |
| `/uav/landing_target` | `geometry_msgs/Point` | 当前选中的落点 |
| `/uav/health` | `std_msgs/Float32MultiArray` | 四桨推力权限真值 |
| `/uav/landing_scores` | `std_msgs/Float32MultiArray` | 评分图的 max / mean |
| `/uav/goal` | `geometry_msgs/Point` | 输入：人工指定落点（0.5 s 内优先） |
| `/uav/cmd_vel` | `geometry_msgs/Twist` | 输入：仅记录，迫降过程中不执行 |

参数分两层：`config/landing.yaml` 是默认值真源，`launch` 只覆盖常用的几个
（scene / fault / rotor / severity / duration）。权重按 cwd → 包目录 → install share 依次查找。

## 已知限制

1. **单桨完全失效、双桨失效仍是物理不可恢复的**：本模块不宣称能救回来，
   两种情况在 onset 后 1 s 内姿态就发散（与 `uav_neural_fault_tolerant_control` 的结论一致）。
   对比实验因此用"健康 / 剩余推力 50% / 剩余推力 22%"三组工况：前两组落在分配层
   可补偿区间内（悬停时坏桨剩余推力高于 2.33/6.0 ≈ 39% 就能靠抬高指令补回来），
   第三组已越过补偿边界，用来展示超出容错能力时的行为。
2. **只覆盖 MuJoCo 单机仿真**，没有真机与实飞数据。
3. **四旋翼没有碰撞体**，"撞到障碍物"是用竖直射线量的几何间隙 + 落点真值高度判出来的，
   不是物理碰撞。
4. 建图是**一次性扫描**的结果，飞行途中只以 10 步一帧的频率增量更新；
   真实系统里应该持续建图并把新观测融进评分。
5. 制导网络是**解析专家律的蒸馏**，不是强化学习；它的上限就是专家律的上限。
