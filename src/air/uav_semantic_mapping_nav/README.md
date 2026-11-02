# uav_semantic_mapping_nav — 语义分割建图与语义路径规划

对应作业任务（2）传感器感知与任务（3）建图与导航。

几何地图只知道"哪里有障碍"，不知道"那是什么"。本模块用语义分割构建语义地图，
使无人机能完成"沿可通行区域飞、绕开植被、从门框穿过"这类需要理解环境的任务。

分割网络是纯 numpy 手写前向与反向的小型编解码网络（不依赖深度学习框架），
在仿真真值分割上训练；语义栅格把占据概率与类别分布分开估计；规划器在语义
代价图上跑 A*。对比实验：几何代价图 / 语义代价图 / 真值分割语义图。

依赖 [uav_mujoco_sim](../uav_mujoco_sim) 提供 `/camera/image_raw`、
`/camera/depth`、`/camera/seg` 与 `/uav/odom`。

---

## 1. 与课程已有模块的区别

| 已有模块 | 区别 |
|---|---|
| `octree_uav_3d_pathfinding` | 几何八叉树占用地图，只有占据/空闲，没有类别 |
| `unmanned_aerial_car_perception` | 是车，且不做地图 |
| `carla_slam_gmapping` | 是车，纯几何 |

课程内无人机建图全为纯几何，没有任何语义建图；分割网络也用到了神经网络，
符合"感知算法需为神经网络"。

## 2. 架构

```
RGB ──→ 分割网络（编码器 3 层 stride-2 + 1×1 分类头 + 最近邻上采样）
              │
              ├─ 9 类语义图 + 逐像素置信度
              │
深度 ─────────┴─→ 逆投影（针孔模型） ─→ 语义栅格
                                          │  占据对数几率（几何）
                                          │  类别投票（语义）
                                          │
                                   语义代价图 ─→ A* ─→ 路径
```

## 3. 安装

运行环境：ROS2 Humble（Ubuntu 20.04，Python 3.8）+ MuJoCo 3.12.0（仿真侧 Python ≥3.10）。

```bash
cd ~/uav_ws/ros2_ws/src
cp -r <本目录> .

# 仿真侧（需要 Python ≥3.10）
uv venv ~/uav_ws/.venv --python 3.12
uv pip install --python ~/uav_ws/.venv/bin/python 'mujoco==3.12.0' numpy

# ROS 侧（Python 3.8）
cd ~/uav_ws/ros2_ws
colcon build --packages-select uav_mujoco_sim uav_semantic_mapping_nav
source install/setup.bash
```

`core/` 与 `plotting.py` 只依赖 numpy 与 matplotlib，没有 mujoco 的机器也能跑
`--evaluate` 与 `--figures`。

## 4. 运行步骤

```bash
# 自检（不需要 ROS、不需要显示器）
python main.py --selftest

# 采集与训练（需要 mujoco）
python main.py --collect  --out data/seg_dataset.npz
python main.py --train    --data data/seg_dataset.npz --out data/

# 评估与出图（只需要 numpy + matplotlib）
python main.py --evaluate --data data/seg_dataset.npz --model data/ --out results.json
python main.py --figures  --data data/seg_dataset.npz --model data/ \
               --outdir ../docs/img/air/uav_semantic_mapping_nav
```

ROS2 在线运行：

```bash
# 仿真 + 桥接 + 语义节点
ros2 launch uav_semantic_mapping_nav semantic.launch.py with_sim:=true

# 只起语义节点
ros2 launch uav_semantic_mapping_nav semantic.launch.py

# 用真值分割替代网络预测（对照）
ros2 launch uav_semantic_mapping_nav semantic.launch.py use_truth:=true
```

## 5. 话题接口

| 方向 | 话题 | 类型 | 说明 |
|---|---|---|---|
| 订阅 | `/camera/image_raw` | sensor_msgs/Image（rgb8） | 分割网络输入 |
| 订阅 | `/camera/depth` | sensor_msgs/Image（32FC1，米） | 逆投影 |
| 订阅 | `/camera/seg` | sensor_msgs/Image（rgb8） | 仅 `use_truth:=true` 时用 |
| 订阅 | `/uav/odom` | nav_msgs/Odometry | 位姿（可重映射到 `/vio/odom`）|
| 订阅 | `/uav/goal` | geometry_msgs/Point | 触发规划 |
| 发布 | `/semantic/map` | nav_msgs/OccupancyGrid | `data` 为语义类别 id，−1 未知 |
| 发布 | `/semantic/map_color` | sensor_msgs/Image（rgb8） | 上色版本，rviz 直接看 |
| 发布 | `/semantic/cost` | nav_msgs/OccupancyGrid | 通行代价，按 ×20 量化成 int8 |
| 发布 | `/semantic/path` | nav_msgs/Path | 规划路径 |
| 发布 | `/semantic/status` | std_msgs/String | 帧数、频率、已观测格数 |

## 6. 文件结构

```
uav_semantic_mapping_nav/
├── main.py                       入口（作业要求以 main 开头）
├── uav_semantic_mapping_nav/
│   ├── cli.py                    采集 / 训练 / 评估 / 出图 / 自检
│   ├── selftest.py               30 项自检，无 ROS
│   ├── plotting.py               配图
│   ├── core/
│   │   ├── nn.py                 手写 Conv2d / Linear / 上采样 / Adam / 交叉熵
│   │   ├── segnet.py             分割网络（编码器 + 1×1 头 + 上采样）
│   │   ├── labels.py             9 类语义表与通行代价
│   │   ├── segdata.py            数据采集（弓字形扫描航线）
│   │   ├── training.py           训练与推理
│   │   ├── semantic_map.py       语义栅格（占据对数几率 + 类别投票）
│   │   ├── planner.py            语义代价图上的 A* 与视线捷径
│   │   ├── metrics.py            mIoU / 像素准确率 / ECE / 路径指标
│   │   └── experiment.py         几何 vs 语义 vs 真值分割的对比
│   └── ros/semantic_node.py      ROS2 在线节点
├── config/semantic.yaml          参数
├── launch/semantic.launch.py     启动入口
├── data/                         数据集与预训练权重
└── test/                         pytest
```

## 7. 实测结果

见 `docs/air/uav_semantic_mapping_nav.md`（由 `--evaluate` 直接生成）。

## 8. 已知限制

- 深度图在节点里按 `stride=3` 抽样后逆投影，0.2 m 分辨率下够用；
  要更细的地图需要把 `map.stride` 调到 1，代价是建图耗时线性上升。
- 语义与占据**分开估计**：分割错了只影响代价，不影响避障。
  代价里的"语义不确定性"项只在置信度低于 0.6 时生效。
- 相机是前视针孔模型，`fovy` 与模型 XML 里的 `fovy` 一致：不一致时逆投影的
  内外参不匹配（`cam.fovy_deg` 默认 60.0，按实际模型改）。
- 训练用的扫描航线按场景的**实际可通行空间**给（`segdata.DEFAULT_SPECS` 里有
  各场景的几何约束说明）；换场景时起始点需重新核对。
