# carla_mapping_navigation —— CARLA 激光雷达建图与神经网络规划导航

> 机器人操作系统及应用 · 课程作业三
>
> 在虚拟环境中实现对机器人的**建图和导航**（导航与 SLAM 同步仿真）。
> **规划算法为神经网络**（纯 numpy 实现，含前向传播与反向传播，无需 TensorFlow/PyTorch）。

## 1. 功能

| 环节 | 输入 | 算法 | 输出 |
|---|---|---|---|
| 建图 | 32 线激光雷达点云 | 占用栅格 + 贝叶斯对数几率更新 | `nav_msgs/OccupancyGrid` 2D 地图 |
| 规划 | 目标方位 + 障碍左右分布 + 最近距离 | **神经网络** MLP `[5, 64, 2]` | (油门, 转向) |

建图体现 SLAM 的**边动边建图**理念：命中点标占据、射线沿途标空闲，实时更新地图。

## 2. 目录结构

```
src/ground/carla_mapping_navigation/
├── main.py                                   # 主入口（train / run / headless 三种模式）
├── main.sh / main.bat                        # 课程约定的 main.* 一键运行脚本
├── carla_mapping_navigation/
│   ├── __init__.py
│   ├── nn_models.py                          # 纯 numpy 神经网络库
│   ├── carla_common.py                       # CARLA Python API 封装
│   └── mapping_navigation_node.py            # ROS 2 节点（建图 + NN 规划）
├── launch/
│   ├── main.launch.py                        # ROS 2 Humble
│   └── main.launch                           # ROS 1 Noetic
├── config/
│   ├── sim_params.yaml                       # 仿真参数
│   └── mapping.rviz                          # RViz 可视化配置（现成可用）
├── resource/carla_mapping_navigation
├── setup.py / setup.cfg / package.xml
├── requirements.txt
└── test/test_mapping_logic.py                # 单元测试（12 项，无需 CARLA）
```

## 3. 快速开始

```bash
# ① 离线训练规划神经网络（不需要 CARLA）
python3 main.py --mode train --epochs 300 --out models/nn_plan.json
#    预期：规划 NN 训练 MSE ≈ 0.006

# ② 离线取证（不需要 CARLA，也不需要图形界面）
python3 main.py --headless --demo --epochs 300 --sim_time 60 --save_dir ~/shots
#    导出 5 张图：占用栅格地图 / 渐进建图快照 ×3 / 训练损失曲线

# ③ 在线建图 + NN 导航（需要 CARLA 服务端）
python3 main.py --mode run --host <宿主机IP> --model models/nn_plan.json \
        --goal "20,8" --sim_time 40

# ④ ROS 2 / ROS 1
ros2 launch carla_mapping_navigation main.launch.py host:=<宿主机IP> goal:="20,8"
roslaunch carla_mapping_navigation main.launch host:=<宿主机IP> goal:="20,8"
```

一键脚本：`bash main.sh --host <宿主机IP>` 或 Windows `main.bat --mode train`。

在 RViz 中查看占用栅格地图：

```bash
ros2 run rviz2 rviz2 -d $(ros2 pkg prefix carla_mapping_navigation)/share/carla_mapping_navigation/config/mapping.rviz
```

## 4. 测试

```bash
python3 test/test_mapping_logic.py     # 12 项全部通过，无需 CARLA
```

其中 `test_grid_contains_vehicle_start` 是**回归测试**：早期版本栅格固定以世界原点为中心、
只覆盖 ±30 m，而车辆出生点在 (36,-5)，导致车辆落在栅格之外、建图结果恒为空。
该测试确保这一缺陷不会复现。

## 5. 实测指标（离线取证模式）

| 指标 | 数值 |
|---|---|
| 规划 NN 训练 MSE | 0.00596 |
| 占据格数 / 空闲格数 | 1380 / 9786 |
| 已知区域比例 | 29.1% |
| 导航最近距离 | 1.458 m（阈值 1.5 m） |
| 到达耗时 | 10.2 s |

渐进建图实测：已知区域 0.2% → 19.7% → 25.3% → 27.7%。

## 6. 与已有示例的关系

本模块是 [`pcl_recorder`（点云地图创建）](https://openhutb.github.io/ros2/ground/pcl_recorder/)
示例的**拓展**：已有示例只把点云存盘，本模块改为概率化占用栅格建图并补充了神经网络规划导航。
**CARLA 服务端启动、宿主机 IP 与端口查看等通用配置步骤不在此重复**，
详见 [设置并连接到 Carla 模拟器](https://openhutb.github.io/ros2/set_up_and_connect_to_carla/)
与 [模块文档](https://openhutb.github.io/ros2/ground/carla_mapping_navigation/)。
