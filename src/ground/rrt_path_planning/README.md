# RRT 快速随机扩展树路径规划仿真

## 简介

本模块在二维栅格空间中实现 **快速随机扩展树（Rapidly-exploring Random Tree, RRT）** 路径规划算法，并用 pygame 进行可视化演示。RRT 是一类基于随机采样的增量式规划方法，适用于高维空间与复杂障碍环境，因其实现简洁、对障碍形状不敏感而被广泛用于机器人运动规划。

模块从起点出发，每步在自由空间随机采样一个点，找到树中离它最近的节点，朝采样点方向扩展一个固定步长的新节点；若新连线不与障碍相交则加入树。以小概率直接朝目标采样以加速收敛，当新节点落入目标半径内即规划成功，再沿父节点链回溯出路径。

## 文件结构

```
rrt_path_planning/
├── main.py / main.sh / main.bat   # 三入口
├── launch/main.launch.py          # ROS 2 launch
├── config/params.yaml             # 参数配置
├── test/test_rrt.py               # 离线单元测试（13 项）
├── requirements.txt               # pygame>=2.0
├── package.xml / setup.py / setup.cfg / resource/   # ROS 2 包
└── rrt_path_planning/
    ├── config.py                  # 仿真参数
    ├── map.py                     # 障碍地图与线段-矩形碰撞检测
    ├── rrt.py                     # RRT 核心：采样、最近邻、扩展、回溯
    └── main.py                    # pygame 可视化与录制
```

## 运行环境

- Python 3.8+
- pygame >= 2.0
- PIL（Pillow，仅录制 gif 时需要）

## 运行步骤

在 `src/ground/rrt_path_planning/` 目录下执行：

```bash
# 安装依赖
pip install -r requirements.txt

# 交互模式：实时显示树扩展过程
python main.py

# 录制模式：离屏渲染并生成演示 gif（适合无显示设备环境）
python main.py --record --output rrt_demo.gif

# Windows
main.bat
# Linux / macOS
./main.sh
```

### ROS 2 launch 启动

```bash
# 在 ROS 2 工作空间中编译后
ros2 launch rrt_path_planning main.launch.py
```

### 单元测试

```bash
python -m pytest test/test_rrt.py -v
```

13 项测试覆盖：障碍点判定、线段-矩形碰撞检测（Liang-Barsky）、最近邻查找、步长受限扩展、自由空间可达性、起点被封死的不可达判定、路径回溯顺序、路径长度计算、随机种子可复现性。

## 运行效果

默认障碍布局下（4 个矩形障碍），种子 42 的 RRT 在约 200~400 次迭代内找到无碰撞路径，路径长度约 1050~1150 像素。

![RRT 规划演示](../img/ground/rrt_path_planning/rrt_demo.gif)

## 已知限制

- 纯二维几何演示，不接真实激光雷达与代价地图，障碍为已知矩形。
- RRT 为单次规划，不保证路径最优（可用 RRT* 改进）。
- 局部极小区域可能需要较多迭代；本模块以 8% 目标偏置采样缓解。
- 路径为折线，未做平滑或动力学约束处理。

## 大模型使用声明

本模块的源代码与文档在编写过程中使用了大语言模型辅助生成与排版，作者对提交内容的正确性与完整性负全部责任。算法实现、参数调优、单元测试与运行验证均由作者完成并实测通过。
