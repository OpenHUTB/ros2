# G1 CostNet 栅格规划对照

在独立栅格示例中比较 CostNet 辅助 A* 和曼哈顿 A* 的路径代价与搜索扩展数。

## 环境与运行

使用 Python 3.11。在模块目录运行 `python -m pip install -r requirements.txt`。

```bash
python main.py --output results
```

示例在一个合成栅格比较经典和神经辅助搜索，输出路线图和 JSON。网络仅处理相同 f 值的排序。可用 `--mode train_planner` 重训，用 `--mode plan_stdio` 接收占据栅格 JSON。

## ROS 1 Noetic 启动

本模块的 ROS 接口使用 ROS 1 Noetic（catkin、rospy）。在 Noetic 源码空间运行，将路径替换为实际值。

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="$PWD:${ROS_PACKAGE_PATH:-}"
roslaunch g1_costnet_grid_planner main.launch sim_python:=/path/to/python sim_entry:=/path/to/main.py
```

## 可复核结果

| 方法或指标 | 路径代价（格） | 扩展节点 |
| --- | ---: | ---: |
| Manhattan A* | 48 | 401 |
| 随仓库提供的 CostNet 辅助 A* | 48 | 49 |
| 40 张合成测试图的 CostNet MAE | 0.756 | — |

扩展数仅对应演示地图。推理按栅格各轴尺寸归一化相对坐标，各轴尺度至少为 32，以保持 32×32 训练输入及已有权重兼容。大图的搜索扩展数会随布局变化。

```bash
python main.py --mode train_planner --epochs 5 --samples 100 --seed 91 --weights results/trained
python main.py --mode plan_stdio --robot-radius 0.3 --stride 3 < request.json
```

请求 JSON 包含 `map`（行优先的一维占据数组）、`width`、`height`、`resolution`（米/格）、`origin`、`start`、`goal`（二维世界坐标）。可用 `robot_radius`、`stride` 字段覆盖命令行默认值。重复在同一 Python 进程调用时复用权重，文件更新后重新加载。

## 大模型使用声明

使用 OpenAI Codex 辅助拆分、编程、验证和文档整理，提交者复核并对提交内容负责。
