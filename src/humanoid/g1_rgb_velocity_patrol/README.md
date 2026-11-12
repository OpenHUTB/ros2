# G1 图像速度巡检

验证直接由 RGB 图像预测速度的 G1 巡检策略，并提供近墙行为复测入口。

## 环境与运行

使用 Python 3.11。在模块目录运行 `python -m pip install -r requirements.txt`。

在模块目录运行 `python scripts/fetch_assets.py /path/to/unitree_rl_gym` 获取固定版本的 G1 资产。底层步态来自 Unitree 预训练策略。

```bash
python main.py --assets /path/to/unitree_rl_gym --record --output results
```

默认 `end2end` 模式直接读取模块自带的 `models/end2end.pt`（1,637,701 字节），拿到仓库和固定版本的 Unitree 仿真资产后即可运行。该模式只加载图像速度模型。`models/mask_controller.pt`（61,173 字节）提供给 `vision.py` 的感知分支复用。

闭环默认使用留出种子 7。运行 `python scripts/evaluate_near_obstacle.py --help` 查看近墙评估参数。策略输出高层速度，由预训练步态执行。

## ROS launch

在 ROS Noetic 源码空间运行，将路径替换为实际值。

```bash
source /opt/ros/noetic/setup.bash
export ROS_PACKAGE_PATH="$PWD:${ROS_PACKAGE_PATH:-}"
roslaunch g1_rgb_velocity_patrol main.launch sim_python:=/path/to/python sim_entry:=/path/to/main.py assets:=/path/to/unitree_rl_gym
```

## 大模型使用声明

使用 OpenAI Codex 辅助拆分、编程、验证和文档整理，提交者复核并对提交内容负责。
