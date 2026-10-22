# 无人机定高日志误差与抗扰恢复时间评估

本模块提供独立 Python 入口与 ROS 2 launch 入口。算法、参数、结果和接口详见[完整说明](../../../docs/air/uav_hover_log_eval.md)。

## 环境与运行

Python 3.10 或 3.11；ROS 2 部分使用 Ubuntu 22.04 / Humble。先在模块目录执行：

```bash
python3 -m pip install -r requirements.txt
python3 main.py
```

ROS 2 使用系统 Python。构建工作空间时只选择本包，避免构建仓库内其他独立实验：

```bash
source /opt/ros/humble/setup.bash
cd ~/ros2
colcon build --packages-select uav_hover_log_eval
source install/setup.bash
ros2 launch uav_hover_log_eval main.launch.py
```

首次安装可用系统 Python 创建 `--system-site-packages` 虚拟环境，保留系统 ROS 包可见性。输出目录默认是 `outputs/`，不覆盖随包附带的数据。

## 验证范围

离线算法在 Windows / Python 3.11 运行；ROS 2 安装、launch 与实际话题收发在 Ubuntu 22.04 / Humble 容器检查。容器验证不等同于另一位使用者的独立机器验收，也没有验证 Ubuntu 20.04。

## 使用声明

使用 ChatGPT / Codex 辅助实现与文档整理；提交者负责审阅、理解并对提交内容负责。
