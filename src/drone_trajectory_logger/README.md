# 新增无人机飞行日志与状态识别 ROS 2 封装模块
基于 ROS 2 实现的无人机飞行轨迹识别节点，订阅飞行指令并实时绘图。

## 包含文件
* `main.py`: 发布节点
* `trajectory_analyzer.py`: 识别与绘图节点
* `launch/`: 启动文件
## 核心代码文件清单

本模块包含以下核心文件，均已提交至仓库：

```text
src/
└── drone_trajectory_logger/
    ├── main.py                 # 轨迹发布节点（入口）
    ├── trajectory_analyzer.py  # 轨迹识别与绘图节点
    ├── setup.py                # ROS 2 功能包配置文件
    ├── package.xml             # 依赖声明文件
    └── launch/
        └── main.launch.py      # 一键启动文件
```

## 快速开始
```bash
colcon build --packages-select drone_trajectory_logger
source install/setup.bash
ros2 launch drone_trajectory_logger main.launch.py
```

(详细原理与实验步骤请参考仓库 docs/ 目录下的说明文档)
