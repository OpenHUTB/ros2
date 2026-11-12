"""humanoid_mujoco_slam_nn_nav: MuJoCo 双足人形机器人的感知 / 规划 / 控制 / 端到端算法库。

模块划分::

    model.py     MuJoCo 场景与 MJCF 生成（机器人本体 + 三个评测场景）
    kinematics.py 双足逆运动学（骨盆/脚掌位姿 -> 关节角）
    gait.py      准静态步态生成器（键盘/速度指令 -> 关节目标）
    sim.py       MuJoCo 仿真内核（物理步进、传感器、里程计、重置）
    sensors.py   激光雷达 / 深度相机 / IMU 的物理仿真
    mapping.py   对数几率占据栅格地图
    slam.py      相关扫描匹配 + 地图更新
    planning.py  A* 全局规划与纯跟踪控制器
    nn.py        NumPy 实现的 MLP / 小 CNN 与 Adam 训练器
    nets.py      四个神经网络（感知 / 规划 / 控制 / 端到端）的推理封装
    metrics.py   导航与定位评价指标
"""

__version__ = "1.0.0"
