"""mujoco_car_nn_avoidance 配置与路径解析。

所有路径基于包自身位置（__file__）推导，不写死绝对路径，
整个模块可整体拷贝到任意工作空间 / 任意机器运行。

物理底盘参数（轮径/轮距/质量/积分器）与 models/*.xml 保持一致，
并采用了经实际调通的稳定驱动方案（velocity 速度执行器 + implicitfast 积分，
见 car_env.py 与 XML 顶部注释）。
"""
from pathlib import Path

# 模块根目录：本包所在目录的上一级，即 src/ground/mujoco_car_nn_avoidance
MODULE_DIR = Path(__file__).resolve().parent.parent
MODELS_DIR = MODULE_DIR / "models"
CONFIG_DIR = MODULE_DIR / "config"
RESULTS_DIR = MODULE_DIR / "results"


class Config:
    """集中管理仿真、传感器、控制与神经网络的全部可调参数。"""

    def __init__(self):
        # ---- 传感器（前视激光射线扇面） ----
        # -60° ~ +60°，5° 间隔，共 25 束；0° 为车头正前方
        self.lidar_angles_deg = list(range(-60, 61, 5))
        self.lidar_range_m = 6.0          # 射线最大探测距离
        # 参与射线检测的 geom group 掩码（1=参与）：group0=障碍可见，
        # group1=地面/路面屏蔽，group2=自车屏蔽（与 XML 约定一致；
        # 车放 group2 是为了渲染可见——group3 会被 Renderer 默认屏蔽）
        self.lidar_geom_groups = (1, 0, 0, 1, 1, 1)

        # ---- 场景文件（多场景测试） ----
        self.scenes = [
            "car_scene_basic.xml",
            "car_scene_slalom.xml",
            "car_scene_dual.xml",
        ]

        # ---- 仿真 ----
        self.timestep = 0.002             # 物理步长 s（与 XML option 一致）
        self.control_hz = 50.0            # 决策/控制频率 Hz
        self.settle_s = 0.6               # 起步静置：先零轮速推进建立接触

        # ---- 车辆运动学 / 控制 ----
        self.v_max = 0.8                  # 最大目标线速度 m/s
        self.omega_max = 2.0              # 最大目标转向角速度 rad/s
        self.wheel_r = 0.05               # 轮半径 m（与 XML 一致）
        self.track = 0.25                 # 轮距 m（与 XML 左右轮中心距一致）
        self.max_wheel_omega = 60.0       # 轮角速度上限 rad/s（对应 ctrlrange）
        self.max_wheel_force = 8.0        # 单驱动轮出力上限 N（对应 XML forcerange）
        self.cruise_throttle = 0.85       # 巡航油门（0~1），空旷直行时较快，利于快速通过障碍路段
        self.goal_x = 6.0                 # 到达判定：车体 x 超过该值视为到达终点

        # ---- 神经网络 ----
        # 输入 = 25 束射线（归一化距离）+ 1 维航向偏差 yaw/pi（来自 IMU/里程计）
        self.input_dim = len(self.lidar_angles_deg) + 1   # 26
        self.hidden = 32
        self.output_dim = 2               # [steer, throttle]
        self.epochs = 300
        self.lr = 0.005
        self.batch = 64
        self.seed = 42

    # ---- 派生常量 ----
    @property
    def dt(self):
        return 1.0 / self.control_hz

    @property
    def steps_per_control(self):
        """一个控制周期内的物理步数。"""
        return max(1, int(round(self.dt / self.timestep)))

    def scene_path(self, name):
        return MODELS_DIR / name
