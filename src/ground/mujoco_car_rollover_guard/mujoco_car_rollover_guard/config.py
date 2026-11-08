"""mujoco_car_rollover_guard 配置与路径解析。

所有路径基于包自身位置（__file__）推导，不写死绝对路径，
整个模块可整体拷贝到任意工作空间 / 任意机器运行。
物理底盘参数（轮半径/轮距/质量）与 models/*.xml 保持一致。
"""
from pathlib import Path

PACKAGE_NAME = "mujoco_car_rollover_guard"
MODULE_DIR = Path(__file__).resolve().parent.parent


def data_dir(name):
    """定位 models/results 等数据目录。

    colcon 安装后模型与权重在 share/<包名>/<name>，源码树里则在同一层目录。
    先按 ament 索引找安装位置，找不到再退回包自身所在目录，所以同一份代码
    既能直接跑 main.py，也能在 ROS 里 ros2 run / ros2 launch 起来。
    """
    try:
        from ament_index_python.packages import get_package_share_directory
        installed = Path(get_package_share_directory(PACKAGE_NAME)) / name
        if installed.is_dir():
            return installed
    except Exception:
        pass
    for candidate in (MODULE_DIR / name, MODULE_DIR.parent / name):
        if candidate.is_dir():
            return candidate
    return MODULE_DIR / name


MODELS_DIR = data_dir("models")
CONFIG_DIR = MODULE_DIR / "config"
RESULTS_DIR = data_dir("results")


class Config:
    """集中管理仿真、传感器、控制与神经网络的全部可调参数。"""

    def __init__(self):
        # ---- 场景：三档载荷高度（质心越高，侧翻阈值越低） ----
        self.scenes = [
            ("car_scene_payload_low.xml", 0.16),
            ("car_scene_payload_mid.xml", 0.26),
            ("car_scene_payload_high.xml", 0.36),
        ]

        # ---- 仿真 ----
        self.timestep = 0.002
        self.control_hz = 50.0
        self.settle_s = 0.6

        # ---- 车辆运动学 / 控制 ----
        self.v_max = 2.0
        self.omega_max = 3.0
        self.wheel_r = 0.05
        self.track = 0.18
        self.max_wheel_omega = 60.0
        self.max_wheel_force = 8.0
        self.speed_keep_gain = 3.0

        # ---- 评价工况：三车速 × 转向斜坡 ----
        self.speeds = [0.8, 1.1, 1.4]
        self.steer_ramp_rate = 0.5
        self.throttle_ramp = 0.6
        self.accel_timeout_s = 4.5
        self.run_max_s = 9.0

        # ---- 侧翻判定与预测时域 ----
        self.rollover_tilt_deg = 55.0
        self.horizon_s = 0.5
        self.risk_threshold = 0.60
        self.guard_gain = 3.0
        self.guard_brake_risk = 0.85
        self.guard_ema = 0.7

        # ---- 神经网络 ----
        self.feature_dim = 9
        self.hidden = 16
        self.epochs = 220
        self.lr = 0.01
        self.batch = 64
        self.seed = 42

        # ---- 固定归一化尺度（物理量程，训练前即确定，不依赖数据统计） ----
        self.scales = (0.6, 0.6, 4.0, 4.0, 4.0, 15.0, 15.0, 4.0, 1.0)

    @property
    def dt(self):
        return 1.0 / self.control_hz

    @property
    def steps_per_control(self):
        return max(1, int(round(self.dt / self.timestep)))

    @property
    def horizon_steps(self):
        return max(1, int(round(self.horizon_s * self.control_hz)))

    def scene_path(self, name):
        return MODELS_DIR / name
