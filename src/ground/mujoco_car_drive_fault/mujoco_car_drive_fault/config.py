"""mujoco_car_drive_fault 配置与路径解析。

所有路径基于包自身位置（__file__）推导，不写死绝对路径，
整个模块可整体拷贝到任意工作空间 / 任意机器运行。
物理底盘参数（轮半径/轮距/质量）与 models/*.xml 保持一致。
"""
from pathlib import Path

PACKAGE_NAME = "mujoco_car_drive_fault"
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
        # ---- 场景：三档单侧轮毂拖滞程度（阻尼 N·m·s/rad） ----
        self.scenes = [
            ("car_scene_fault_light.xml", 0.08),
            ("car_scene_fault_mid.xml", 0.10),
            ("car_scene_fault_severe.xml", 0.12),
        ]
        self.fault_sides = ("left", "right")
        # 健康轮毂关节的基础阻尼（场景 <default> 里 joint 的 damping），
        # 注入拖滞时另一侧要恢复到这个值，否则"右轮失效"会把两轮都拖住
        self.healthy_damping = 0.05

        # ---- 仿真 ----
        self.timestep = 0.002
        self.control_hz = 50.0
        self.settle_s = 0.4

        # ---- 车辆运动学 / 控制 ----
        self.v_max = 2.0
        self.omega_max = 1.5
        self.wheel_r = 0.05
        self.track = 0.18
        self.max_wheel_omega = 60.0
        self.max_wheel_force = 8.0
        self.speed_keep_gain = 0.25   # 归一化油门 / (m/s) 的速度误差修正增益
        self.throttle_cap_rate = 0.6

        # ---- 评价工况：三档车速 ----
        self.speeds = [0.6, 0.9, 1.2]
        self.run_max_s = 6.0
        self.distance_s = 6.5          # 沿车道跑 6.5 m 结束

        # ---- 跑偏判定与预测时域 ----
        self.deviate_m = 0.50          # 横向偏差超过 0.5 m 记为跑偏失败
        self.horizon_s = 1.0
        self.risk_threshold = 0.60
        self.guard_brake_risk = 0.85   # 风险高于此值改判为“无法维持”，减速停车
        self.guard_ema = 0.7

        # ---- 补偿控制器（两侧对称 PD） ----
        self.comp_k_y = 0.9            # 横向偏差增益
        self.comp_k_psi = 1.2          # 航向增益
        self.comp_k_yaw = 1.0          # 偏航角速度增益
        self.comp_ki = 0.5             # 积分增益：吃掉恒定拖滞的稳态偏置
        self.comp_integ_limit = 0.6    # 积分限幅
        self.comp_limit = 0.45         # 补偿转向限幅（归一化）
        self.rule_y = 0.15             # 规则法起作用的横向偏差门限（m）
        self.rule_yaw_deg = 3.0        # 规则法起作用的航向偏角门限（deg）
        self.recover_y = 0.10          # 补偿保持到偏差回到 0.1 m 以内
        self.recover_yaw_deg = 1.5     # 且航向回到 1.5° 以内才松手

        # ---- 神经网络 ----
        self.feature_dim = 9
        self.hidden = 16
        self.epochs = 220
        self.lr = 0.01
        self.batch = 64
        self.seed = 42

        # ---- 固定归一化尺度（物理量程，训练前即确定，不依赖数据统计） ----
        self.scales = (1.0, 0.6, 1.5, 8.0, 1.5, 30.0, 30.0, 30.0, 30.0)

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
