"""
common.py —— 包内共享工具：路径解析、水下动力学、时变洋流。

为什么单独抽出来：
  sim_node（ROS 实时仿真）、cruise_net（纯 Python 离线采集/训练/闭环评估）
  两边都要用同一套水下动力学与同一套模型加载逻辑。放在这里保证
  "bash main.sh 离线跑的闭环" 与 "ros2 launch 在线跑的仿真" 物理一致，
  避免两边各写一份漂移。
"""
import os
from pathlib import Path

import numpy as np


def get_package_root() -> Path:
    """
    返回本包源码根目录（含 models/、config/、weights/、data/）。
    为什么需要它：
      colcon install 后模型会被拷到 share/<pkg>/，但开发期用 --symlink-install
      直接指向源码目录；统一在这里解析，离线脚本和 ROS 节点都能找到模型。
    """
    # common.py 在 <root>/mjc_rov_current_teleop/common.py
    return Path(__file__).resolve().parent.parent


def find_model_path(explicit: str = "") -> str:
    """按 显式参数 → 源码 models/ → install 后 share/ 的顺序找 MJCF。"""
    if explicit:
        p = Path(explicit)
        if p.exists():
            return str(p)
    root = get_package_root()
    cand = root / "models" / "rov_pond.xml"
    if cand.exists():
        return str(cand)
    # install 后路径：ament 把 models 拷到 share/mjc_rov_current_teleop/models/
    share = Path("/opt/ros/humble/share/mjc_rov_current_teleop/models/rov_pond.xml")
    if share.exists():
        return str(share)
    raise FileNotFoundError(
        "找不到 MuJoCo 模型 rov_pond.xml。请确认包目录结构完整，"
        "或通过参数 model_path 指定模型绝对路径。"
    )


def find_weights_path(explicit: str = "") -> Path:
    """权重路径：显式参数优先，否则用 包根/weights/cruise_mlp.pt。"""
    if explicit:
        return Path(explicit)
    return get_package_root() / "weights" / "cruise_mlp.pt"


def set_mujoco_gl_backend() -> None:
    """
    在 import mujoco 之前强制 OSMesa 离屏渲染后端。
    为什么必须在 import 前：MuJoCo 首次初始化时读取 MUJOCO_GL 环境变量，
    之后再改不生效；EGL/GLFW 在无显卡虚拟机上不可用，OSMesa 是唯一稳的。
    """
    os.environ.setdefault("MUJOCO_GL", "osmesa")
    os.environ.setdefault("PYOPENGL_PLATFORM", "osmesa")


class WaterDynamics:
    """
    水下 6-DOF 简化动力学：
      浮力补偿 + 线性/角度黏性阻尼 + 横滚/俯仰姿态自稳 + 时变洋流扰动。

    为什么不用 MuJoCo 自带 fluid 选项：
      fluid 参数在不同 MuJoCo 小版本行为差异大，且 3.1.x 对 freejoint 的
      浮力/阻尼建模不如直接施加 qfrc_applied 可控；课程环境无 GPU，
      用代码层施加既稳又能精确控制参数（全部走 ROS declare_parameter）。

    力矩/力都施加在基座坐标系（freejoint 的前 6 维 qfrc_applied）。
    """

    def __init__(self, params: dict):
        # params 里的键与 config/*.yaml 一一对应
        self.buoyancy_z = float(params.get("buoyancy_z", 0.6))
        self.damping_lin = float(params.get("damping_lin", 1.2))
        self.damping_ang = float(params.get("damping_ang", 0.8))
        self.stab_roll = float(params.get("stab_roll", 4.0))
        self.stab_pitch = float(params.get("stab_pitch", 4.0))
        self.current_amp_x = float(params.get("current_amp_x", 0.15))
        self.current_amp_y = float(params.get("current_amp_y", 0.10))
        self.current_w_x = float(params.get("current_w_x", 0.30))
        self.current_w_y = float(params.get("current_w_y", 0.23))
        self.max_force_x = float(params.get("max_force_x", 2.0))
        self.max_force_y = float(params.get("max_force_y", 2.0))
        self.max_torque_yaw = float(params.get("max_torque_yaw", 0.5))

    def current_force(self, t: float) -> np.ndarray:
        """
        时变洋流扰动：Fx=ax*sin(wx*t)，Fy=ay*sin(wy*t)。
        为什么用两个不同频率：让扰动不可被单一正弦补偿器完全抵消，
        MLP 巡航策略必须学习对扰动的鲁棒跟随，而不是简单前馈。
        """
        fx = self.current_amp_x * np.sin(self.current_w_x * t)
        fy = self.current_amp_y * np.sin(self.current_w_y * t)
        return np.array([fx, fy, 0.0, 0.0, 0.0, 0.0])

    def apply(self, data, t: float, body_idx: int, twist_cmd: np.ndarray) -> None:
        """
        在 data.qfrc_applied 上叠加：浮力 + 阻尼 + 姿态自稳 + 洋流 + 用户推力。
        twist_cmd: 6 维 [vx_cmd, vy_cmd, 0, 0, 0, yaw_rate_cmd]（期望速度指令）。
        实现上用比例环节把期望速度映射成推力，再叠加洋流，等价于底层 PD。
        """
        # 读取当前速度（自由联合：qvel 前 6 维 = [vx,vy,vz,wx,wy,wz]）
        qvel = data.qvel[:6]
        # 读取当前姿态（quat 在 qpos[7:11]），把它拆成 roll/pitch 用于自稳
        quat = data.qpos[3:7]
        roll, pitch = quat_to_rp(quat)

        F = np.zeros(6)
        # 1) 浮力补偿：向上恒力，抵消一部分重力（gravity 已在 xml 设为 -2）
        F[2] += self.buoyancy_z
        # 2) 黏性阻尼：与速度反向
        F[0] -= self.damping_lin * qvel[0]
        F[1] -= self.damping_lin * qvel[1]
        F[2] -= self.damping_lin * qvel[2]
        F[3] -= self.damping_ang * qvel[3]
        F[4] -= self.damping_ang * qvel[4]
        F[5] -= self.damping_ang * qvel[5]
        # 3) 横滚/俯仰自稳：姿态误差 * 增益，把 ROV 拉回水平
        F[3] += -self.stab_roll * roll
        F[4] += -self.stab_pitch * pitch
        # 4) 洋流扰动
        F += self.current_force(t)
        # 5) 用户期望速度 → 推力（速度闭环 P 控制，限幅）
        # 为什么是闭环：F = kp*(v_desired - v_actual)，让 ROV 真正跟踪期望速度，
        # 而不是开环加到终端速度导致过冲。kp 取大些保证跟踪刚度。
        kp = 8.0
        F[0] += np.clip(kp * (twist_cmd[0] - qvel[0]), -self.max_force_x, self.max_force_x)
        F[1] += np.clip(kp * (twist_cmd[1] - qvel[1]), -self.max_force_y, self.max_force_y)
        F[5] += np.clip(kp * (twist_cmd[5] - qvel[5]), -self.max_torque_yaw, self.max_torque_yaw)

        data.qfrc_applied[:] = F


def quat_to_rp(q: np.ndarray):
    """
    四元数 -> roll, pitch（小角度近似）。
    为什么只要 roll/pitch：ROV 的横滚/俯仰需要自稳，偏航 yaw 不做自稳
    （巡航本来就要转航向），所以这里不解析 yaw。
    """
    w, x, y, z = q[0], q[1], q[2], q[3]
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -0.999, 0.999))
    return roll, pitch
