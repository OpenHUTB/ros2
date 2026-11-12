"""纯 MuJoCo 仿真核心（无 DDS / 无手柄 / 无 ROS）。

上游 lac-code 的仿真依赖 ``unitree_sdk2py`` 走 CycloneDDS 与一个 Xbox 手柄，
评测机既没有手柄也起不了 DDS 域。本模块把那一层剥掉，直接在 Python 里完成
「读 sensordata -> 算 PD 力矩 -> 写 ctrl -> mj_step」的闭环：

    sensordata[0:nu]           关节位置 q（按 actuator 顺序，即 SDK 槽位顺序）
    sensordata[nu:2nu]         关节速度 dq
    sensordata[2nu:3nu]        关节力矩 tau
    sensor_adr['imu_quat']     IMU 姿态四元数 (w, x, y, z)
    sensor_adr['imu_gyro']     IMU 角速度

与上游一致的两处物理设置必须保留，否则站立时会持续脚底蠕滑：
  * ``opt.cone = mjCONE_ELLIPTIC``（真圆 Coulomb 摩擦锥）
  * ``opt.impratio = 30``（摩擦约束相对法向约束的阻抗比）
"""

from __future__ import annotations

import os
import shutil
import tempfile
from dataclasses import dataclass
from typing import Optional

import numpy as np

from .lac.constants import DEFAULT_JOINT_POS_23, JOINT_23_TO_SDK, NUM_JOINTS

# G1 站立时骨盆高度（与训练/部署的默认指令高度一致）
DEFAULT_BASE_HEIGHT = 0.74
# 4 个 23DOF 用不到的 SDK 槽位：waist_roll / waist_pitch / l_wrist_pitch / l_wrist_yaw
INACTIVE_SDK_SLOTS = tuple(i for i in range(29) if i not in set(JOINT_23_TO_SDK))


@dataclass
class JointState:
    """一帧本体状态，全部按 JOINT_NAMES_23DOF 顺序。"""

    q: np.ndarray        # (23,) 关节位置 rad
    dq: np.ndarray       # (23,) 关节速度 rad/s
    quat: np.ndarray     # (4,)  (w, x, y, z)
    gyro: np.ndarray     # (3,)  rad/s
    tau: np.ndarray      # (23,) 关节力矩 N·m


def _has_non_ascii(path: str) -> bool:
    return any(ord(c) > 127 for c in path)


def resolve_model_dir(explicit: Optional[str] = None) -> str:
    """定位 G1 模型目录（含 scene_23dof.xml 与 meshes/）。

    优先级：显式参数 > 环境变量 > ament share > 源码树相对路径。
    MuJoCo 的 C 侧不接受非 ASCII 路径，含中文时复制到临时目录再加载。
    """
    candidates = []
    if explicit:
        candidates.append(explicit)
    env = os.environ.get('HUMANOID_G1_MODEL_DIR')
    if env:
        candidates.append(env)

    try:  # 已安装：install/<pkg>/share/<pkg>/assets/g1
        from ament_index_python.packages import get_package_share_directory
        candidates.append(os.path.join(
            get_package_share_directory('humanoid_g1_compliance'), 'assets', 'g1'))
    except Exception:  # noqa: BLE001  未 source 或没装 ament
        pass

    # 源码树：<pkg>/humanoid_g1_compliance/sim_core.py -> <pkg>/assets/g1
    here = os.path.dirname(os.path.abspath(__file__))
    candidates.append(os.path.join(os.path.dirname(here), 'assets', 'g1'))

    for path in candidates:
        if path and os.path.isfile(os.path.join(path, 'scene_23dof.xml')):
            return _ascii_copy(path)
    raise FileNotFoundError(
        '找不到 G1 模型目录（需含 scene_23dof.xml）。已尝试：\n  '
        + '\n  '.join(str(c) for c in candidates))


def _ascii_copy(path: str) -> str:
    if not _has_non_ascii(os.path.abspath(path)):
        return path
    dst = os.path.join(tempfile.gettempdir(), 'humanoid_g1_assets')
    stamp = os.path.join(dst, '.stamp')
    src_stamp = str(os.path.getmtime(os.path.join(path, 'scene_23dof.xml')))
    if os.path.isdir(dst) and os.path.exists(stamp):
        with open(stamp, 'r', encoding='utf-8') as fh:
            if fh.read().strip() == src_stamp:
                return dst
    shutil.rmtree(dst, ignore_errors=True)
    shutil.copytree(path, dst)
    with open(os.path.join(dst, '.stamp'), 'w', encoding='utf-8') as fh:
        fh.write(src_stamp)
    return dst


class G1Sim:
    """G1 23-DOF 的 MuJoCo 闭环封装。"""

    def __init__(self, model_dir: Optional[str] = None, timestep: float = 0.005,
                 impratio: float = 30.0, gravity: bool = True):
        import mujoco
        self._mj = mujoco

        self.model_dir = resolve_model_dir(model_dir)
        self.model = mujoco.MjModel.from_xml_path(
            os.path.join(self.model_dir, 'scene_23dof.xml'))
        self.data = mujoco.MjData(self.model)

        self.model.opt.timestep = timestep
        if impratio > 0:
            self.model.opt.impratio = impratio
            self.model.opt.cone = mujoco.mjtCone.mjCONE_ELLIPTIC
        if not gravity:
            self.model.opt.gravity[:] = 0.0

        self.nu = int(self.model.nu)
        self.timestep = float(timestep)

        # sensor 地址按名字取，避免依赖声明顺序
        self._adr = {}
        for name in ('imu_quat', 'imu_gyro', 'imu_acc'):
            sid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SENSOR, name)
            if sid < 0:
                raise RuntimeError(f'模型缺少 sensor {name}')
            self._adr[name] = int(self.model.sensor_adr[sid])

        self.pelvis_id = self._body_id('pelvis')
        self.torso_id = self._body_id('torso_link')
        self._joint_qadr = np.array(
            [int(self.model.jnt_qposadr[
                self.model.actuator_trnid[i, 0]]) for i in range(self.nu)],
            dtype=np.int64)

        self.reset()

    # ---- 基础 ----
    def _body_id(self, name: str) -> int:
        bid = self._mj.mj_name2id(self.model, self._mj.mjtObj.mjOBJ_BODY, name)
        if bid < 0:
            raise RuntimeError(f'模型缺少 body {name}')
        return int(bid)

    def body_id(self, name: str) -> int:
        return self._body_id(name)

    def reset(self, base_height: float = DEFAULT_BASE_HEIGHT) -> None:
        """复位到训练默认站姿：直立、双脚着地、关节在 default pose。"""
        self._mj.mj_resetData(self.model, self.data)
        self.data.qpos[0:3] = (0.0, 0.0, base_height)
        self.data.qpos[3:7] = (1.0, 0.0, 0.0, 0.0)
        for i, sdk in enumerate(JOINT_23_TO_SDK):
            self.data.qpos[self._joint_qadr[sdk]] = DEFAULT_JOINT_POS_23[i]
        self.data.ctrl[:] = 0.0
        self._mj.mj_forward(self.model, self.data)

    # ---- 读写 ----
    def read(self) -> JointState:
        sd = self.data.sensordata
        q29 = sd[:self.nu]
        dq29 = sd[self.nu:2 * self.nu]
        tau29 = sd[2 * self.nu:3 * self.nu]
        idx = np.asarray(JOINT_23_TO_SDK, dtype=np.int64)
        a = self._adr['imu_quat']
        g = self._adr['imu_gyro']
        return JointState(
            q=q29[idx].copy(),
            dq=dq29[idx].copy(),
            quat=sd[a:a + 4].copy(),
            gyro=sd[g:g + 3].copy(),
            tau=tau29[idx].copy(),
        )

    def apply_pd(self, q_target: np.ndarray, kp: np.ndarray,
                 kd: np.ndarray) -> None:
        """把位置指令写成力矩：tau = kp*(q_t - q) - kd*dq。

        23 个活跃槽位之外（waist_roll / waist_pitch / 双侧腕 pitch、yaw）保持 0 力矩，
        与上游部署一致——这些关节在策略里不参与控制。
        """
        st = self.read()
        ctrl = np.zeros(self.nu, dtype=np.float64)
        for i, sdk in enumerate(JOINT_23_TO_SDK):
            ctrl[sdk] = kp[i] * (q_target[i] - st.q[i]) - kd[i] * st.dq[i]
        for slot in INACTIVE_SDK_SLOTS:
            if slot < self.nu:
                ctrl[slot] = 0.0
        self.data.ctrl[:] = ctrl

    def apply_force(self, body_id: int, force, torque=None) -> None:
        """在指定 body 的质心坐标系原点施加外力/力矩（世界坐标）。"""
        self.data.xfrc_applied[body_id, :3] = np.asarray(force, dtype=np.float64)
        if torque is not None:
            self.data.xfrc_applied[body_id, 3:6] = np.asarray(torque, dtype=np.float64)

    def clear_forces(self) -> None:
        self.data.xfrc_applied[:] = 0.0

    def step(self, n: int = 1) -> None:
        for _ in range(n):
            self._mj.mj_step(self.model, self.data)

    # ---- 派生量 ----
    def projected_gravity(self, quat: np.ndarray) -> np.ndarray:
        """世界系重力方向在机体系的投影；直立时 ≈ (0, 0, -1)。"""
        q = np.asarray(quat, dtype=np.float64)
        n = float(np.linalg.norm(q))
        if n < 1e-9:
            return np.array([0.0, 0.0, -1.0])
        w, x, y, z = q / n
        # 用四元数把 (0,0,-1) 旋转到机体系（即取逆旋转）
        gx = 2.0 * (x * z - w * y)
        gy = 2.0 * (y * z + w * x)
        gz = 1.0 - 2.0 * (x * x + y * y)
        return np.array([gx, gy, -gz])

    def pelvis_pos(self) -> np.ndarray:
        return self.data.xpos[self.pelvis_id].copy()

    def body_pos(self, body_id: int) -> np.ndarray:
        return self.data.xpos[body_id].copy()

    def total_mass(self) -> float:
        return float(self.model.body_mass.sum())


def default_pose_23() -> np.ndarray:
    return np.asarray(DEFAULT_JOINT_POS_23, dtype=np.float64)


__all__ = [
    'G1Sim', 'JointState', 'DEFAULT_BASE_HEIGHT', 'NUM_JOINTS',
    'resolve_model_dir', 'default_pose_23', 'INACTIVE_SDK_SLOTS',
]
