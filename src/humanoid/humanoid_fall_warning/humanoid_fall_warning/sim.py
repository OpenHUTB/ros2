"""sim.py —— 多模态跌倒预警仿真封装"""
from __future__ import annotations
import os
import numpy as np
try:
    import mujoco
except (ImportError, OSError, RuntimeError):
    mujoco = None

JOINT_NAMES = [
    "hip_roll_l", "hip_pitch_l", "knee_l", "ankle_pitch_l", "ankle_roll_l",
    "hip_roll_r", "hip_pitch_r", "knee_r", "ankle_pitch_r", "ankle_roll_r",
    "shoulder_pitch_l", "elbow_l", "shoulder_pitch_r", "elbow_r", "neck",
]

# 贴地起手姿态：脚掌平贴地面。reset() 必须从这里起手 —— 若直接把站姿目标写进
# qpos，脚会悬空 3~4 cm，落地冲击会把机器人推倒（与本模块同一机型在任务②
# 深度姿态模块里实测 553 步内跌倒）。
INIT_QPOS = np.array([
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)

# 站姿平衡（ankle strategy）：把「质心相对足底的偏移 + 根水平速度」折算成踝关节
# 目标角修正。纯关节 PD 是开环的，本机型 1.2 s 起就慢漂、3 s 最大倾角 89.8° 并
# 完全倒下（脚掌支撑面 0.22 m 对应倾覆角 9.6°，越过就必倒）。加这层反馈后
# 3 s 最大倾角 1.73°。增益沿用同一机型在任务②的 5×5 网格标定结果。
BALANCE_KP = 0.5
BALANCE_KD = 0.2

# 站姿目标按 ankle ≈ -(hip + knee) 配平后网格标定得到。原站姿
# hip=0.08 / knee=-0.16 / ankle=-0.08 不满足配平关系，无外力 3 s 内自身前倾
# 89.8° 倒地 —— 这会污染整个跌倒预警数据集（机器人不需要外力就会倒）。
DEFAULT_STAND_TARGET = np.array([
    0.0, 0.16, -0.08, 0.20, 0.0,
    0.0, 0.16, -0.08, 0.20, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)

# 紧急下蹲姿态：髋屈 + 膝屈 + 踝背屈。踝背屈（ankle_pitch 取正）把重心拉回
# 足底支撑面内，缺了它下蹲时脚会打滑前滑。
SQUAT_TARGET = np.array([
    0.0, 0.30, -0.45, 0.15, 0.0,
    0.0, 0.30, -0.45, 0.15, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)


class FallWarningSim:
    def __init__(self, model_path, kp=60.0, kd=6.0, image_h=64, image_w=64):
        if mujoco is None:
            raise RuntimeError("mujoco 未安装")
        with open(model_path, "r", encoding="utf-8") as f:
            xml_str = f.read()
        self.model = mujoco.MjModel.from_xml_string(xml_str)
        self.data = mujoco.MjData(self.model)
        self.kp, self.kd = kp, kd
        self.image_h, self.image_w = image_h, image_w
        self.qpos_js, self.qvel_js = 7, 6
        self._renderer = None
        self._rgb_renderer = None
        self._cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self._foot_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom_l")
        self.timestep = float(self.model.opt.timestep)
        self.reset()

    def reset(self):
        """回到贴地起手姿态。不能直接写 DEFAULT_STAND_TARGET，否则脚悬空。"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_js:self.qpos_js+15] = INIT_QPOS
        mujoco.mj_forward(self.model, self.data)

    def step(self, target=None, balance=True):
        """推进一步。

        Args:
            target: 15 维关节目标角，None 表示维持站姿。
            balance: 是否叠加站姿平衡（ankle strategy）修正。默认开启；关掉就是
                纯开环 PD，即原来 3 s 自己倒下的状态，只在做对照实验时用。
        """
        if target is None:
            target = DEFAULT_STAND_TARGET
        if balance:
            target = np.array(target, dtype=np.float64, copy=True)
            com = self.data.subtree_com[0]
            foot = self.data.geom_xpos[self._foot_id]
            ex = float(com[0] - foot[0])
            ey = float(com[1] - foot[1])
            # 质心前移 → 踝背屈把重心推回去；角速度项做阻尼
            u = BALANCE_KP * ex + BALANCE_KD * float(self.data.qvel[0])
            target[3] += u
            target[8] += u
            # 侧向：质心右偏 → 左踝外翻、右踝内翻
            v = BALANCE_KP * ey + BALANCE_KD * float(self.data.qvel[1])
            target[4] -= v
            target[9] += v
        qj = self.data.qpos[self.qpos_js:self.qpos_js+15]
        qv = self.data.qvel[self.qvel_js:self.qvel_js+15]
        tau = np.clip(self.kp*(target-qj) - self.kd*qv, -280.0, 280.0)
        self.data.ctrl[:] = tau
        mujoco.mj_step(self.model, self.data)

    def apply_push(self, fxyz, duration_steps=40, balance=True):
        """对躯干施加水平推力若干步。"""
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        fx, fy, fz = fxyz
        for _ in range(max(1, int(duration_steps))):
            self.data.xfrc_applied[bid, :3] = [fx, fy, fz]
            self.step(balance=balance)
        self.data.xfrc_applied[bid, :3] = [0.0, 0.0, 0.0]

    @property
    def root_z(self): return float(self.data.qpos[2])

    def fallen(self, z=0.6): return self.root_z < z

    def torso_pose(self):
        """返回躯干 (pitch, roll) 弧度，ZYX 欧拉角顺序。"""
        qw, qx, qy, qz = self.data.qpos[3:7]
        # roll (绕 x 轴)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))
        # pitch (绕 y 轴)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = float(np.arcsin(np.clip(sinp, -1, 1)))
        return pitch, roll

    def render_front(self, gray=True):
        """头部前视相机（depth/RGB 二分类的输入源）。"""
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.image_h, self.image_w)
        self._renderer.update_scene(self.data, camera=self._cam_id)
        img = self._renderer.render()
        if gray:
            g = 0.299*img[..., 0] + 0.587*img[..., 1] + 0.114*img[..., 2]
            return g.astype(np.float32)/255.0
        return img

    def render_rgb(self, height=240, width=320, camera="overview"):
        """第三人称 RGB 渲染，用于录真实演示动图。

        默认用 overview 相机；front_camera 装在头部朝下，单独用它录出来的画面
        只有躯干顶面，看不出站姿和倒地。
        """
        if self._rgb_renderer is None:
            self._rgb_renderer = mujoco.Renderer(self.model, height, width)
        cam_id = -1
        if camera:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
        self._rgb_renderer.update_scene(self.data, camera=cam_id)
        return self._rgb_renderer.render()

    def close(self):
        for r in (self._renderer, self._rgb_renderer):
            if r:
                try: r.close()
                except Exception: pass
        self._renderer = None
        self._rgb_renderer = None
