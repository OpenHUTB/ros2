"""sim.py —— 深度图姿态估计用仿真封装"""
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
# 贴地起手姿态：脚掌平贴地面。reset() 必须从这里起手，若直接把站姿目标写进 qpos，
# 脚会悬空 3~4 cm，落地冲击会把机器人推倒（实测 553 步内跌倒）。
INIT_QPOS = np.array([
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, 0.08, -0.16, -0.08, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)

# 站姿平衡（ankle strategy）：把「质心相对足底的偏移 + 根水平速度」折算成踝关节
# 目标角修正。纯关节 PD 是开环的，这个模型 1.5 s 起就慢漂、3 s 最大倾角 59.8°
# （超过脚掌支撑面 0.22 m 对应的 9.6° 倾覆角就必然倒）；加这层反馈后 3 s 最大
# 倾角 1.73°。增益由 5×5 网格标定，符号为正（质心前移 → 踝背屈）。
BALANCE_KP = 0.5
BALANCE_KD = 0.2
# 站姿目标按 ankle ≈ -(hip + knee) 配平后网格标定得到（60 组粗扫 + 100 组精扫，
# 判据是 3 s 无扰动最大倾角）。最优解 hip=0.16 / knee=-0.08 / ankle=0.20：
# 3 s 最大倾角 1.56°，抗推存活步数是原站姿的 2 倍左右。
# 原站姿 hip=0.08 / knee=-0.16 / ankle=-0.08 不满足配平关系，无外力 1.4 s 内
# 自身前倾 86.9° 而倒下。
DEFAULT_STAND_TARGET = np.array([
    0.0, 0.16, -0.08, 0.20, 0.0,
    0.0, 0.16, -0.08, 0.20, 0.0,
    0.0, -1.4, 0.0, -1.4, 0.0,
], dtype=np.float64)


class DepthPoseSim:
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
        self._cam_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_CAMERA, "front_camera")
        self._foot_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "foot_geom_l")
        self.reset()

    def reset(self):
        """回到贴地起手姿态。注意不能直接写 DEFAULT_STAND_TARGET，否则脚悬空。"""
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_js:self.qpos_js+15] = INIT_QPOS
        mujoco.mj_forward(self.model, self.data)

    def step(self, target=None, balance=True):
        """推进一步。

        Args:
            target: 15 维关节目标角，None 表示维持站姿。
            balance: 是否叠加站姿平衡（ankle strategy）修正。默认开启，关掉就是
                纯开环 PD —— 那正是原来 3 s 自己倒下的状态，只在做对照实验时用。
        """
        if target is None:
            target = DEFAULT_STAND_TARGET
        if balance:
            target = np.array(target, dtype=np.float64, copy=True)
            com = self.data.subtree_com[0]
            foot = self.data.geom_xpos[self._foot_id]
            ex = float(com[0] - foot[0])
            ey = float(com[1] - foot[1])
            # 质心前移 → 踝背屈（正向）把重心推回去；角速度项做阻尼
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

    def apply_push(self, fxyz, duration_steps=40):
        bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "torso")
        fx, fy, fz = fxyz
        for _ in range(max(1, int(duration_steps))):
            self.data.xfrc_applied[bid, :3] = [fx, fy, fz]
            self.step()
        self.data.xfrc_applied[bid, :3] = [0.0, 0.0, 0.0]

    @property
    def root_z(self):
        return float(self.data.qpos[2])

    def fallen(self, z_thresh=0.6):
        return self.root_z < z_thresh

    def torso_pose(self):
        """返回躯干 (pitch, roll, yaw) 弧度，ZYX欧拉角顺序。"""
        qw, qx, qy, qz = self.data.qpos[3:7]
        # roll (绕x轴)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = float(np.arctan2(sinr_cosp, cosr_cosp))
        # pitch (绕y轴)
        sinp = 2 * (qw * qy - qz * qx)
        pitch = float(np.arcsin(np.clip(sinp, -1, 1)))
        # yaw (绕z轴)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))
        return pitch, roll, yaw

    def render_depth(self):
        """渲染深度图，返回 (H,W) float32，归一化到[0,1]。

        使用MuJoCo离屏渲染的真实深度缓冲，而非RGB亮度近似。
        """
        if self._renderer is None:
            self._renderer = mujoco.Renderer(self.model, self.image_h, self.image_w)
        self._renderer.update_scene(self.data, camera=self._cam_id)
        # MuJoCo 2.3+ 支持 render(depth=True) 获取真实深度缓冲
        try:
            depth = self._renderer.render(depth=True)
            # 深度值为距离(m)，归一化到[0,1]（近处=0，远处=1）
            znear = self.model.vis.map.znear
            zfar = self.model.vis.map.zfar
            depth_norm = np.clip((depth - znear) / (zfar - znear), 0.0, 1.0)
            return depth_norm.astype(np.float32)
        except TypeError:
            # 旧版MuJoCo不支持depth参数，回退到RGB亮度近似
            img = self._renderer.render()
            gray = 0.299*img[..., 0] + 0.587*img[..., 1] + 0.114*img[..., 2]
            return (1.0 - gray.astype(np.float32)/255.0)

    def render_rgb(self, height=480, width=640, camera=None):
        """渲染彩色画面，用于录制演示动图。

        Args:
            camera: 相机名。默认用深度图那个 front_camera；注意它在头部朝下，
                录出来是俯视躯干顶面，看演示要用 "overview"（跟随 torso）。
        """
        if self._rgb_renderer is None:
            self._rgb_renderer = mujoco.Renderer(self.model, height, width)
        cam_id = self._cam_id
        if camera is not None:
            cam_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_CAMERA, camera)
            if cam_id < 0:
                raise ValueError(f"模型里没有相机 {camera!r}")
        self._rgb_renderer.update_scene(self.data, camera=cam_id)
        return self._rgb_renderer.render()

    def close(self):
        for attr in ("_renderer", "_rgb_renderer"):
            r = getattr(self, attr, None)
            if r is not None:
                try: r.close()
                except Exception: pass
                setattr(self, attr, None)
