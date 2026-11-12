"""无人艇自主靠泊仿真核心。

泊位位于 x≈17.5 处，开口朝 -x，宽 4 m，后端有靠岸墙。
任务：从远处驶近、对准、贴边停稳。
"""

import os

import numpy as np
import mujoco

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "usv_dock.xml")

N_BEAMS = 91
FOV = np.deg2rad(270.0)
ANGLE_MIN = -FOV / 2.0
ANGLE_MAX = FOV / 2.0
MAX_RANGE = 12.0

MAX_THRUST = 45.0

BERTH_X = 17.5          # 泊位中心 x
BERTH_HALF_W = 3.45      # 半宽
DOCK_END_X = 18.6       # 靠岸墙内侧

# 成功判据
TOL_Y = 0.45            # 横向偏差
TOL_YAW = 0.15          # 艏向偏差 (rad)
TOL_V = 0.20            # 速度


class DockSim(object):
    def __init__(self, model_path=MODEL_PATH, seed=0):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        self.sid_lidar = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        self.sid_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_l")
        self.sid_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_r")
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "usv")

        self.beam_angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
        self.geomgroup = np.ones(6, dtype=np.uint8)
        self.geomid = np.array([-1], dtype=np.int32)

        self.start = np.array([0.0, 0.0, 0.0])
        self.reset()

    # ---------------------------------------------------------------- 状态
    def reset(self, start=None, seed=None):
        mujoco.mj_resetData(self.model, self.data)
        if start is None:
            start = self.sample_start(seed)
        self.data.qpos[0] = start[0]
        self.data.qpos[1] = start[1]
        self.data.qpos[2] = start[2]
        mujoco.mj_forward(self.model, self.data)
        self.start = np.array(start, dtype=np.float64)
        self.collided = False
        self.steps = 0
        return self.observation()

    def sample_start(self, seed=None):
        rng = self.rng if seed is None else np.random.default_rng(seed)
        x = rng.uniform(-8.0, -2.0)
        y = rng.choice([-1.0, 1.0]) * rng.uniform(1.0, 2.5)
        yaw = rng.uniform(-0.5, 0.5)
        return (float(x), float(y), float(yaw))

    def pose(self):
        d = self.data
        return (float(d.qpos[0]), float(d.qpos[1]), float(d.qpos[2]))

    def speed(self):
        return float(np.hypot(self.data.qvel[0], self.data.qvel[1]))

    def yaw_rate(self):
        return float(self.data.qvel[2])

    # ---------------------------------------------------------------- 传感器
    def lidar(self):
        x, y, yaw = self.pose()
        p = self.data.site_xpos[self.sid_lidar].copy()
        out = np.empty(N_BEAMS)
        for i, a in enumerate(self.beam_angles):
            th = yaw + a
            vec = np.array([np.cos(th), np.sin(th), 0.0])
            dist = mujoco.mj_ray(self.model, self.data, p, vec,
                                 self.geomgroup, 1, self.bid, self.geomid)
            if dist is None or dist < 0.0 or not np.isfinite(dist):
                dist = MAX_RANGE
            out[i] = min(float(dist), MAX_RANGE)
        return out

    def berth_error(self):
        """相对泊位的偏差：横向 y、艏向 yaw、到泊位中心距离。"""
        x, y, yaw = self.pose()
        return np.array([y - 0.0, yaw, BERTH_X - x])

    def observation(self):
        e = self.berth_error()
        return {"scan": self.lidar(), "berth": e, "speed": self.speed()}

    # ---------------------------------------------------------------- 动力学
    def step(self, tl, tr, n_sub=5):
        tl = float(np.clip(tl, -MAX_THRUST, MAX_THRUST))
        tr = float(np.clip(tr, -MAX_THRUST, MAX_THRUST))
        for _ in range(n_sub):
            yaw = float(self.data.qpos[2])
            fx = np.array([np.cos(yaw), np.sin(yaw), 0.0])
            pl = self.data.site_xpos[self.sid_l].copy()
            pr = self.data.site_xpos[self.sid_r].copy()
            mujoco.mj_applyFT(self.model, self.data, fx * tl, np.zeros(3), pl, self.bid, self.data.qfrc_applied)
            mujoco.mj_applyFT(self.model, self.data, fx * tr, np.zeros(3), pr, self.bid, self.data.qfrc_applied)
            mujoco.mj_step(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
        self.steps += 1
        self._check()
        return self.observation()

    def _check(self):
        x, y, yaw = self.pose()
        # 撞码头臂或靠岸墙
        if x > 15.5 and abs(y) > BERTH_HALF_W + 0.25:
            self.collided = True
        if x > DOCK_END_X - 0.70:
            self.collided = True
        if abs(x) > 24 or abs(y) > 24:
            self.collided = True

    def docked(self):
        x, y, yaw = self.pose()
        return (15.0 <= x <= DOCK_END_X - 0.6 and abs(y) <= TOL_Y
                and abs(yaw) <= TOL_YAW and self.speed() <= TOL_V)


def thrust_from_twist(v_cmd, w_cmd):
    # 稳态满足 F = c*v（每轴阻尼 c = 20 N·s/m），故每侧推力约 10*v；
    # 角速度所需的推力差由 k_w 给出（与阻尼匹配后时间常数约 0.4 s）。
    c = 20.0
    k_w = 30.0
    base = 0.5 * c * v_cmd
    diff = 0.5 * w_cmd * k_w
    return base + diff, base - diff
