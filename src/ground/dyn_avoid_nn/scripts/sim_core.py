"""移动障碍物避让仿真核心。

场地 16 m × 12 m，车从左下角驶向右上角的目标点。
场内有 3 个静止障碍和 2 个沿 y 轴往复扫动的圆柱障碍。

移动障碍用运动学方式驱动：每步把滑移关节的 qpos 设成
y = A·sin(ω·t + φ)，因此它们的位置与速度都是解析已知的（供专家使用），
而神经网络只能从激光雷达序列中自己推断。
"""

import os

import numpy as np
import mujoco

# 场地/目标常量放在 consts.py（不依赖 mujoco），便于 oracle 与单元测试在
# 没有仿真器的环境下导入。
from consts import HALF_X, HALF_Y, GOAL, GOAL_R, START  # noqa: E402

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "dyn_world.xml")

N_BEAMS = 91
FOV = np.deg2rad(360.0)
ANGLE_MIN = -FOV / 2.0
ANGLE_MAX = FOV / 2.0
MAX_RANGE = 10.0

MAX_THRUST = 45.0
HULL_R = 0.45

DT = 0.05          # 一个控制步对应的仿真时间 (0.01 × 5 子步)

# 移动障碍参数：body 名、振幅、角频率、相位
MOVERS = [
    {"body": "mov0", "amp": 2.8, "omega": 0.35, "phase": 0.0},
    {"body": "mov1", "amp": 2.4, "omega": 0.45, "phase": 2.1},
]


class DynSim(object):
    def __init__(self, model_path=MODEL_PATH, seed=0):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        self.sid_lidar = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        self.sid_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_l")
        self.sid_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_r")
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")

        # 机器人关节地址必须按名字查：XML 里移动障碍的关节排在机器人之前，
        # 直接写 qpos[0..2] 会写到障碍的滑移关节上（曾经因此"车不动、障碍在动"）。
        def _adr(name):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])

        self.jx, self.jy, self.jyaw = (_adr("rx")[0], _adr("ry")[0], _adr("ryaw")[0])
        self.dx, self.dy, self.dyaw = (_adr("rx")[1], _adr("ry")[1], _adr("ryaw")[1])

        self.mover_bid = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, m["body"]) for m in MOVERS]
        self.mover_jid = [mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, m["body"] + "_y") for m in MOVERS]
        self.mover_qadr = [int(self.model.jnt_qposadr[j]) for j in self.mover_jid]

        self.beam_angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
        self.geomgroup = np.ones(6, dtype=np.uint8)
        self.geomid = np.array([-1], dtype=np.int32)

        # 静止障碍（矩形）
        self.obstacles = []
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if name is None or not name.startswith("stat"):
                continue
            p = self.model.geom_pos[gid]
            s = self.model.geom_size[gid]
            self.obstacles.append({"x": float(p[0]), "y": float(p[1]),
                                   "hx": float(s[0]), "hy": float(s[1])})
        self.reset()

    # ---------------------------------------------------------------- 状态
    def sample_start(self, rng):
        """在左侧区域随机采样一个不与静止障碍重叠的起点。"""
        for _ in range(300):
            x = float(rng.uniform(-7.2, -4.0))
            y = float(rng.uniform(-5.0, 4.0))
            ok = abs(x) < HALF_X - 0.6 and abs(y) < HALF_Y - 0.6
            for o in self.obstacles:
                if abs(x - o["x"]) < o["hx"] + 0.9 and abs(y - o["y"]) < o["hy"] + 0.9:
                    ok = False
                    break
            if ok:
                return (x, y, float(rng.uniform(-0.8, 0.8)))
        return START

    def reset(self, start=None, rng=None, jitter=True):
        if start is None:
            start = self.sample_start(rng if rng is not None else self.rng)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.jx] = start[0]
        self.data.qpos[self.jy] = start[1]
        self.data.qpos[self.jyaw] = start[2]
        # 障碍相位抖动：让每次回合的扫动节奏不同
        self.t = float((rng if rng is not None else self.rng).uniform(0.0, 8.0)) if jitter else 0.0
        self._apply_movers()
        mujoco.mj_forward(self.model, self.data)
        self.collided = False
        self.steps = 0
        return self.observation()

    def _apply_movers(self):
        for m, qadr in zip(MOVERS, self.mover_qadr):
            self.data.qpos[qadr] = m["amp"] * np.sin(m["omega"] * self.t + m["phase"])

    def pose(self):
        d = self.data
        return (float(d.qpos[self.jx]), float(d.qpos[self.jy]), float(d.qpos[self.jyaw]))

    def speed(self):
        return float(np.hypot(self.data.qvel[self.dx], self.data.qvel[self.dy]))

    def yaw_rate(self):
        return float(self.data.qvel[self.dyaw])

    def movers(self):
        """返回每个移动障碍的 (x, y, vx, vy)。"""
        out = []
        for m, bid in zip(MOVERS, self.mover_bid):
            p = self.data.xpos[bid]
            v = m["amp"] * m["omega"] * np.cos(m["omega"] * self.t + m["phase"])
            out.append((float(p[0]), float(p[1]), 0.0, float(v)))
        return out

    def mover_radius(self):
        return 0.55

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

    def observation(self):
        return {"scan": self.lidar(), "pose": self.pose(), "speed": self.speed(),
                "movers": self.movers(), "t": self.t}

    # ---------------------------------------------------------------- 动力学
    def step(self, tl, tr, n_sub=5):
        tl = float(np.clip(tl, -MAX_THRUST, MAX_THRUST))
        tr = float(np.clip(tr, -MAX_THRUST, MAX_THRUST))
        for _ in range(n_sub):
            yaw = float(self.data.qpos[self.jyaw])
            fx = np.array([np.cos(yaw), np.sin(yaw), 0.0])
            pl = self.data.site_xpos[self.sid_l].copy()
            pr = self.data.site_xpos[self.sid_r].copy()
            mujoco.mj_applyFT(self.model, self.data, fx * tl, np.zeros(3), pl, self.bid, self.data.qfrc_applied)
            mujoco.mj_applyFT(self.model, self.data, fx * tr, np.zeros(3), pr, self.bid, self.data.qfrc_applied)
            self.t += 0.01
            self._apply_movers()
            mujoco.mj_step(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0
        self.steps += 1
        self._check_collision()
        return self.observation()

    def _check_collision(self):
        x, y, _ = self.pose()
        if abs(x) > HALF_X - 0.35 or abs(y) > HALF_Y - 0.35:
            self.collided = True
            return
        for o in self.obstacles:
            dx = max(abs(x - o["x"]) - o["hx"], 0.0)
            dy = max(abs(y - o["y"]) - o["hy"], 0.0)
            if np.hypot(dx, dy) < HULL_R:
                self.collided = True
                return
        for mx, my, _, _ in self.movers():
            if np.hypot(mx - x, my - y) < HULL_R + self.mover_radius():
                self.collided = True
                return

    # ---------------------------------------------------------------- 任务
    def goal_bearing(self):
        x, y, yaw = self.pose()
        return float(np.arctan2(GOAL[1] - y, GOAL[0] - x) - yaw)

    def goal_distance(self):
        x, y, _ = self.pose()
        return float(np.hypot(GOAL[0] - x, GOAL[1] - y))

    def reached(self):
        return self.goal_distance() < GOAL_R


def thrust_from_twist(v_cmd, w_cmd):
    c = 16.0
    k_w = 26.0
    base = 0.5 * c * v_cmd
    diff = 0.5 * w_cmd * k_w
    return base + diff, base - diff
