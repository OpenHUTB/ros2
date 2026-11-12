"""狭窄通道通行仿真核心。

场地 14 m × 12 m，中间一道隔墙把场地分成两半，墙上有三个开口：
0.7 m（车宽 0.6 m，实际过不去）、1.4 m、2.4 m。
小车要从左侧出发，自己选一个能通过的开口穿到右侧的目标点。
"""

import os

import numpy as np
import mujoco

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "gap_world.xml")

HALF_X = 7.0
HALF_Y = 6.0

N_BEAMS = 91
FOV = np.deg2rad(270.0)
ANGLE_MIN = -FOV / 2.0
ANGLE_MAX = FOV / 2.0
MAX_RANGE = 8.0

MAX_THRUST = 45.0
HULL_R = 0.42
ROBOT_HALF_W = 0.30
GOAL_R = 0.7
GOAL_X = 6.0
START_X = -6.0

# 由 XML 推出的开口（地面真值，供专家使用）
GAPS = [
    {"lo": -3.55, "hi": -2.85},
    {"lo": -0.70, "hi": 0.70},
    {"lo": 2.00, "hi": 4.40},
]
MIN_PASSABLE = 0.95        # 开口宽度小于此值视为不可通行（车宽 0.6 + 余量）


class GapSim(object):
    def __init__(self, model_path=MODEL_PATH, seed=0):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        def _adr(name):
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            return int(self.model.jnt_qposadr[jid]), int(self.model.jnt_dofadr[jid])

        self.jx, self.jy, self.jyaw = (_adr("rx")[0], _adr("ry")[0], _adr("ryaw")[0])
        self.dx, self.dy, self.dyaw = (_adr("rx")[1], _adr("ry")[1], _adr("ryaw")[1])

        self.sid_lidar = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        self.sid_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_l")
        self.sid_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_r")
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")

        self.beam_angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
        self.geomgroup = np.ones(6, dtype=np.uint8)
        self.geomid = np.array([-1], dtype=np.int32)

        # 墙段（矩形，供碰撞判定）
        self.walls = []
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if name is None or not name.startswith("seg"):
                continue
            p = self.model.geom_pos[gid]
            s = self.model.geom_size[gid]
            self.walls.append({"x": float(p[0]), "y": float(p[1]),
                               "hx": float(s[0]), "hy": float(s[1])})

        # 由墙段之间的空隙推导开口，避免几何真值与模型文件两处维护
        self.gaps = []
        segs = sorted(self.walls, key=lambda w: w["y"])
        for a, b in zip(segs, segs[1:]):
            lo = a["y"] + a["hy"]
            hi = b["y"] - b["hy"]
            if hi - lo > 0.05:
                self.gaps.append({"lo": float(lo), "hi": float(hi)})
        self.reset()

    # ---------------------------------------------------------------- 任务
    def sample_task(self, rng):
        y0 = float(rng.uniform(-4.5, 4.5))
        y1 = float(rng.uniform(-4.5, 4.5))
        return (START_X, y0), (GOAL_X, y1)

    def reset(self, start=None, goal=None, rng=None, jitter=True):
        r = rng if rng is not None else self.rng
        s, g = self.sample_task(r) if start is None else (start, goal)
        self.start = s
        self.goal = g
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.jx] = s[0]
        self.data.qpos[self.jy] = s[1]
        self.data.qpos[self.jyaw] = float(r.uniform(-0.5, 0.5)) if jitter else 0.0
        mujoco.mj_forward(self.model, self.data)
        self.collided = False
        self.steps = 0
        return self.observation()

    def pose(self):
        d = self.data
        return (float(d.qpos[self.jx]), float(d.qpos[self.jy]), float(d.qpos[self.jyaw]))

    def speed(self):
        return float(np.hypot(self.data.qvel[self.dx], self.data.qvel[self.dy]))

    def yaw_rate(self):
        return float(self.data.qvel[self.dyaw])

    def goal_distance(self):
        x, y, _ = self.pose()
        return float(np.hypot(self.goal[0] - x, self.goal[1] - y))

    def goal_bearing(self):
        x, y, yaw = self.pose()
        return float(np.arctan2(self.goal[1] - y, self.goal[0] - x) - yaw)

    def reached(self):
        return self.goal_distance() < GOAL_R

    def crossed(self):
        x, _, _ = self.pose()
        return x > 0.35

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
                "goal": self.goal}

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
        for w in self.walls:
            dx = max(abs(x - w["x"]) - w["hx"], 0.0)
            dy = max(abs(y - w["y"]) - w["hy"], 0.0)
            if np.hypot(dx, dy) < HULL_R:
                self.collided = True
                return


def thrust_from_twist(v_cmd, w_cmd):
    c = 16.0
    k_w = 26.0
    base = 0.5 * c * v_cmd
    diff = 0.5 * w_cmd * k_w
    return base + diff, base - diff
