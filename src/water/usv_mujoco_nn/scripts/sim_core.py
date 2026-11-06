"""MuJoCo 水面无人艇（USV）仿真核心。

对外提供：
    USVSim.reset()      重置到随机的起点/终点
    USVSim.lidar()      返回 91 束二维激光雷达测距（米）
    USVSim.step(u)      施加双推进器推力并推进一步
    USVSim.pose()       返回 (x, y, yaw)

坐标系：世界系 x 向右、y 向上、yaw 逆时针为正（右手系）。
推进器布局：尾部左右各一台，沿船体 x 轴正方向出力，差速产生艏摇力矩。
"""

import os

import numpy as np
import mujoco

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "usv_world.xml")

# 激光雷达参数
N_BEAMS = 91                     # 射线数
FOV = np.deg2rad(270.0)          # 视场角 270 度
ANGLE_MIN = -FOV / 2.0
ANGLE_MAX = FOV / 2.0
MAX_RANGE = 12.0                 # 最大量程 (m)

# 推进器
MAX_THRUST = 45.0                # 单桨最大推力 (N)
HULL_HALF_LEN = 0.75             # 用于碰撞判定的船体半长
HULL_HALF_WID = 0.35

ARENA_HALF = 25.0


class USVSim(object):
    def __init__(self, model_path=MODEL_PATH, seed=0):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        self.sid_lidar = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        self.sid_thr_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_l")
        self.sid_thr_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_r")
        self.bid_usv = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "usv")
        if self.bid_usv < 0:
            raise RuntimeError("模型中找不到名为 usv 的 body")

        # 从模型里读出障碍物几何，供建图、可视化和真值碰撞判定使用
        self.obstacles = []
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if name is None or not name.startswith("obs"):
                continue
            gtype = self.model.geom_type[gid]
            pos = self.model.geom_pos[gid].copy()
            size = self.model.geom_size[gid].copy()
            if gtype == mujoco.mjtGeom.mjGEOM_CYLINDER:
                self.obstacles.append({"type": "circle", "x": float(pos[0]),
                                       "y": float(pos[1]), "r": float(size[0])})
            elif gtype == mujoco.mjtGeom.mjGEOM_BOX:
                self.obstacles.append({"type": "box", "x": float(pos[0]), "y": float(pos[1]),
                                       "hx": float(size[0]), "hy": float(size[1])})

        self.beam_angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
        self.geomgroup = np.ones(6, dtype=np.uint8)
        self.geomid = np.array([-1], dtype=np.int32)
        self.reset()

    # ------------------------------------------------------------------ 基础状态
    def reset(self, start=None, goal=None, randomize=True):
        mujoco.mj_resetData(self.model, self.data)

        if start is None or goal is None:
            if randomize:
                start, goal = self._sample_pair()
            else:
                start, goal = (-18.0, -18.0, 0.0), (18.0, 18.0)

        self.data.qpos[0] = start[0]
        self.data.qpos[1] = start[1]
        self.data.qpos[2] = start[2]
        self.goal = np.array(goal[:2], dtype=np.float64)
        self.start = np.array(start, dtype=np.float64)
        mujoco.mj_forward(self.model, self.data)

        self.collided = False
        self.steps = 0
        return self.observation()

    def pose(self):
        return (float(self.data.qpos[0]), float(self.data.qpos[1]), float(self.data.qpos[2]))

    def _sample_pair(self):
        p_start = self._free_point()
        for _ in range(200):
            p_goal = self._free_point()
            if np.linalg.norm(p_goal - p_start) > 18.0:
                th = np.arctan2(p_goal[1] - p_start[1], p_goal[0] - p_start[0])
                return (p_start[0], p_start[1], th), p_goal
        return (-18.0, -18.0, 0.0), np.array([18.0, 18.0])

    def _free_point(self):
        for _ in range(500):
            p = self.rng.uniform(-21.0, 21.0, size=2)
            if not self._point_in_obstacle(p, margin=2.5):
                return p
        return np.array([0.0, 0.0])

    def _point_in_obstacle(self, p, margin=0.0):
        for o in self.obstacles:
            if o["type"] == "circle":
                if np.hypot(p[0] - o["x"], p[1] - o["y"]) < o["r"] + margin:
                    return True
            else:
                if (abs(p[0] - o["x"]) < o["hx"] + margin) and (abs(p[1] - o["y"]) < o["hy"] + margin):
                    return True
        return False

    # ------------------------------------------------------------------ 传感器
    def lidar(self):
        """按当前位姿做射线投射，返回 (N_BEAMS,) 的测距数组。"""
        x, y, yaw = self.pose()
        pnt = self.data.site_xpos[self.sid_lidar].copy()
        ranges = np.empty(N_BEAMS, dtype=np.float64)
        for i, a in enumerate(self.beam_angles):
            th = yaw + a
            vec = np.array([np.cos(th), np.sin(th), 0.0])
            dist = mujoco.mj_ray(self.model, self.data, pnt, vec,
                                 self.geomgroup, 1, self.bid_usv, self.geomid)
            if dist is None or dist < 0.0 or not np.isfinite(dist):
                dist = MAX_RANGE
            ranges[i] = min(float(dist), MAX_RANGE)
        return ranges

    def goal_in_body(self):
        x, y, yaw = self.pose()
        d = self.goal - np.array([x, y])
        c, s = np.cos(-yaw), np.sin(-yaw)
        return np.array([c * d[0] - s * d[1], s * d[0] + c * d[1]]), float(np.linalg.norm(d))

    def observation(self):
        ranges = self.lidar()
        gb, gd = self.goal_in_body()
        return {"scan": ranges, "goal_body": gb, "goal_dist": gd}

    # ------------------------------------------------------------------ 动力学
    def step(self, thrust_l, thrust_r, n_sub=5):
        """施加左右推进器推力，推进 n_sub 个小步（每步 0.01 s）。"""
        tl = float(np.clip(thrust_l, 0.0, MAX_THRUST))
        tr = float(np.clip(thrust_r, 0.0, MAX_THRUST))

        for _ in range(n_sub):
            x, y, yaw = self.pose()
            fx = np.array([np.cos(yaw), np.sin(yaw), 0.0])
            pl = self.data.site_xpos[self.sid_thr_l].copy()
            pr = self.data.site_xpos[self.sid_thr_r].copy()
            mujoco.mj_applyFT(self.model, self.data, fx * tl, np.zeros(3), pl,
                              self.bid_usv, self.data.qfrc_applied)
            mujoco.mj_applyFT(self.model, self.data, fx * tr, np.zeros(3), pr,
                              self.bid_usv, self.data.qfrc_applied)
            mujoco.mj_step(self.model, self.data)
            self.data.qfrc_applied[:] = 0.0

        self.steps += 1
        self._check_collision()
        return self.observation()

    def _check_collision(self):
        x, y, _ = self.pose()
        if abs(x) > ARENA_HALF - 1.0 or abs(y) > ARENA_HALF - 1.0:
            self.collided = True
            return
        for o in self.obstacles:
            if o["type"] == "circle":
                if np.hypot(x - o["x"], y - o["y"]) < o["r"] + HULL_HALF_LEN:
                    self.collided = True
                    return
            else:
                dx = max(abs(x - o["x"]) - o["hx"], 0.0)
                dy = max(abs(y - o["y"]) - o["hy"], 0.0)
                if np.hypot(dx, dy) < HULL_HALF_LEN:
                    self.collided = True
                    return


def thrust_from_twist(v_cmd, w_cmd):
    """把期望线速度/角速度映射为左右推力（差速分配）。"""
    k_v = 18.0     # N per (m/s)
    k_w = 7.0      # N per (rad/s)
    base = v_cmd * k_v + 3.0     # 保底推力，克服阻力
    diff = w_cmd * k_w
    return base + diff, base - diff
