"""区域覆盖清扫仿真核心。

场地 24 m × 16 m，内部有若干矩形家具障碍；差速清扫车用平面三自由度模型，
车头brush 覆盖半径内的栅格被标记为已清扫。

对外提供：
    CoverSim.reset()      回到起点、清空覆盖计数
    CoverSim.lidar()      61 束二维激光雷达
    CoverSim.step(tl,tr)  施加双轮推力推进一步并更新覆盖
    CoverSim.coverage()   当前覆盖率
"""

import os

import numpy as np
import mujoco

MODEL_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "models", "cover_world.xml")

# 场地
# HALF_X / HALF_Y 是墙体的**中心线**位置（对应 cover_world.xml 里 pos=±6 / ±5 的盒子），
# 墙体半厚 WALL_T=0.3，所以真实内壁在 ±(6-0.3)=5.7 / ±(5-0.3)=4.7。
# 所有"到墙距离"的量必须用 INNER_*，否则会恒定多算 0.30 m。
HALF_X = 6.0
HALF_Y = 5.0
WALL_T = 0.3            # 墙体半厚（与 cover_world.xml 的 size 一致）
INNER_X = HALF_X - WALL_T
INNER_Y = HALF_Y - WALL_T
RES = 0.5
NX = int(2 * HALF_X / RES)
NY = int(2 * HALF_Y / RES)

# 激光雷达
N_BEAMS = 61
FOV = np.deg2rad(270.0)
ANGLE_MIN = -FOV / 2.0
ANGLE_MAX = FOV / 2.0
MAX_RANGE = 7.0

# 车辆
MAX_THRUST = 40.0
BRUSH_R = 0.75          # 清扫半径
HULL_R = 0.45           # 碰撞半径

START = (-4.6, 3.6, 0.7)


class CoverSim(object):
    def __init__(self, model_path=MODEL_PATH, seed=0):
        self.model = mujoco.MjModel.from_xml_path(model_path)
        self.data = mujoco.MjData(self.model)
        self.rng = np.random.default_rng(seed)

        self.sid_lidar = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "lidar")
        self.sid_l = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_l")
        self.sid_r = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "thr_r")
        self.sid_brush = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, "brush_center")
        self.bid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, "robot")

        self.beam_angles = np.linspace(ANGLE_MIN, ANGLE_MAX, N_BEAMS)
        self.geomgroup = np.ones(6, dtype=np.uint8)
        self.geomid = np.array([-1], dtype=np.int32)

        # 障碍物（矩形）
        self.obstacles = []
        for gid in range(self.model.ngeom):
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, gid)
            if name is None or not name.startswith("furn"):
                continue
            p = self.model.geom_pos[gid]
            s = self.model.geom_size[gid]
            self.obstacles.append({"x": float(p[0]), "y": float(p[1]),
                                   "hx": float(s[0]), "hy": float(s[1])})

        # 自由栅格与覆盖统计
        self.free_mask = self._build_free_mask()
        # 沿墙清扫的目标区域：距墙 1.0 m 以内的自由栅格
        self.wall_mask = self._build_wall_mask()
        self.visited = np.zeros_like(self.free_mask, dtype=bool)
        self.reset()

    def _build_wall_mask(self):
        mask = np.zeros_like(self.free_mask, dtype=bool)
        for i in range(NY):
            for j in range(NX):
                if not self.free_mask[i, j]:
                    continue
                p = self.cell_center(i, j)
                d_wall = min(INNER_X - abs(p[0]), INNER_Y - abs(p[1]))
                if d_wall <= 1.0:
                    mask[i, j] = True
        return mask

    def wall_coverage(self):
        total = int(self.wall_mask.sum())
        if total == 0:
            return 0.0
        return float((self.visited & self.wall_mask).sum()) / total

    def wall_error(self):
        """当前车体到最近墙面的距离，用于评估沿墙精度。"""
        x, y, _ = self.pose()
        return min(INNER_X - abs(x), INNER_Y - abs(y))

    # ---------------------------------------------------------------- 栅格
    def _build_free_mask(self):
        mask = np.ones((NY, NX), dtype=bool)
        for i in range(NY):
            for j in range(NX):
                p = self.cell_center(i, j)
                if abs(p[0]) > HALF_X - 0.35 or abs(p[1]) > HALF_Y - 0.35:
                    mask[i, j] = False
                    continue
                for o in self.obstacles:
                    if abs(p[0] - o["x"]) < o["hx"] + 0.45 and abs(p[1] - o["y"]) < o["hy"] + 0.45:
                        mask[i, j] = False
                        break
        return mask

    def cell_center(self, i, j):
        return np.array([-HALF_X + (j + 0.5) * RES, -HALF_Y + (i + 0.5) * RES])

    def world_to_cell(self, p):
        j = int((p[0] + HALF_X) / RES)
        i = int((p[1] + HALF_Y) / RES)
        return int(np.clip(i, 0, NY - 1)), int(np.clip(j, 0, NX - 1))

    # ---------------------------------------------------------------- 状态
    def reset(self, start=START):
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[0] = start[0]
        self.data.qpos[1] = start[1]
        self.data.qpos[2] = start[2]
        mujoco.mj_forward(self.model, self.data)
        self.visited[:] = False
        self.collided = False
        self.steps = 0
        self._mark_coverage()
        return self.observation()

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

    def local_coverage(self, size=21):
        """车体系 size×size 的已清扫栅格（1=已清扫，0=未清扫或不可达）。"""
        x, y, yaw = self.pose()
        half = size // 2
        offs = (np.arange(size) - half) * RES
        dx, dy = np.meshgrid(offs, offs)
        c, s = np.cos(yaw), np.sin(yaw)
        wx = x + c * dx - s * dy
        wy = y + s * dx + c * dy
        j = ((wx + HALF_X) / RES).astype(int)
        i = ((wy + HALF_Y) / RES).astype(int)
        ok = (i >= 0) & (i < NY) & (j >= 0) & (j < NX)
        out = np.zeros((size, size), dtype=np.float32)
        out[ok] = self.visited[i[ok], j[ok]].astype(np.float32)
        return out

    def observation(self):
        return {"scan": self.lidar(),
                "cov": self.local_coverage(),
                "pose": self.pose(),
                "speed": self.speed()}

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
        self._check_collision()
        self._mark_coverage()
        return self.observation()

    def _mark_coverage(self):
        p = self.data.site_xpos[self.sid_brush]
        x, y = float(p[0]), float(p[1])
        r = int(np.ceil(BRUSH_R / RES))
        ci, cj = self.world_to_cell((x, y))
        for di in range(-r, r + 1):
            for dj in range(-r, r + 1):
                i, j = ci + di, cj + dj
                if 0 <= i < NY and 0 <= j < NX:
                    c = self.cell_center(i, j)
                    if np.hypot(c[0] - x, c[1] - y) <= BRUSH_R:
                        self.visited[i, j] = True

    def _check_collision(self):
        x, y, _ = self.pose()
        if abs(x) > INNER_X or abs(y) > INNER_Y:
            self.collided = True
            return
        for o in self.obstacles:
            dx = max(abs(x - o["x"]) - o["hx"], 0.0)
            dy = max(abs(y - o["y"]) - o["hy"], 0.0)
            if np.hypot(dx, dy) < HULL_R:
                self.collided = True
                return

    # ---------------------------------------------------------------- 指标
    def coverage(self):
        total = int(self.free_mask.sum())
        if total == 0:
            return 0.0
        return float((self.visited & self.free_mask).sum()) / total

    def over_clean(self):
        """重复覆盖率：重复清扫的栅格数 / 已清扫栅格数。"""
        covered = int((self.visited & self.free_mask).sum())
        if covered == 0:
            return 0.0
        return 1.0 - float(covered) / max(int(self.visited.sum()), 1)


def thrust_from_twist(v_cmd, w_cmd):
    c = 16.0
    k_w = 26.0
    base = 0.5 * c * v_cmd
    diff = 0.5 * w_cmd * k_w
    return base + diff, base - diff
