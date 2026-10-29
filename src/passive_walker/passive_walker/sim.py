"""轮辐式被动行走器（rimless wheel）的 MuJoCo 仿真核心。

物理要点：
- 模型没有任何执行器（``model.nu == 0``），全程不写 ``data.ctrl``；
- 用 ``slide_x + slide_z + hinge_y`` 三个关节把刚体限制在矢状面内：
  允许轮心随辐条翻转自然升降，同时避免 3D 侧翻；
- 只给一个很小的初始前向速度（相当于“轻推一下”），之后完全靠
  重力沿 5° 斜坡驱动、靠落地碰撞耗能，形成稳定的被动滚动极限环；
- 每转过 2π/N 弧度（一根辐条）记作“一步”。

本文件不依赖 ROS，可以直接 ``python -m passive_walker.sim`` 跑通。
"""

import math
import os

XML_NAME = "passive_walker.xml"
N_SPOKE = 12                     # XML 里的辐条数，仅作兜底默认值
RADIUS = 0.45                    # 辐条长度（轮心到端部小球中心）
Q_X, Q_Z, Q_ANGLE = 0, 1, 2      # qpos 顺序：slide_x, slide_z, roll


def find_asset(name=XML_NAME):
    """按顺序查找模型文件：包内 assets → 源码布局 ../assets → ROS 安装目录 → 当前目录。"""
    here = os.path.dirname(os.path.abspath(__file__))
    candidates = [
        os.path.join(here, "assets", name),
        os.path.normpath(os.path.join(here, "..", "assets", name)),
        os.path.join(os.getcwd(), "assets", name),
    ]
    try:                                            # ROS 2 安装后模型在 share/ 下
        from ament_index_python.packages import get_package_share_directory
        candidates.insert(0, os.path.join(
            get_package_share_directory("passive_walker"), "assets", name))
    except Exception:
        pass
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError("找不到模型文件 %s，已尝试：%s" % (name, candidates))


class PassiveWalker:
    """无动力的平面轮辐式被动行走器。"""

    def __init__(self, xml_path=None, push_speed=0.3):
        import mujoco

        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_path(xml_path or find_asset())
        self.data = mujoco.MjData(self.model)
        self.dt = float(self.model.opt.timestep)
        self.push_speed = float(push_speed)
        self._hub_z0 = float(self.model.body_pos[1][2])   # 轮心初始世界高度
        self.n_spoke = self._count_tips()
        self.step_angle = 2.0 * math.pi / self.n_spoke
        self.reset()

    # ---------- 生命周期 ----------
    def reset(self, push_speed=None):
        d = self.data
        self._mujoco.mj_resetData(self.model, d)
        speed = self.push_speed if push_speed is None else float(push_speed)
        d.qvel[Q_X] = speed                       # 前向线速度
        d.qvel[Q_ANGLE] = speed / self._roll_radius()   # 匹配的滚动角速度
        self._mujoco.mj_forward(self.model, d)
        self._x0 = float(d.qpos[Q_X])
        self._a0 = float(d.qpos[Q_ANGLE])
        h = self._hub_clearance()
        self._hmin = self._hmax = h
        self._switches = 0
        self._i = 0
        self._prev_contacts = self._contact_key()

    # ---------- 模型查询 ----------
    def _slope_normal(self):
        """坡面法向（xz 平面内的单位向量），用于把轮心高度折算成“距坡面距离”。"""
        for i in range(self.model.ngeom):
            if self.model.geom_type[i] == self._mujoco.mjtGeom.mjGEOM_PLANE:
                m = self.data.geom_xmat[i]
                return float(m[2]), float(m[8])        # 局部 z 轴在世界中的 xz 分量
        return 0.0, 1.0

    def _hub_clearance(self):
        """轮心到坡面的垂直距离（沿法向），比世界 z 坐标更直观。"""
        nx, nz = self._slope_normal()
        return nx * float(self.data.qpos[Q_X]) + nz * float(self.data.qpos[Q_Z] + self._hub_z0)

    def _tip_geoms(self):
        """端部小球：半径 < 0.04 的 SPHERE geom（轮毂半径 0.05 不会误判）。"""
        mj = self._mujoco
        return [i for i in range(self.model.ngeom)
                if self.model.geom_type[i] == mj.mjtGeom.mjGEOM_SPHERE
                and float(self.model.geom_size[i][0]) < 0.04]

    def _count_tips(self):
        tips = len(self._tip_geoms())
        return tips if tips > 0 else N_SPOKE

    def _roll_radius(self):
        """滚动半径 = 辐条长度 + 端部小球半径。"""
        tips = self._tip_geoms()
        return RADIUS + (float(self.model.geom_size[tips[0]][0]) if tips else 0.03)

    def _contact_key(self):
        """当前接触对集合，用于统计“触地/离地”切换次数。"""
        d = self.data
        return tuple(sorted((min(int(c.geom1), int(c.geom2)),
                             max(int(c.geom1), int(c.geom2)))
                            for c in d.contact[:int(d.ncon)]))

    # ---------- 推进 ----------
    def step(self, n=1):
        for _ in range(int(n)):
            self._mujoco.mj_step(self.model, self.data)
            h = self._hub_clearance()
            self._hmin = min(self._hmin, h)
            self._hmax = max(self._hmax, h)
            # 接触状态每 20 个物理步（10 ms）采样一次，避免数值抖动被重复计数
            self._i += 1
            if self._i % 20 == 0:
                key = self._contact_key()
                if key != self._prev_contacts:
                    self._switches += 1
                    self._prev_contacts = key

    def run(self, seconds):
        self.step(int(round(seconds / self.dt)))

    # ---------- 状态 ----------
    @property
    def x(self):
        return float(self.data.qpos[Q_X])

    @property
    def angle(self):
        return float(self.data.qpos[Q_ANGLE])

    @property
    def distance(self):
        return self.x - self._x0

    @property
    def steps(self):
        """按滚动角度计步（12 辐条时每 30° 一步）。"""
        return int(abs(self.angle - self._a0) / self.step_angle)

    @property
    def speed(self):
        return float(self.data.qvel[Q_X])

    @property
    def hub_height(self):
        """轮心到坡面的垂直距离（沿坡面法向）。"""
        return self._hub_clearance()

    @property
    def ncon(self):
        return int(self.data.ncon)

    def summary(self):
        return {
            "sim_time": float(self.data.time),
            "distance_m": self.distance,
            "steps": self.steps,
            "speed_mps": self.speed,
            "hub_height_m": self.hub_height,
            "hub_height_min_m": self._hmin,
            "hub_height_max_m": self._hmax,
            "contact_switches": self._switches,
            "ncon": self.ncon,
            "n_spoke": self.n_spoke,
            "n_actuator": int(self.model.nu),
        }


def run_headless(seconds=20.0, push_speed=0.3, xml_path=None):
    sim = PassiveWalker(xml_path=xml_path, push_speed=push_speed)
    sim.run(seconds)
    return sim.summary()


def format_summary(info):
    return ("无执行器 nu=%d（0 表示完全被动）| 仿真 %.1f s | 前进 %.2f m | 步数 %d | "
            "稳态速度 %.2f m/s | 轮心距坡面 %.3f m（%.3f~%.3f）"
            % (info["n_actuator"], info["sim_time"], info["distance_m"], info["steps"],
               info["speed_mps"], info["hub_height_m"], info["hub_height_min_m"],
               info["hub_height_max_m"]))


if __name__ == "__main__":
    print(format_summary(run_headless()))
