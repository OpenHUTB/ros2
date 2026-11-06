# -*- coding: utf-8 -*-
"""MuJoCo 场景封装（需要 MuJoCo）。

职责边界：
    - 只管"加载模型 / 步进 / 读写状态 / 求交射线 / 求真实间距"；
    - 不含任何决策逻辑（决策在 decision.py，纯函数）；
    - 不含任何指标统计（统计在 metrics.py）。

关于"真值间距"：site "lidar" 位于前保险杠处（局部 x = 0.22 m，与车体半长一致），
因此"射线测得的距离"和"车头到障碍物的真实间距"在同一基准上，可以直接对比。
这一点让"感知误差"成为可评价指标，同时保证决策链路里没有用到真值。
"""

import math
from dataclasses import dataclass

import numpy as np

from . import config
from .sensors import LidarFan


class MujocoUnavailable(RuntimeError):
    """未安装 MuJoCo 时抛出，便于入口打印可操作的安装提示。"""


def import_mujoco():
    """导入 MuJoCo，失败时给出可操作的提示。"""
    try:
        import mujoco  # noqa: WPS433 (延迟导入，便于无 MuJoCo 环境下跑纯逻辑测试)
    except Exception as exc:  # pragma: no cover
        raise MujocoUnavailable(
            "无法导入 mujoco（{}: {}）。请先安装：\n"
            "  python3 -m pip install mujoco==3.12.0\n"
            "无图形界面的机器不影响仿真与测试（本模块不需要相机离屏渲染），"
            "只有 --demo / --interactive 需要 OpenGL 窗口。".format(type(exc).__name__, exc)
        ) from exc
    return mujoco


def describe_environment() -> str:
    """返回一行环境信息，供启动日志打印。"""
    try:
        mujoco = import_mujoco()
        return "MuJoCo {} / numpy {} / python {}".format(
            getattr(mujoco, "__version__", "unknown"),
            np.__version__,
            ".".join(str(v) for v in __import__("sys").version_info[:3]),
        )
    except MujocoUnavailable as exc:
        return "MuJoCo 不可用：{}".format(exc)


@dataclass
class CarState:
    """一帧车辆状态。"""

    t: float
    x: float
    y: float
    yaw: float
    v: float                     #: 沿车头方向的速度 (m/s)
    yaw_rate: float              #: 偏航角速度 (rad/s)
    wheel_left: float            #: 左轮角速度 (rad/s)
    wheel_right: float           #: 右轮角速度 (rad/s)
    z: float = 0.0
    upright: float = 1.0         #: 车体"上方向"的 z 分量，1 表示水平


class CarEnv:
    """加载 car_road_scene.xml 并推进仿真。"""

    def __init__(self, scene_path,
                 timestep=config.TIMESTEP,
                 control_hz=config.CONTROL_HZ,
                 lidar_angles_deg=config.LIDAR_ANGLES_DEG,
                 lidar_max_range=config.LIDAR_MAX_RANGE_M):
        self.mujoco = import_mujoco()
        self.scene_path = str(scene_path)
        self.timestep = float(timestep)
        self.control_hz = float(control_hz)
        self.steps_per_control = max(1, int(round(1.0 / (self.control_hz * self.timestep))))

        self.model = self.mujoco.MjModel.from_xml_path(self.scene_path)
        self.data = self.mujoco.MjData(self.model)
        self.model.opt.timestep = self.timestep

        # ---------------- 名称 -> id（名称写错时立刻报出可读错误） ----------------
        self.car_body_id = self._id(self.mujoco.mjtObj.mjOBJ_BODY, "car")
        self.obstacle_body_id = self._id(self.mujoco.mjtObj.mjOBJ_BODY, "obstacle")
        self.obstacle_geom_id = self._id(self.mujoco.mjtObj.mjOBJ_GEOM, "obstacle_geom")
        self.roadside_geom_id = self._id(self.mujoco.mjtObj.mjOBJ_GEOM, "roadside_geom")
        self.site_id = self._id(self.mujoco.mjtObj.mjOBJ_SITE, config.LIDAR_SITE_NAME)

        self.actuator_left = self._id(self.mujoco.mjtObj.mjOBJ_ACTUATOR, "left_motor")
        self.actuator_right = self._id(self.mujoco.mjtObj.mjOBJ_ACTUATOR, "right_motor")

        self.joint_left = self._id(self.mujoco.mjtObj.mjOBJ_JOINT, "left_wheel_joint")
        self.joint_right = self._id(self.mujoco.mjtObj.mjOBJ_JOINT, "right_wheel_joint")

        self.free_joint_id = self._id(self.mujoco.mjtObj.mjOBJ_JOINT, "car_root")
        self._free_qpos_adr = int(self.model.jnt_qposadr[self.free_joint_id])
        self._free_dof_adr = int(self.model.jnt_dofadr[self.free_joint_id])

        # ---------------- 传感器 ----------------
        self.lidar = LidarFan(
            self.model, self.data, self.mujoco,
            angles_deg=lidar_angles_deg,
            max_range=lidar_max_range,
        )

        # ---------------- 几何量（用于真值间距，全部从模型读取而非硬编码） ----
        self._front_offset = float(self.model.site_pos[self.site_id][0])
        self._obstacle_half = self.model.geom_size[self.obstacle_geom_id].copy()

        self.reset()

    # ------------------------------------------------------------------ 基础
    def _id(self, obj_type, name):
        obj_id = self.mujoco.mj_name2id(self.model, obj_type, name)
        if obj_id < 0:
            raise ValueError(
                "场景 '{}' 中缺少名为 '{}' 的对象；请检查 models/car_road_scene.xml".format(
                    self.scene_path, name))
        return obj_id

    def reset(self) -> None:
        """复位到 XML 定义的初始状态。"""
        self.mujoco.mj_resetData(self.model, self.data)
        self.mujoco.mj_forward(self.model, self.data)

    def set_wheel_speeds(self, left_omega: float, right_omega: float) -> None:
        """写入左右轮目标角速度（速度伺服，ctrl 为目标值）。"""
        self.data.ctrl[self.actuator_left] = float(left_omega)
        self.data.ctrl[self.actuator_right] = float(right_omega)

    def step(self, count: int = 1) -> None:
        """物理步进 count 次。"""
        for _ in range(int(count)):
            self.mujoco.mj_step(self.model, self.data)

    def advance_control_period(self) -> None:
        """推进一个控制周期（控制频率与物理步长解耦）。"""
        self.step(self.steps_per_control)

    # ------------------------------------------------------------------ 状态
    @property
    def time(self) -> float:
        return float(self.data.time)

    def scan(self) -> np.ndarray:
        """一帧射线距离数组。"""
        return self.lidar.scan()

    def sensed_distance(self):
        """本车道最近距离 (m)；无有效命中返回 None。"""
        return self.lidar.min_distance()

    def state(self) -> CarState:
        """读取车辆状态。"""
        qpos = self.data.qpos
        quat = qpos[self._free_qpos_adr + 3: self._free_qpos_adr + 7]
        rotation = self._quat_to_matrix(quat)
        yaw = math.atan2(rotation[1, 0], rotation[0, 0])

        qvel = self.data.qvel
        velocity = qvel[self._free_dof_adr: self._free_dof_adr + 3]
        heading = np.array([math.cos(yaw), math.sin(yaw), 0.0])
        forward_speed = float(np.dot(velocity, heading))

        return CarState(
            t=self.time,
            x=float(qpos[self._free_qpos_adr + 0]),
            y=float(qpos[self._free_qpos_adr + 1]),
            yaw=yaw,
            v=forward_speed,
            yaw_rate=float(qvel[self._free_dof_adr + 5]),
            wheel_left=float(qvel[self.model.jnt_dofadr[self.joint_left]]),
            wheel_right=float(qvel[self.model.jnt_dofadr[self.joint_right]]),
            z=float(qpos[self._free_qpos_adr + 2]),
            upright=float(rotation[2, 2]),
        )

    @staticmethod
    def _quat_to_matrix(quat) -> np.ndarray:
        """四元数 (w, x, y, z) -> 旋转矩阵。手写以免依赖具体 MuJoCo 版本的工具函数。"""
        w, x, y, z = (float(v) for v in quat)
        return np.array([
            [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
            [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
            [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
        ], dtype=np.float64)

    # ------------------------------------------------------------------ 真值
    def truth_clearance(self) -> float:
        """车头（site 位置）到主障碍物表面的真实间距 (m)，仅用于性能评价。

        障碍物为轴对齐长方体，因此取"点到 AABB 的距离"即可。
        """
        site = np.array(self.data.site_xpos[self.site_id], dtype=np.float64)
        center = np.array(self.data.xpos[self.obstacle_body_id], dtype=np.float64)
        half = self._obstacle_half

        delta = np.abs(site[:2] - center[:2]) - half[:2]
        outside = np.maximum(delta, 0.0)
        return float(np.hypot(outside[0], outside[1]))

    def has_obstacle_contact(self) -> bool:
        """是否与主障碍物或路侧物体发生了接触（用于碰撞判定）。"""
        for index in range(self.data.ncon):
            contact = self.data.contact[index]
            pair = (int(contact.geom1), int(contact.geom2))
            if self.obstacle_geom_id in pair or self.roadside_geom_id in pair:
                return True
        return False

    # ------------------------------------------------------------------ 视图
    def launch_viewer(self):
        """打开交互视窗（需要 OpenGL）；返回上下文管理器。

        `mujoco.viewer` 是**子模块**，必须显式导入：只写 `import mujoco` 时，
        部分版本上 `mujoco.viewer` 这个属性并不存在，会报
        "module 'mujoco' has no attribute 'viewer'"（实测 MuJoCo 3.15 就是如此）。
        """
        try:
            import mujoco.viewer   # noqa: F401  子模块需要显式导入
        except Exception as exc:   # pragma: no cover - 无图形环境时才会进入
            raise MujocoUnavailable(
                "无法加载 MuJoCo 视窗模块（{}）。\n"
                "  - 无图形界面请改用 --headless；\n"
                "  - 虚拟机里打不开窗口时先试："
                "MESA_GL_VERSION_OVERRIDE=3.3 python3 main.py --demo；\n"
                "  - 仍失败可安装 sudo apt install -y libglfw3 libgl1".format(exc))
        viewer = self.mujoco.viewer.launch_passive(self.model, self.data)
        # 车体 geoms 在 group=3，默认视窗不渲染该组；显式开启让车可见。
        try:
            viewer.opt.geomgroup[3] = 1
        except AttributeError:
            pass
        return viewer

    # ------------------------------------------------------------------ 诊断
    def set_pitch_deg(self, degrees: float) -> None:
        """把自车绕 y 轴旋转指定角度，并前向传播一次。

        仅用于测试与诊断：验证感知对车身俯仰的鲁棒性
        （观测到实车加减速时俯仰约 ±4°，曾导致射线越过障碍物顶端而丢失目标）。
        """
        half = math.radians(degrees) / 2.0
        self.data.qpos[self._free_qpos_adr + 3: self._free_qpos_adr + 7] = (
            math.cos(half), 0.0, math.sin(half), 0.0)
        self.mujoco.mj_forward(self.model, self.data)
