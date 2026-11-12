# -*- coding: utf-8 -*-
"""MuJoCo 播放器：把 ZMP 规划出的关节轨迹放到台阶场景里，量穿透与高度。

这个模块**不是**动力学平衡控制：躯干位姿与关节角都按规划直接写入（运动学驱动），
MuJoCo 只用来做几何/接触判定。这样评价的是「ZMP 规划 + IK 出来的轨迹是否可执行」，
而不是「控制器能否在扰动下站稳」——后者需要力矩级控制，不在本模块范围。
"""

import numpy as np

import humanoid_stair_zmp_preview.config as config
import humanoid_stair_zmp_preview.gait as gait


class StairSim:
    """按规划轨迹驱动平面双足上下台阶。"""

    def __init__(self, step_height=config.STEP_HEIGHT, step_depth=config.STEP_DEPTH,
                 count=config.STEP_COUNT, direction="up"):
        import mujoco
        from humanoid_stair_zmp_preview import scene

        self._mj = mujoco
        self.plan = gait.plan(step_height, step_depth, count, direction)
        self.xml = scene.build_scene_xml(step_height, step_depth, count, direction)
        self.model = mujoco.MjModel.from_xml_string(self.xml)
        self.data = mujoco.MjData(self.model)

        names = ["root_x", "root_z", "root_pitch", "hip_l", "knee_l", "ankle_l",
                 "hip_r", "knee_r", "ankle_r"]
        self._q = [int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, n))
                   for n in names]
        self._qa = [int(self.model.jnt_qposadr[j]) for j in self._q]
        self._foot_geoms = {int(mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_GEOM, g))
                            for g in ("foot_geom_l", "foot_geom_r")}
        self.n = len(self.plan["com"])
        self.dt_plan = self.plan["dt"]
        self._steps_per = max(1, int(round(self.dt_plan / config.TIMESTEP)))
        self.mujoco = mujoco

    def run(self):
        """按规划逐帧推进，返回遥测。"""
        plan = self.plan
        com = plan["com"]
        jl, jr = plan["joints"]["l"], plan["joints"]["r"]
        t_pen, z_torso, contacts = [], [], []
        d = self.data
        self.mujoco.mj_forward(self.model, d)

        for k in range(self.n):
            # 躯干 z 跟随**支撑脚**：上台阶时支撑脚是刚踏上的那只（较高），
            # 下台阶时是较低的那只。早期统一用 max()，下台阶升幅就少了半个台阶
            # （实测 -0.300 而非 -0.450）。
            # 躯干高度取**双脚较低者**（即着地那只）。若跟着前脚升，
            # 后腿会被拉长到 1.045 m（> 腿长 0.80 m），306 帧不可达，
            # 脚被 IK 限幅后甩进台阶里 —— 这就是 -220 mm 穿透的来源。
            fz = float(min(plan["feet"]["l"][k, 1], plan["feet"]["r"][k, 1]))
            zt = config.HIP_Z - config.CROUCH + fz
            d.qpos[self._qa[0]] = com[k]
            # 注意：torso body 自带 pos z = HIP_Z，qpos 是这个基座的**偏移**，
            # 写成绝对高度会让高度被算两次（脚悬空、接触数恒为 0）。
            d.qpos[self._qa[1]] = zt - config.HIP_Z
            d.qpos[self._qa[2]] = 0.0
            d.qpos[self._qa[3]] = jl["hip"][k]
            d.qpos[self._qa[4]] = jl["knee"][k]
            d.qpos[self._qa[5]] = jl["ankle"][k]
            d.qpos[self._qa[6]] = jr["hip"][k]
            d.qpos[self._qa[7]] = jr["knee"][k]
            d.qpos[self._qa[8]] = jr["ankle"][k]
            self.mujoco.mj_forward(self.model, d)

            # 足底穿透：接触距离的最小值（负值 = 穿透）
            pen = 0.0
            for i in range(d.ncon):
                c = d.contact[i]
                if int(c.geom1) in self._foot_geoms or int(c.geom2) in self._foot_geoms:
                    pen = min(pen, float(c.dist))
            t_pen.append(pen)
            z_torso.append(zt)
            contacts.append(int(d.ncon))

        return dict(t=np.arange(self.n) * self.dt_plan,
                    penetration=np.asarray(t_pen),
                    torso_z=np.asarray(z_torso),
                    ncon=np.asarray(contacts),
                    com=com, plan=plan)

    def render(self, width=640, height=360, camera="side"):
        """离屏渲染一帧。"""
        mj = self.mujoco
        if getattr(self, "_renderer", None) is None:
            self._renderer = mj.Renderer(self.model, height, width)
        cam = mj.mj_name2id(self.model, mj.mjtObj.mjOBJ_CAMERA, camera)
        self._renderer.update_scene(self.data, camera=cam if cam >= 0 else None)
        return self._renderer.render()

    def close(self):
        pass