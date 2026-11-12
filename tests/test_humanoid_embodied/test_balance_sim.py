# -*- coding: utf-8 -*-
"""平衡控制层 + MuJoCo 冒烟测试。

* 无 MuJoCo 时只跑纯计算部分（力矩限幅、踝策略单调性等）；
* 有 MuJoCo 时额外验证：模型能编译、站立 20 秒不摔倒、行走有前进位移。

这三个物理用例是本模块的"活文档"——它们把调试期踩过的坑全部钉住：
  1. IMU site 必须写 ``zaxis``，否则 framequat 永远是恒等四元数；
  2. ctrl 对 <position> 执行器是"目标角"而不是"力矩"，填力矩会把机器人弹飞；
  3. 站立构型不能完全伸直（运动学奇异），必须屈膝；
  4. 相位推进不允许被 period 上限压回 0.77 cycle/s 的发散区。
"""

from __future__ import annotations

import math
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import _path  # noqa: E402

from humanoid_embodied_control import gait  # noqa: E402
from humanoid_embodied_control.balance import (  # noqa: E402
    BalanceController, is_fallen, tilt_angle,
)

QUAT_ID = (1.0, 0.0, 0.0, 0.0)
ZERO3 = (0.0, 0.0, 0.0)


class TestBalancePure:
    def test_balance_forces_zero_when_level(self):
        b = BalanceController()
        f = b.compute_balance(QUAT_ID, ZERO3, (0.0, 0.0, 0.95), ZERO3)
        assert len(f) == 6
        assert abs(f[3]) < 1e-9
        assert abs(f[4]) < 1e-9

    def test_balance_clips_to_limit(self):
        b = BalanceController(kp_balance=1e9, kd_balance=1e9, torque_limit=100.0)
        f = b.compute_balance((0.98, 0.2, 0.0, 0.0), (5.0, 5.0, 5.0), (0, 0, 0.95), ZERO3)
        assert all(abs(v) <= 100.0 + 1e-6 for v in f)

    def test_joint_command_is_angle_not_torque(self):
        """关键回归：下发的是角度，量级 ≤ π。"""
        b = BalanceController()
        cmd = b.joint_command([0.1] * 12)
        assert len(cmd) == 12
        assert all(abs(v) <= math.pi for v in cmd)

    def test_joint_command_rate_limits(self):
        b = BalanceController(max_joint_step=0.1)
        first = b.joint_command([0.0] * 12)
        b.joint_command([0.0] * 12)
        jump = b.joint_command([1.0] * 12)
        assert all(abs(v - first[i]) <= 0.1 + 1e-9 for i, v in enumerate(jump))

    def test_joint_command_clips_absurd_target(self):
        b = BalanceController()
        cmd = b.joint_command([300.0] * 12)
        assert all(abs(v) <= math.pi for v in cmd), "目标角未裁剪到 ±π"

    def test_ankle_correction_opposes_pitch(self):
        """踝补偿必须与俯仰误差反号（把躯干推向竖直）。

        这里只锁"反号"这个定性性质，不锁 absolute 方向：在 models/humanoid.xml 上
        开环实测 ankle=+0.20 → pitch=-9.04°，即正踝角把躯干往前推；
        但闭环下 kp_balance=3200 的根力矩环是主导项，两个符号在
        纯站立时都能稳定（实测 tilt 都 <1°）。符号的实际可判别性只在
        步态中体现，由 :class:`TestMuJoCoModel` 的行走用例把关。
        """
        b = BalanceController()
        # 后倾 20° 的四元数（w=cos10°, y=+sin10°）
        quat_back = (math.cos(math.radians(20) / 2), 0.0, math.sin(math.radians(20) / 2), 0.0)
        # 前倾 20° 的四元数（w=cos10°, y=-sin10°）
        quat_fwd = (math.cos(math.radians(20) / 2), 0.0, -math.sin(math.radians(20) / 2), 0.0)

        back_ank, _ = b.ankle_correction(quat_back, (0.0, 0.0, 0.0), kp=0.5, kd=0.0)
        fwd_ank, _ = b.ankle_correction(quat_fwd, (0.0, 0.0, 0.0), kp=0.5, kd=0.0)
        assert back_ank * fwd_ank < 0, "踝补偿没有随俯仰反向变化"
        # 后倾（pitch>0）→ 补偿为负；前倾（pitch<0）→ 补偿为正
        assert back_ank < 0 < fwd_ank

    def test_ankle_correction_limit(self):
        b = BalanceController()
        quat = (0.0, 0.0, 1.0, 0.0)     # 90°
        ankle, _ = b.ankle_correction(quat, (9.0, 9.0, 0.0), kp=99.0, kd=99.0, limit=0.2)
        assert abs(ankle) <= 0.2 + 1e-9

    def test_apply_ankle_strategy_mutates(self):
        b = BalanceController()
        pose = gait.stand_pose()
        out = b.apply_ankle_strategy(pose, QUAT_ID, ZERO3)
        assert out is pose, "应原地修改并返回同一 dict"
        assert "ankle_l_pitch" in pose

    def test_tilt_angle(self):
        assert tilt_angle(QUAT_ID) < 1e-9
        q = (math.cos(math.pi / 8), math.sin(math.pi / 8), 0.0, 0.0)
        assert abs(tilt_angle(q) - math.pi / 4) < 1e-9

    def test_is_fallen_by_height(self):
        assert not is_fallen(QUAT_ID, 0.9)
        assert is_fallen(QUAT_ID, 0.4)

    def test_is_fallen_by_tilt(self):
        q = (math.cos(math.pi / 5), math.sin(math.pi / 5), 0.0, 0.0)   # 倾斜 72°
        assert is_fallen(q, 0.9)

    def test_z_hold_force(self):
        b = BalanceController(z_hold=0.946, fz_scale=1.0)
        low = b.compute_balance(QUAT_ID, ZERO3, (0, 0, 0.5), ZERO3)
        high = b.compute_balance(QUAT_ID, ZERO3, (0, 0, 1.4), ZERO3)
        assert low[2] > 0, "躯干低于目标时应给向上的支撑"
        assert high[2] < 0, "躯干高于目标时应给向下的力"

    def test_z_hold_disabled(self):
        b = BalanceController(fz_scale=0.0)
        f = b.compute_balance(QUAT_ID, ZERO3, (0, 0, 0.5), ZERO3)
        assert f[2] == 0.0, "fz_scale=0 时不应有竖直支撑"


@pytest.mark.skipif(not _path.have_mujoco(), reason="mujoco 不可用，跳过物理用例")
class TestMuJoCoModel:
    def test_model_compiles(self):
        import mujoco
        m = mujoco.MjModel.from_xml_path(_path.model_path())
        assert m.nq == 19, "nq 应为 19（freejoint 7 + 12 个关节）"
        assert m.nu == 18, "应含 6 个平衡通道 + 12 个关节伺服"
        assert m.opt.timestep <= 0.001, "时间步必须 ≤1 ms，否则伺服会数值爆炸"

    def test_imu_has_zaxis(self):
        """关键回归：IMU site 必须有 zaxis，否则 framequat 恒等。"""
        import mujoco
        import numpy as np
        m = mujoco.MjModel.from_xml_path(_path.model_path())
        d = mujoco.MjData(m)
        d.qpos[3:7] = [math.cos(math.pi / 8), 0.0, math.sin(math.pi / 8), 0.0]
        mujoco.mj_forward(m, d)
        q = d.sensor("torso_quat").data
        assert not np.allclose(q, [1.0, 0.0, 0.0, 0.0]), \
            "framequat 输出恒等四元数：site 缺 zaxis"

    def test_scene_static(self):
        """场景物体不能计入机器人质量（曾经让机器人背 150 kg）。"""
        import mujoco
        m = mujoco.MjModel.from_xml_path(_path.model_path())
        assert float(m.body_mass.sum()) < 60.0

    def test_foot_touches_ground(self):
        import mujoco
        m = mujoco.MjModel.from_xml_path(_path.model_path())
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        g = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "foot_l_geom")
        bottom = d.geom_xpos[g][2] - m.geom_size[g][2]
        assert abs(bottom) < 0.01, "初始脚底应刚好贴地"

    def test_stand_20s_no_fall(self):
        import mujoco
        import numpy as np
        m = mujoco.MjModel.from_xml_path(_path.model_path())
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        bal = BalanceController()
        cmd = gait.to_vector(gait.stand_pose())
        dt = m.opt.timestep
        for k in range(int(20.0 / dt)):
            q = d.sensor("torso_quat").data
            w = d.sensor("torso_gyro").data
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            gyro = (float(w[0]), float(w[1]), float(w[2]))
            d.ctrl[:] = 0.0
            d.ctrl[6:18] = bal.joint_command(cmd)
            bf = bal.compute_balance(quat, gyro, tuple(float(v) for v in d.qpos[0:3]),
                                     tuple(float(v) for v in d.qvel[0:3]))
            for i in range(6):
                d.ctrl[i] = bf[i]
            mujoco.mj_step(m, d)
            assert not is_fallen(quat, float(d.qpos[2])), "t=%.1fs 站立摔倒" % d.time
        assert abs(float(d.qpos[2]) - 0.946) < 0.1, "站立终态高度异常"

    def test_walk_makes_progress(self):
        """25 s 内必须产生前进位移（本模块的核心能力验证）。"""
        import mujoco
        import numpy as np
        m = mujoco.MjModel.from_xml_path(_path.model_path())
        d = mujoco.MjData(m)
        mujoco.mj_forward(m, d)
        bal = BalanceController()
        rate = gait.phase_rate(0.2, 0.20, 1.3)
        dt = m.opt.timestep
        every = max(1, int(round(0.01 / dt)))
        phase = 0.0
        pose = gait.stand_pose()
        x0 = float(d.qpos[0])
        for k in range(int(25.0 / dt)):
            q = d.sensor("torso_quat").data
            w = d.sensor("torso_gyro").data
            quat = (float(q[0]), float(q[1]), float(q[2]), float(q[3]))
            gyro = (float(w[0]), float(w[1]), float(w[2]))
            if k % every == 0:
                phase = (phase + rate * (dt * every)) % 1.0
                pose = gait.walk_pose(phase, 0.20, 0.10)
            d.ctrl[:] = 0.0
            d.ctrl[6:18] = bal.joint_command(gait.to_vector(gait._clip_pose(pose)))
            bf = bal.compute_balance(quat, gyro, tuple(float(v) for v in d.qpos[0:3]),
                                     tuple(float(v) for v in d.qvel[0:3]))
            for i in range(6):
                d.ctrl[i] = bf[i]
            mujoco.mj_step(m, d)
            if is_fallen(quat, float(d.qpos[2])):
                break
        assert not is_fallen(quat, float(d.qpos[2])), "行走中摔倒"
        assert float(d.qpos[0]) - x0 > 1.0, "行走 25 s 前进不足 1 m"
