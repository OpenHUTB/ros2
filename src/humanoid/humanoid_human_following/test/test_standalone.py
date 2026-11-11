"""humanoid_human_following 自动化测试（standalone，无需 ROS）。"""

import os
import sys
import unittest

# 保证可导入包源码（未安装时）
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

import numpy as np
import mujoco
from humanoid_human_following.sim_core import (
    FollowController, CONTROL_HZ, D_KEEP, V_MAX, W_MAX)


class TestFollowController(unittest.TestCase):
    def test_converge_when_target_static(self):
        """目标静止时，机器人应追近并保持到期望跟随距离附近。"""
        ctrl = FollowController()
        jid = ctrl.model.body_jntadr[ctrl.target_body.id]
        j0 = ctrl.model.jnt_qposadr[jid]
        ctrl.data.qpos[j0:j0+3] = [3.0, 0.0, 0.3]
        ctrl.freeze = True
        dt = 1.0 / CONTROL_HZ
        n_sub = max(1, int(round(dt / ctrl.model.opt.timestep)))
        d = 0.0
        for _ in range(int(8 * CONTROL_HZ)):
            d, _, _, _ = ctrl.follow_step(dt)
            for _ in range(n_sub):
                mujoco.mj_step(ctrl.model, ctrl.data)
        self.assertLess(abs(d - D_KEEP), 0.25,
                        msg='距离未收敛到期望值: d=%.2f' % d)

    def test_control_bounded(self):
        """控制量（速度/转向）应被限幅。"""
        ctrl = FollowController()
        dt = 1.0 / CONTROL_HZ
        _, v, w, _ = ctrl.follow_step(dt)
        self.assertLessEqual(abs(v), V_MAX + 1e-6)
        self.assertLessEqual(abs(w), W_MAX + 1e-6)


if __name__ == '__main__':
    unittest.main(verbosity=2)
