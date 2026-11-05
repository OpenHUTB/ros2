"""
humanoid_visual_servo_grasp 独立回归测试
=======================================
不依赖 ROS，直接驱动 sim_core 验证视觉伺服抓取核心逻辑。

运行:
    python3 test/test_standalone.py
    python3 test/test_standalone.py --test   # pytest 方式
"""

import numpy as np
import mujoco

from humanoid_visual_servo_grasp.sim_core import (
    ServoGraspController, CONTROL_HZ, REACH_TOL)


def _run(ctrl):
    """推进抓取过程，返回 (最终误差, 是否到达, 是否抓取完成)。"""
    dt = 1.0 / CONTROL_HZ
    n_sub = max(1, int(round(dt / ctrl.model.opt.timestep)))
    err_final = None
    for _ in range(int(6 * CONTROL_HZ)):
        err, reached, grasped = ctrl.servo_step(dt)
        for _ in range(n_sub):
            mujoco.mj_step(ctrl.model, ctrl.data)
        err_final = err
        if reached:
            return err_final, reached, grasped
    return err_final, ctrl.reached, ctrl.grasped


def test_reach_target():
    """末端应渐进收敛到目标物位置，并在容差内判定到达。"""
    ctrl = ServoGraspController()
    err, reached, _ = _run(ctrl)
    assert reached, '末端未判定到达目标'
    assert err < REACH_TOL, '末端误差 %.3f 未进入容差 %.3f' % (err, REACH_TOL)


def test_grasp_closed():
    """到达目标后，两指夹具应合拢完成抓取。"""
    ctrl = ServoGraspController()
    _, _, grasped = _run(ctrl)
    assert grasped, '抓取未完成（夹具未合拢）'
    fl = ctrl._joint_angle('finger_L_j')
    fr = ctrl._joint_angle('finger_R_j')
    assert abs(fl) > 0.01 or abs(fr) > 0.01, '夹具未实际合拢'


if __name__ == '__main__':
    import sys
    if '--test' in sys.argv:
        test_reach_target()
        test_grasp_closed()
        print('所有独立回归测试通过')
    else:
        print('运行方式: python3 test/test_standalone.py --test')
