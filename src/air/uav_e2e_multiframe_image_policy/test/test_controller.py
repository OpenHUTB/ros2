"""运动控制器测试。"""

import math

from uav_e2e_multiframe_image_policy.controller import ImagePolicyController


def test_forward_moves_along_x():
    ctl = ImagePolicyController(x=0.0, y=0.0, yaw=0.0)
    ctl.update(0.8, 0.0, 0.1)
    assert ctl.x > 0.0
    assert abs(ctl.y) < 1e-9


def test_yaw_command_clamped():
    ctl = ImagePolicyController()
    ctl.update(0.5, 5.0, 0.1)
    assert abs(ctl.yaw) <= 1.20 * 0.1 + 1e-9
