"""专家参考控制器测试。"""

from uav_e2e_multiframe_image_policy import scene, expert


def test_forward_bounded():
    cmd = expert.expert_command(1.0, scene.centerline_y(1.0), 0.0)
    assert expert.V_BASE - 1e-9 <= cmd["forward"] <= expert.V_MAX + 1e-9
    assert cmd["forward"] > 0.0


def test_positive_offset_calls_for_right_correction():
    cmd = expert.expert_command(1.0, scene.centerline_y(1.0) + 0.4, 0.0)
    assert cmd["yaw_cmd"] < 0.0
