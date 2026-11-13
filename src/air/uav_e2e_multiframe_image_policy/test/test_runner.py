"""端到端闭环仿真测试。"""

from uav_e2e_multiframe_image_policy.runner import E2ERunner


def test_closed_loop_reaches_far_end():
    runner = E2ERunner()
    runner.run()
    assert runner.step_count > 0
    assert runner.metrics["x_reached"] > 8.0


def test_closed_loop_lateral_bounded():
    runner = E2ERunner()
    runner.run()
    assert runner.metrics["max_abs_lateral"] < 1.0
    assert runner.metrics["max_speed"] <= 0.90 + 1e-9
