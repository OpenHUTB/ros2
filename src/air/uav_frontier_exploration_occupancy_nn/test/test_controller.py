"""导航控制器测试。"""

from uav_frontier_exploration_occupancy_nn.controller import ExplorerController


def test_advance_toward_target():
    ctrl = ExplorerController(x=0.0, y=0.0, yaw=0.0)
    out = ctrl.update(1.0, 0.0, 1.0 / 20.0,
                      {"ahead": True, "left": True, "right": True})
    assert out["reached"] is False
    assert ctrl.x > 0.0
    assert abs(ctrl.yaw) < 0.2


def test_blocked_forward_keeps_position():
    ctrl = ExplorerController(x=0.0, y=0.0, yaw=0.0)
    out = ctrl.update(1.0, 0.0, 1.0 / 20.0,
                      {"ahead": False, "left": True, "right": False})
    assert out["speed"] == 0.0
    assert ctrl.x == 0.0
    assert ctrl.y == 0.0
