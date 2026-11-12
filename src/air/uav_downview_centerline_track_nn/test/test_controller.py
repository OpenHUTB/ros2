"""循迹控制器闭环单元测试。"""

import numpy as np

from uav_downview_centerline_track_nn.controller import TrackController, TrackState
from uav_downview_centerline_track_nn.perception import DownviewCamera
from uav_downview_centerline_track_nn.policy import TrackPolicyNetwork
from uav_downview_centerline_track_nn import track

DT = 1.0 / 50.0


def _run(steps: int):
    cam = DownviewCamera()
    policy = TrackPolicyNetwork()
    controller = TrackController()
    state = TrackState(x=0.0, y=1.2, yaw=0.0)
    errors = []
    for _ in range(steps):
        percept = cam.observe(state)
        speed = float(np.hypot(state.vx, state.vy))
        feat = DownviewCamera.normalize(percept["offset"], percept["heading"], speed)
        out = policy.forward(feat)
        state = controller.step(state, out, DT)
        errors.append(abs(track.lateral_error(state.x, state.y)))
    return state, errors


def test_closed_loop_reduces_lateral_error():
    _, errors = _run(400)
    early = np.mean(errors[:20])
    late = np.mean(errors[-20:])
    assert late < early


def test_step_does_not_mutate_input():
    controller = TrackController()
    state = TrackState(x=1.0, y=0.5, yaw=0.2)
    before = (state.x, state.y, state.yaw)
    controller.step(state, np.array([0.5, 0.0]), DT)
    assert (state.x, state.y, state.yaw) == before


def test_yaw_kept_in_range():
    state, _ = _run(400)
    assert -np.pi <= state.yaw <= np.pi
