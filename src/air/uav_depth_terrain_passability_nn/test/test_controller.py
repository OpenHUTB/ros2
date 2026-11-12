"""地形通行控制器单元测试。"""

import numpy as np

from uav_depth_terrain_passability_nn.controller import TerrainController, TerrainState
from uav_depth_terrain_passability_nn.depth_sensor import DepthTerrainSensor
from uav_depth_terrain_passability_nn.policy import PassabilityNetwork
from uav_depth_terrain_passability_nn import terrain


def test_closed_loop_stays_in_corridor():
    sensor = DepthTerrainSensor()
    net = PassabilityNetwork()
    ctrl = TerrainController()
    state = TerrainState(x=0.0, y=0.0, yaw=0.0)
    for _ in range(50 * 12):
        percept = sensor.observe(state)
        feat = DepthTerrainSensor.normalize(percept["features"])
        decision = net.forward(feat)
        state = ctrl.step(state, decision, 1.0 / 50.0)
    assert abs(state.y) < 0.5
    assert not terrain.is_blocked(state.x, state.y)


def test_step_does_not_mutate_input():
    ctrl = TerrainController()
    state = TerrainState(x=1.0, y=0.2, yaw=0.1)
    decision = {"sector": 1, "passability": np.array([0.2, 0.6, 0.2]), "speed": 0.4}
    ctrl.step(state, decision, 1.0 / 50.0)
    assert state.x == 1.0 and state.y == 0.2 and state.yaw == 0.1


def test_yaw_kept_in_range():
    ctrl = TerrainController()
    state = TerrainState(x=0.0, y=0.0, yaw=0.0)
    decision = {"sector": 0, "passability": np.array([0.6, 0.2, 0.2]), "speed": 0.4}
    for _ in range(200):
        state = ctrl.step(state, decision, 1.0 / 50.0)
    assert -np.pi <= state.yaw <= np.pi
