"""验证制动与高度响应。 Verify braking and altitude response."""
import mujoco
import pytest

from simulation import Scenario, Simulation


@pytest.mark.parametrize("speed", [0.0, 1.5, 3.0])
def test_braking(speed):
    sim = Simulation(Scenario(speed=speed))
    _, report = sim.run()
    assert report["collision_steps"] == 0
    assert sim.data.ncon == 0
    assert report["min_clearance_m"] > 0.4
    assert abs(report["final_speed_m_s"]) < 0.02


def test_height_recovers_after_displacement():
    sim = Simulation()
    sim.data.qpos[1] = 0.1
    mujoco.mj_forward(sim.model, sim.data)
    sim.run()
    assert abs(sim.observe()[2] - sim.scenario.height) < 0.005


@pytest.mark.parametrize("changes", [{"speed": float("nan")}, {"duration": 0}, {"height": 0.1}])
def test_invalid_scenario(changes):
    with pytest.raises(ValueError):
        Scenario(**changes)
