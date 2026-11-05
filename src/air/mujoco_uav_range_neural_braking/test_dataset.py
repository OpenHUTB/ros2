"""检查传感器闭环与回合划分。 Check sensor control and episode-level splits."""
import csv
import hashlib

import pytest

from dataset import generate
from simulation import Scenario, Simulation


def test_sensor_ray_and_installation_offset():
    sim = Simulation(controller="range")
    distance, closing, altitude, valid = sim.sense()
    assert valid
    assert distance == pytest.approx(6.49)
    assert distance + 0.01 == pytest.approx(sim.observe()[0])
    assert closing == 0
    assert altitude == pytest.approx(1.2)


def test_range_control_does_not_use_geometric_truth():
    sim = Simulation(controller="range")
    sim.observe = lambda: (0.1, 50.0, 1.2)
    # 若控制误用几何真值，将产生最大制动；测距控制应保持前进。
    # Using geometric truth here would brake; sensor control should accelerate.
    assert sim.step()["accel_m_s2"] > 0


def test_missing_range_brakes():
    sim = Simulation(controller="range")
    sim.data.sensor("front_range").data[0] = -1
    row = sim.step()
    assert row["sensor_valid"] == 0
    assert row["accel_m_s2"] == -2.5


@pytest.mark.parametrize("speed", [0.0, 1.5, 3.0])
def test_sensor_braking(speed):
    sim = Simulation(Scenario(speed=speed), controller="range")
    _, report = sim.run()
    assert report["collision_steps"] == 0
    assert report["invalid_sensor_steps"] == 0
    assert report["min_clearance_m"] > 0.3
    assert abs(report["final_speed_m_s"]) < 0.02


def test_splits_are_disjoint_and_reproducible(tmp_path):
    first = generate(tmp_path / "first", episodes=5)
    second = generate(tmp_path / "second", episodes=5)
    all_ids = []
    for name, split in first["splits"].items():
        path = tmp_path / "first" / split["file"]
        assert hashlib.sha256(path.read_bytes()).hexdigest() == split["sha256"]
        assert split["sha256"] == second["splits"][name]["sha256"]
        with path.open(encoding="utf-8", newline="") as stream:
            rows = list(csv.DictReader(stream))
        expected = {ep["scenario"] for ep in split["episodes"]}
        assert {row["episode_id"] for row in rows} == expected
        assert len(rows) == split["rows"]
        all_ids.extend(expected)
    assert len(all_ids) == len(set(all_ids)) == 5
