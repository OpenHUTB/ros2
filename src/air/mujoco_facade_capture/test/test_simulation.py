"""实际物理积分、测距与扫描进度验证。"""

import numpy as np

from facade.scene import ray_features
from facade.simulation import FacadeSimulation, run_scan


def test_camera_range_geometry_and_force_driven_motion():
    sim = FacadeSimulation()
    try:
        np.testing.assert_allclose(ray_features(sim.ranges()), 4., atol=.03)
        start = sim.position.copy()
        for _ in range(100):
            sim.step(np.array([.5, 0., 0.]))
        assert sim.position[0] > start[0] + .25
        assert abs(sim.position[2] - start[2]) < .02
    finally:
        sim.close()


def test_scan_reaches_all_waypoints(tmp_path):
    metrics = run_scan(tmp_path, capture=False)
    assert metrics["completion_ratio"] == 1.
    assert metrics["minimum_wall_distance_m"] > 3.8
    assert (tmp_path / "trajectory.csv").exists()
