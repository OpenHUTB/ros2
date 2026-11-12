"""数据集按场景记录，且由实际测距闭环仿真生成。"""

import numpy as np

from facade.dataset import collect_scenarios


def test_collection_saves_finite_scenario_samples(tmp_path):
    scenario = {"style": "grid", "distance": 4., "offset": .25}
    report = collect_scenarios(tmp_path, [scenario], sample_every=20, duration=36.)
    data = np.load(tmp_path / "samples.npz")
    assert data["features"].shape[1] == 15
    assert data["targets"].shape[1] == 3
    assert np.isfinite(data["features"]).all()
    assert report["episodes"][0]["waypoints_completed"] > 0
