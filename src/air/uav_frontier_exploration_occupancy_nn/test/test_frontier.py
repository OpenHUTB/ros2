"""前沿聚类与候选特征测试。"""

from uav_frontier_exploration_occupancy_nn.occupancy_map import OccupancyMap
from uav_frontier_exploration_occupancy_nn.frontier import (
    cluster_frontiers, extract_candidates)


def test_cluster_groups_adjacent():
    m = OccupancyMap()
    m.raycast(0.5, 0.5, 0.0, 1.5)
    clusters = cluster_frontiers(m)
    assert len(clusters) >= 1
    flat = [cell for cluster in clusters for cell in cluster]
    assert len(flat) == len(set(flat))


def test_candidates_features_in_range():
    m = OccupancyMap()
    m.raycast(0.5, 0.5, 0.0, 1.5)
    candidates = extract_candidates(m, 0.5, 0.5, 0.0)
    assert len(candidates) >= 1
    for cand in candidates:
        feat = cand["features"]
        assert len(feat) == 4
        assert -1.0 <= feat[0] <= 1.0
        assert -1.0 <= feat[1] <= 1.0
        assert 0.0 <= feat[2] <= 1.0
        assert 0.0 <= feat[3] <= 1.0
