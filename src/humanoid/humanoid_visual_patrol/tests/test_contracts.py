import os
from pathlib import Path
import sys

import numpy as np
import pytest

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from patrol.vision import mask_command


def test_no_floor_does_not_command_forward_motion():
    assert mask_command(np.zeros((96,128),dtype=np.uint8))[0] == 0


def test_unilateral_floor_steers_into_visible_free_space():
    left = np.zeros((96,128),dtype=np.uint8)
    left[:,:60] = 1
    assert mask_command(left)[1] > 0
    assert mask_command(left[:,::-1])[1] < 0


def test_bad_neural_estimates_cannot_change_primary_astar_optimal_cost():
    from patrol.planning import astar,distances
    rng=np.random.default_rng(93)
    grid=np.zeros((16,16),dtype=bool)
    grid[2:14,7]=True
    grid[10,7]=False
    start,goal=(4,3),(4,12)
    expected=distances(grid,goal)[start]
    route,_=astar(grid,start,goal,rng.normal(0,10000,grid.shape))
    assert len(route)-1==expected
    assert all(not grid[p] for p in route)


def test_planner_reports_unreachable_goal():
    from patrol.planning import astar
    grid=np.zeros((10,10),dtype=bool);grid[5,:]=True
    with pytest.raises(ValueError,match='No path'):
        astar(grid,(2,2),(8,8))


@pytest.mark.skipif(not os.environ.get('HUMANOID_ASSETS'),reason='set HUMANOID_ASSETS for real physics checks')
def test_real_robot_stays_upright_and_scan_excludes_body():
    from patrol.scene import IndoorSimulator
    sim = IndoorSimulator(os.environ['HUMANOID_ASSETS'])
    try:
        # Robot starts 1.88m from side walls; there must be no 7cm self-return.
        assert min(sim.scan()) > 1.0
        result = sim.smoke_test()
        assert not result['fallen']
        assert result['displacement_m'] > 1
        assert result['final_yaw_rad'] > .3
    finally:
        sim.close()
