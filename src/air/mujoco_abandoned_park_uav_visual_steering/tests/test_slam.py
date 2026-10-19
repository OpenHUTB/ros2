from pathlib import Path
import sys

MODULE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(MODULE_ROOT))

from navigation import OccupancyGrid
from slam import IncrementalSlam, RangeHit


def test_slam_integrates_pose_and_range_scan():
    slam = IncrementalSlam(OccupancyGrid(20, 20), cell_size=0.5)
    slam.update_scan([RangeHit(2.0, 0.0)])
    slam.integrate_odometry(1.0, 0.0, 0.0)
    slam.update_scan([RangeHit(1.0, 1.57079632679)])
    assert len(slam.trajectory) == 2
    assert len(slam.grid.occupied) == 2
