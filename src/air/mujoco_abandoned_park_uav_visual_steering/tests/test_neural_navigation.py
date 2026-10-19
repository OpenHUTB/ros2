from navigation import OccupancyGrid
from neural_navigation import NeuralPlanner
from slam import RangeHit, SynchronizedSlamNavigation

def test_mlp_policy_emits_control_from_map_features():
    action = NeuralPlanner().predict(OccupancyGrid(width=12, height=12), (6, 6), (10, 6))
    assert action.label in NeuralPlanner.labels

def test_slam_and_neural_planner_are_updated_in_one_step():
    loop = SynchronizedSlamNavigation(OccupancyGrid(width=12, height=12), (10, 6))
    action = loop.step(0.5, 0.0, 0.0, [RangeHit(1.0, 0.0)])
    assert len(loop.slam.trajectory) == 2
    assert action.label in NeuralPlanner.labels
