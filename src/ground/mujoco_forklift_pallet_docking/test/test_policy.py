import numpy as np

from mujoco_forklift_pallet_docking.policy import ImagePolicy, teacher_action


def test_policy_returns_bounded_throttle_and_brake():
    output = ImagePolicy().predict(np.zeros((2, 64, 64), dtype=np.uint8))
    assert output.shape == (2, 2)
    assert np.all((output >= 0.0) & (output <= 1.0))


def test_teacher_brakes_before_stopping_distance():
    assert teacher_action(0.0, 2.0)[1] == 1.0
    assert teacher_action(5.0, 2.0)[0] == 1.0


def test_camera_observes_pallet_from_different_distances():
    from mujoco_forklift_pallet_docking.simulation import ForkliftDockingSimulation

    far = ForkliftDockingSimulation(start_x=-5.0)
    near = ForkliftDockingSimulation(start_x=-3.0)
    try:
        assert not np.array_equal(far.image(), near.image())
    finally:
        far.close()
        near.close()
