from pathlib import Path
from math import hypot

import pytest

mujoco = pytest.importorskip('mujoco')

from mujoco_musculoskeletal_arm_reach.controller import (
    ArmControlState,
    MuscleArmController,
)
from mujoco_musculoskeletal_arm_reach.simulation import ArmSimulation


def test_mujoco_model_loads_and_steps_safely():
    model_path = Path(__file__).parents[1] / 'models' / 'arm.xml'
    simulation = ArmSimulation(model_path)
    assert simulation.model.nq == 2
    assert simulation.model.nu == 6
    observation = simulation.reset(0.15, 1.0)
    assert observation.shoulder_rad == pytest.approx(0.15)
    assert observation.elbow_rad == pytest.approx(1.0)
    observation = simulation.step((0.03, 0.03, 0.03, 0.03, 0.03, 0.03))
    assert observation.time_s > 0.0
    assert len(observation.muscle_forces_n) == 6
    simulation.safe_stop()


def test_simulation_rejects_invalid_activation_count():
    model_path = Path(__file__).parents[1] / 'models' / 'arm.xml'
    simulation = ArmSimulation(model_path)
    with pytest.raises(ValueError):
        simulation.step((0.1, 0.1))


@pytest.mark.parametrize(
    'target',
    ((0.38, -0.17), (0.30, -0.10), (0.18, -0.32)),
)
def test_closed_loop_reaches_target(target):
    model_path = Path(__file__).parents[1] / 'models' / 'arm.xml'
    simulation = ArmSimulation(model_path)
    controller = MuscleArmController()
    simulation.reset(-0.25, 0.75)
    for _ in range(1600):
        observation = simulation.observe()
        command = controller.command(
            ArmControlState(
                target_x_m=target[0],
                target_z_m=target[1],
                shoulder_rad=observation.shoulder_rad,
                elbow_rad=observation.elbow_rad,
                shoulder_velocity_rad_s=(
                    observation.shoulder_velocity_rad_s
                ),
                elbow_velocity_rad_s=observation.elbow_velocity_rad_s,
            )
        )
        observation = simulation.step(command)
    error = hypot(
        observation.end_effector_x_m - target[0],
        observation.end_effector_z_m - target[1],
    )
    assert error < 0.06
